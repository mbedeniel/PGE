"""Imitate the ROS 2 interface of MotoROS2 in front of a simulated Yaskawa arm.

Run in the robot namespace (e.g. /yaskawa_LEFT). Same names and types as MotoROS2 on the
real robots, so that client code (scripts, MoveIt) works unchanged in simulation:

  action   follow_joint_trajectory  control_msgs/action/FollowJointTrajectory
  topic    robot_status             industrial_msgs/msg/RobotStatus
  services start_traj_mode          motoros2_interfaces/srv/StartTrajMode
           start_point_queue_mode   motoros2_interfaces/srv/StartPointQueueMode
           stop_traj_mode           std_srvs/srv/Trigger
           reset_error              motoros2_interfaces/srv/ResetError
           read_single_io           motoros2_interfaces/srv/ReadSingleIO
           write_single_io          motoros2_interfaces/srv/WriteSingleIO

  topic    joint_states             sensor_msgs/msg/JointState (reliable: received by every
                                    subscriber, best-effort ones included)

joint_states is the simulated arm's state from joint_state_broadcaster (local topic
joint_state_broadcaster/joint_states), republished under the MotoROS2 name, as on the real arms
where MotoROS2 publishes it and ros2_control republishes it on the local topic.
Goals are checked like MotoROS2 does (trajectory mode, 2 to max_points points, all joints,
velocities, increasing times, zero final velocity, start at the current position), then
forwarded to the joint_trajectory_controller of the simulation, to start at once as MotoROS2 does.

Start-up, as on the real arms (see motion_mode.py): nothing moves before a motion mode is started,
and only one mode is active at a time.
- Trajectory mode (start_traj_mode): MotoROS2 executes follow_joint_trajectory goals itself. In
  simulation the goals go through arm_controller, so start_traj_mode activates it when it is inactive
  and stop_traj_mode deactivates it again (parameter activate_controller_in_traj_mode).
- Point-queue mode (start_point_queue_mode): on the real arms the ros2_control hardware interface
  sends one point per control cycle, and arm_controller is activated by hand afterwards. Here the
  service only records the mode; activating arm_controller is left to the user, as on the real arms.
  stop_traj_mode is refused while arm_controller is active, as MotoROS2 refuses it while points are
  still being sent. queue_traj_point itself is not imitated: in simulation the controller writes to
  the simulated joints directly.
Simulation cannot refuse motion: arm_controller activated without start_point_queue_mode still moves
the simulated arm, whereas the real arm refuses every point.
"""

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from industrial_msgs.msg import RobotMode, RobotStatus, TriState
from controller_manager_msgs.srv import ListControllers, SwitchController
from motoros2_interfaces.msg import InitTrajEnum, IoResultCodes, MotionReadyEnum
from motoros2_interfaces.srv import (
    ReadSingleIO,
    ResetError,
    StartPointQueueMode,
    StartTrajMode,
    WriteSingleIO,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

from mfja_yaskawa_motoros2_sim import motion_mode as mm
from mfja_yaskawa_motoros2_sim import trajectory_checks as checks

MODE_MESSAGES = {
    mm.READY: '',
    mm.NOT_READY_OTHER_TRAJ_MODE_ACTIVE: MotionReadyEnum.NOT_READY_OTHER_TRAJ_MODE_ACTIVE_STR,
    mm.NOT_READY_ERROR: 'The controller has an active Error (simulated). Call reset_error.',
}


class MotoROS2Sim(Node):

    def __init__(self):
        super().__init__('motoros2_sim')
        self.declare_parameter('joint_names', [f'joint_{i}' for i in range(1, 7)])
        self.declare_parameter('controller_action', 'arm_controller/follow_joint_trajectory')
        self.declare_parameter('max_points', 200)
        self.declare_parameter('start_tolerance', 0.01)
        self.declare_parameter('check_goals', True)
        self.declare_parameter('status_rate', 10.0)
        self.declare_parameter('controller_name', 'arm_controller')
        self.declare_parameter('controller_manager', 'controller_manager')
        self.declare_parameter('activate_controller_in_traj_mode', True)
        self.declare_parameter('gripper_init_address', 10133)   # OT#100, to confirm on the pendant
        self.declare_parameter('gripper_open_address', 10134)   # OT#101
        self.declare_parameter('gripper_close_address', 10135)  # OT#102

        self._joints = list(self.get_parameter('joint_names').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._start_tolerance = float(self.get_parameter('start_tolerance').value)
        self._check_goals = bool(self.get_parameter('check_goals').value)
        self._controller_name = self.get_parameter('controller_name').value
        controller_manager = self.get_parameter('controller_manager').value.rstrip('/')
        self._traj_mode_activates = bool(
            self.get_parameter('activate_controller_in_traj_mode').value)
        self._gripper = {
            int(self.get_parameter('gripper_init_address').value): 'init',
            int(self.get_parameter('gripper_open_address').value): 'open',
            int(self.get_parameter('gripper_close_address').value): 'close',
        }

        self._lock = threading.Lock()
        self._mode = None                     # None, mm.TRAJECTORY or mm.POINT_QUEUE
        self._activated_by_traj_mode = False  # arm_controller activated by start_traj_mode
        self._in_error = False
        self._error_codes = []
        self._in_motion = False
        self._positions = {}
        self._io = {}
        self._active_client_goal = None

        group = ReentrantCallbackGroup()
        self._joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(JointState, 'joint_state_broadcaster/joint_states',
                                 self._on_joint_states, 10, callback_group=group)
        self._status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.create_timer(1.0 / float(self.get_parameter('status_rate').value),
                          self._publish_status, callback_group=group)

        self.create_service(StartTrajMode, 'start_traj_mode', self._start_traj_mode,
                            callback_group=group)
        self.create_service(StartPointQueueMode, 'start_point_queue_mode',
                            self._start_point_queue_mode, callback_group=group)
        self.create_service(Trigger, 'stop_traj_mode', self._stop_traj_mode, callback_group=group)
        self.create_service(ResetError, 'reset_error', self._reset_error, callback_group=group)
        self.create_service(ReadSingleIO, 'read_single_io', self._read_single_io,
                            callback_group=group)
        self.create_service(WriteSingleIO, 'write_single_io', self._write_single_io,
                            callback_group=group)

        self._list_client = self.create_client(
            ListControllers, f'{controller_manager}/list_controllers', callback_group=group)
        self._switch_client = self.create_client(
            SwitchController, f'{controller_manager}/switch_controller', callback_group=group)
        self._client = ActionClient(
            self, FollowJointTrajectory, self.get_parameter('controller_action').value,
            callback_group=group)
        self._server = ActionServer(
            self, FollowJointTrajectory, 'follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=self._cancel,
            callback_group=group)

        self.get_logger().info(
            f'MotoROS2 imitation ready in {self.get_namespace()}: '
            f'joints {self._joints}, at most {self._max_points} points per trajectory. '
            'As on the real robot, call start_traj_mode (MotoROS2 trajectories) or '
            f'start_point_queue_mode (then activate {self._controller_name}) before any motion.')

    # ----------------------------------------------------------------- state
    def _on_joint_states(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._positions[name] = position
        self._joint_states_pub.publish(msg)

    def _publish_status(self):
        def tri(value):
            return TriState(val=TriState.TRUE if value else TriState.FALSE)

        with self._lock:
            msg = RobotStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.mode = RobotMode(val=RobotMode.AUTO)
            msg.e_stopped = tri(False)
            msg.drives_powered = tri(self._mode is not None)
            msg.motion_possible = tri(self._mode is not None and not self._in_error)
            msg.in_motion = tri(self._in_motion)
            msg.in_error = tri(self._in_error)
            msg.error_codes = list(self._error_codes)
        self._status_pub.publish(msg)

    # -------------------------------------------------------------- services
    async def _controller_state(self):
        """State of the trajectory controller ('active', 'inactive', ...), or None if unknown."""
        if not self._list_client.wait_for_service(timeout_sec=2.0):
            return None
        response = await self._list_client.call_async(ListControllers.Request())
        for controller in response.controller:
            if controller.name == self._controller_name:
                return controller.state
        return None

    async def _set_controller_active(self, active):
        """Activate or deactivate the trajectory controller. Return (changed, error message)."""
        state = await self._controller_state()
        if state is None:
            return False, f'{self._controller_name} is not loaded (controller manager not reached).'
        if (state == 'active') == active:
            return False, ''
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            return False, 'switch_controller service not available.'
        request = SwitchController.Request()
        if active:
            request.activate_controllers = [self._controller_name]
        else:
            request.deactivate_controllers = [self._controller_name]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 5
        response = await self._switch_client.call_async(request)
        if not response.ok:
            verb = 'activate' if active else 'deactivate'
            return False, f'Could not {verb} {self._controller_name}.'
        return True, ''

    def _answer_start(self, service, response, code, message=None):
        response.result_code = MotionReadyEnum(value=code)
        response.message = MODE_MESSAGES.get(code, '') if message is None else message
        # One logging call per severity: rclpy refuses two severities from the same line.
        if code == mm.READY:
            self.get_logger().info(f'{service} -> {code} READY')
        else:
            self.get_logger().warning(f'{service} -> {code} {response.message}')
        return response

    async def _start_traj_mode(self, _request, response):
        with self._lock:
            code = mm.start_result(self._mode, mm.TRAJECTORY, self._in_error)
            already = self._mode == mm.TRAJECTORY
        if code != mm.READY:
            return self._answer_start('start_traj_mode', response, code)
        if self._traj_mode_activates and not already:
            # Trajectory mode: MotoROS2 executes the goals itself; here they go through
            # the simulated controller, which must therefore be active.
            changed, error = await self._set_controller_active(True)
            if error:
                return self._answer_start('start_traj_mode', response,
                                          MotionReadyEnum.NOT_READY_UNSPECIFIED, error)
            self._activated_by_traj_mode = changed
        with self._lock:
            self._mode = mm.TRAJECTORY
        return self._answer_start('start_traj_mode', response, mm.READY)

    def _start_point_queue_mode(self, _request, response):
        with self._lock:
            code = mm.start_result(self._mode, mm.POINT_QUEUE, self._in_error)
            if code == mm.READY:
                self._mode = mm.POINT_QUEUE
        self._answer_start('start_point_queue_mode', response, code)
        if code == mm.READY:
            self.get_logger().info(
                f'Point-queue mode: activate {self._controller_name} to move the arm, as on the '
                'real arms.')
        return response

    async def _stop_traj_mode(self, _request, response):
        with self._lock:
            mode, in_motion = self._mode, self._in_motion
        streaming = mode == mm.POINT_QUEUE and (await self._controller_state()) == 'active'
        if mm.stop_refused(mode, in_motion, streaming):
            response.success = False
            response.message = mm.QUEUE_NOT_EMPTY
            hint = (f' Deactivate {self._controller_name} first (on the real arms, stop the '
                    'ros2_control launch: its hardware interface keeps sending points).'
                    if streaming else ' Wait for the end of the motion.')
            self.get_logger().warning(f'stop_traj_mode refused: {response.message}.{hint}')
            return response
        if mode == mm.TRAJECTORY and self._activated_by_traj_mode:
            _changed, error = await self._set_controller_active(False)
            if error:
                self.get_logger().warning(f'stop_traj_mode: {error}')
            self._activated_by_traj_mode = False
        with self._lock:
            self._mode = None
        response.success = True
        response.message = ''
        self.get_logger().info(f'stop_traj_mode: {mode or "no"} mode stopped')
        return response

    def _reset_error(self, _request, response):
        with self._lock:
            self._in_error = False
            self._error_codes = []
        response.result_code = MotionReadyEnum(value=MotionReadyEnum.READY)
        response.message = 'Errors reset (simulated).'
        return response

    def _read_single_io(self, request, response):
        if not checks.io_address_valid(request.address):
            response.result_code = IoResultCodes.READ_ADDRESS_INVALID
            response.message = IoResultCodes.READ_ADDRESS_INVALID_STR
            response.success = False
            return response
        with self._lock:
            response.value = int(self._io.get(request.address, 0))
        response.result_code = IoResultCodes.OK
        response.message = IoResultCodes.OK_STR
        response.success = True
        return response

    def _write_single_io(self, request, response):
        if not checks.io_address_writable(request.address):
            response.result_code = IoResultCodes.WRITE_ADDRESS_INVALID
            response.message = IoResultCodes.WRITE_ADDRESS_INVALID_STR
            response.success = False
            return response
        if request.value not in (0, 1):
            response.result_code = IoResultCodes.WRITE_VALUE_INVALID
            response.message = IoResultCodes.WRITE_VALUE_INVALID_STR
            response.success = False
            return response
        with self._lock:
            self._io[request.address] = request.value
        action = self._gripper.get(request.address)
        if action and request.value == 1:
            self.get_logger().info(
                f'Gripper "{action}" requested on IO {request.address} '
                '(gripper not simulated yet: only the IO value is stored).')
        response.result_code = IoResultCodes.OK
        response.message = IoResultCodes.OK_STR
        response.success = True
        return response

    # ---------------------------------------------------------------- action
    def _cancel(self, _goal_handle):
        client_goal = self._active_client_goal
        if client_goal is not None:
            client_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _abort(self, goal_handle, code, message):
        self.get_logger().warning(f'Trajectory refused: {message} (error_code {code})')
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        return result

    async def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        with self._lock:
            mode, in_motion = self._mode, self._in_motion
            positions = dict(self._positions)
        refusal = mm.goal_refusal(mode)
        if refusal == mm.NOT_READY_SERVO_OFF:
            return self._abort(goal_handle, checks.motoros2_error_code(refusal),
                               MotionReadyEnum.NOT_READY_SERVO_OFF_STR)
        if refusal == mm.INIT_TRAJ_WRONG_MODE:
            return self._abort(goal_handle, checks.motoros2_error_code(refusal),
                               InitTrajEnum.INIT_TRAJ_WRONG_MODE_STR)
        if in_motion:
            return self._abort(goal_handle, checks.motoros2_error_code(
                checks.INIT_TRAJ_ALREADY_IN_MOTION), 'Already running a trajectory.')
        if self._check_goals:
            code, message = checks.check_trajectory(
                trajectory.joint_names, trajectory.points, self._joints,
                positions or None, self._max_points, self._start_tolerance)
            if code != checks.INIT_TRAJ_OK:
                return self._abort(goal_handle, checks.motoros2_error_code(code), message)

        if not self._client.wait_for_server(timeout_sec=5.0):
            return self._abort(goal_handle, checks.motoros2_error_code(
                MotionReadyEnum.NOT_READY_UNSPECIFIED),
                'The simulated trajectory controller is not available.')

        # MotoROS2 starts a trajectory as soon as it is received: of the goal fields, only
        # goal_tolerance and goal_time_tolerance are used, header.stamp is ignored. The
        # joint_trajectory_controller would instead wait until header.stamp, which clients
        # written for the real robot fill with wall-clock time, far ahead of simulated time.
        request = goal_handle.request
        request.trajectory.header.stamp.sec = 0
        request.trajectory.header.stamp.nanosec = 0

        with self._lock:
            self._in_motion = True
        try:
            client_goal = await self._client.send_goal_async(
                request,
                feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback))
            if not client_goal.accepted:
                return self._abort(goal_handle, checks.motoros2_error_code(
                    MotionReadyEnum.NOT_READY_UNSPECIFIED),
                    'The simulated trajectory controller rejected the goal.')
            self._active_client_goal = client_goal
            response = await client_goal.get_result_async()
        finally:
            self._active_client_goal = None
            with self._lock:
                self._in_motion = False

        result = response.result
        if response.status == GoalStatus.STATUS_SUCCEEDED and result.error_code == 0:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MotoROS2Sim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
