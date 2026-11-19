"""Imitate the ROS 2 interface of MotoROS2 in front of a simulated Yaskawa arm.

Run in the robot namespace (e.g. /yaskawa_LEFT). Same names and types as MotoROS2 on the
real robots, so that client code (scripts, MoveIt) works unchanged in simulation:

  action   follow_joint_trajectory  control_msgs/action/FollowJointTrajectory
  topic    robot_status             industrial_msgs/msg/RobotStatus
  services start_traj_mode          motoros2_interfaces/srv/StartTrajMode
           stop_traj_mode           std_srvs/srv/Trigger
           reset_error              motoros2_interfaces/srv/ResetError
           read_single_io           motoros2_interfaces/srv/ReadSingleIO
           write_single_io          motoros2_interfaces/srv/WriteSingleIO

joint_states is published by the joint_state_broadcaster of the simulation.
Goals are checked like MotoROS2 does (trajectory mode, 2 to max_points points, all joints,
velocities, increasing times, zero final velocity, start at the current position), then
forwarded to the joint_trajectory_controller of the simulation.
"""

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from industrial_msgs.msg import RobotMode, RobotStatus, TriState
from motoros2_interfaces.msg import IoResultCodes, MotionReadyEnum
from motoros2_interfaces.srv import (
    ReadSingleIO,
    ResetError,
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

from mfja_yaskawa_motoros2_sim import trajectory_checks as checks


class MotoROS2Sim(Node):

    def __init__(self):
        super().__init__('motoros2_sim')
        self.declare_parameter('joint_names', [f'joint_{i}' for i in range(1, 7)])
        self.declare_parameter('controller_action', 'arm_controller/follow_joint_trajectory')
        self.declare_parameter('max_points', 200)
        self.declare_parameter('start_tolerance', 0.01)
        self.declare_parameter('check_goals', True)
        self.declare_parameter('status_rate', 10.0)
        self.declare_parameter('gripper_init_address', 10133)   # OT#100, to confirm on the pendant
        self.declare_parameter('gripper_open_address', 10134)   # OT#101
        self.declare_parameter('gripper_close_address', 10135)  # OT#102

        self._joints = list(self.get_parameter('joint_names').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._start_tolerance = float(self.get_parameter('start_tolerance').value)
        self._check_goals = bool(self.get_parameter('check_goals').value)
        self._gripper = {
            int(self.get_parameter('gripper_init_address').value): 'init',
            int(self.get_parameter('gripper_open_address').value): 'open',
            int(self.get_parameter('gripper_close_address').value): 'close',
        }

        self._lock = threading.Lock()
        self._traj_mode = False
        self._in_error = False
        self._error_codes = []
        self._in_motion = False
        self._positions = {}
        self._io = {}
        self._active_client_goal = None

        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, 10,
                                 callback_group=group)
        self._status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.create_timer(1.0 / float(self.get_parameter('status_rate').value),
                          self._publish_status, callback_group=group)

        self.create_service(StartTrajMode, 'start_traj_mode', self._start_traj_mode,
                            callback_group=group)
        self.create_service(Trigger, 'stop_traj_mode', self._stop_traj_mode, callback_group=group)
        self.create_service(ResetError, 'reset_error', self._reset_error, callback_group=group)
        self.create_service(ReadSingleIO, 'read_single_io', self._read_single_io,
                            callback_group=group)
        self.create_service(WriteSingleIO, 'write_single_io', self._write_single_io,
                            callback_group=group)

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
            'Call start_traj_mode before sending a trajectory, as on the real robot.')

    # ----------------------------------------------------------------- state
    def _on_joint_states(self, msg):
        with self._lock:
            for name, position in zip(msg.name, msg.position):
                self._positions[name] = position

    def _publish_status(self):
        def tri(value):
            return TriState(val=TriState.TRUE if value else TriState.FALSE)

        with self._lock:
            msg = RobotStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.mode = RobotMode(val=RobotMode.AUTO)
            msg.e_stopped = tri(False)
            msg.drives_powered = tri(self._traj_mode)
            msg.motion_possible = tri(self._traj_mode and not self._in_error)
            msg.in_motion = tri(self._in_motion)
            msg.in_error = tri(self._in_error)
            msg.error_codes = list(self._error_codes)
        self._status_pub.publish(msg)

    # -------------------------------------------------------------- services
    def _start_traj_mode(self, _request, response):
        with self._lock:
            if self._in_error:
                code = MotionReadyEnum.NOT_READY_ERROR
                message = 'The controller has an active Error (simulated). Call reset_error.'
            else:
                self._traj_mode = True
                code, message = MotionReadyEnum.READY, 'Ready'
        response.result_code = MotionReadyEnum(value=code)
        response.message = message
        self.get_logger().info(f'start_traj_mode -> {code} {message}')
        return response

    def _stop_traj_mode(self, _request, response):
        with self._lock:
            if self._in_motion:
                response.success = False
                response.message = 'Cannot stop trajectory mode while a motion is executed.'
            else:
                self._traj_mode = False
                response.success = True
                response.message = 'Trajectory mode stopped (servos stay on).'
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
            traj_mode, in_motion = self._traj_mode, self._in_motion
            positions = dict(self._positions)
        if in_motion:
            return self._abort(goal_handle, checks.motoros2_error_code(
                checks.INIT_TRAJ_ALREADY_IN_MOTION), 'Already running a trajectory.')
        if not traj_mode:
            return self._abort(goal_handle, checks.motoros2_error_code(
                MotionReadyEnum.NOT_READY_SERVO_OFF),
                'Servo power is OFF. Please call start_traj_mode.')
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

        with self._lock:
            self._in_motion = True
        try:
            client_goal = await self._client.send_goal_async(
                goal_handle.request,
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
