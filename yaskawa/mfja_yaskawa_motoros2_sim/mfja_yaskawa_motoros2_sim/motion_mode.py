"""Motion modes of MotoROS2 0.2.x, without ROS dependencies.

MotoROS2 has two motion modes; only one can be active at a time:

  trajectory   started by start_traj_mode; used by the follow_joint_trajectory action of MotoROS2
               (the scripts and the MoveIt launch of mfja_ros2_yaskawa)
  point_queue  started by start_point_queue_mode; used by queue_traj_point, i.e. by the ros2_control
               hardware interface of the real arms (motoros2_hw_interfaces)

Starting one mode while the other is active is refused (NOT_READY_OTHER_TRAJ_MODE_ACTIVE);
starting the active mode again answers READY. stop_traj_mode ends either mode, but is refused
while points remain to be executed ("increment queue not empty"). A follow_joint_trajectory goal
is refused when no mode is active (servos off) or in point-queue mode (wrong mode).

Codes: motoros2_interfaces/msg/MotionReadyEnum.msg and InitTrajEnum.msg.
"""

TRAJECTORY = 'trajectory'
POINT_QUEUE = 'point_queue'

READY = 1
NOT_READY_ERROR = 102
NOT_READY_SERVO_OFF = 106
NOT_READY_OTHER_TRAJ_MODE_ACTIVE = 114
INIT_TRAJ_WRONG_MODE = 209

QUEUE_NOT_EMPTY = "Can't disable trajectory mode: increment queue not empty"


def start_result(active_mode, requested_mode, in_error):
    """Result code of start_traj_mode or start_point_queue_mode, checked in MotoROS2's order."""
    if active_mode is not None and active_mode != requested_mode:
        return NOT_READY_OTHER_TRAJ_MODE_ACTIVE
    if in_error:
        return NOT_READY_ERROR
    return READY


def stop_refused(active_mode, in_motion, streaming):
    """True when stop_traj_mode must be refused because points remain to be executed.

    in_motion  a follow_joint_trajectory goal is being executed (trajectory mode)
    streaming  points are being sent in point-queue mode (simulation: arm_controller active)
    """
    return in_motion or (active_mode == POINT_QUEUE and streaming)


def goal_refusal(active_mode):
    """Code refusing a follow_joint_trajectory goal because of the mode, or None."""
    if active_mode is None:
        return NOT_READY_SERVO_OFF
    if active_mode != TRAJECTORY:
        return INIT_TRAJ_WRONG_MODE
    return None
