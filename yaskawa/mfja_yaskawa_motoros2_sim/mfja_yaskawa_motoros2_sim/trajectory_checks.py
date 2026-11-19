"""Checks applied by MotoROS2 to a FollowJointTrajectory goal, without ROS dependencies.

The codes are those of motoros2_interfaces/msg/InitTrajEnum.msg (MotoROS2 0.2.x).
MotoROS2 reports them in the action result as error_code = -(100000 + code)
(ROS INVALID_GOAL = -1, followed by the 5-digit MotoROS2 code).
"""

import math

INIT_TRAJ_OK = 0
INIT_TRAJ_TOO_SMALL = 201
INIT_TRAJ_TOO_BIG = 202
INIT_TRAJ_ALREADY_IN_MOTION = 203
INIT_TRAJ_INVALID_STARTING_POS = 204
INIT_TRAJ_INVALID_JOINTNAME = 206
INIT_TRAJ_INCOMPLETE_JOINTLIST = 207
INIT_TRAJ_BACKWARD_TIME = 210
INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS = 211
INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES = 212
INIT_TRAJ_INVALID_ENDING_VELOCITY = 213

ROS_INVALID_GOAL = -1


def motoros2_error_code(code, ros_code=ROS_INVALID_GOAL):
    """Combine a ROS FollowJointTrajectory code and a MotoROS2 code: -1, 202 -> -100202."""
    return -(abs(ros_code) * 100000 + code)


def _seconds(duration):
    """Accept a builtin_interfaces/Duration-like object (sec, nanosec) or a float."""
    if isinstance(duration, (int, float)):
        return float(duration)
    return duration.sec + duration.nanosec * 1e-9


def check_trajectory(joint_names, points, expected_joints, current_positions=None,
                     max_points=200, start_tolerance=0.01, end_velocity_tolerance=1e-4):
    """Return (code, message). code == INIT_TRAJ_OK when the goal would be accepted.

    joint_names        joint names of the trajectory
    points             objects with .positions, .velocities, .time_from_start
    expected_joints    joint names configured on the controller
    current_positions  dict joint name -> current position (rad), or None to skip
    """
    if len(points) < 2:
        return INIT_TRAJ_TOO_SMALL, 'Trajectory must contain at least two points.'
    if len(points) > max_points:
        return INIT_TRAJ_TOO_BIG, (
            f'Trajectory contains {len(points)} points; the MFJA setup accepts at most {max_points}.')
    unknown = [name for name in joint_names if name not in expected_joints]
    if unknown:
        return INIT_TRAJ_INVALID_JOINTNAME, f'Invalid joint name(s): {unknown}.'
    if len(set(joint_names)) != len(expected_joints):
        return INIT_TRAJ_INCOMPLETE_JOINTLIST, 'Trajectory must contain data for all joints.'

    n_joints = len(joint_names)
    previous_time = None
    for index, point in enumerate(points):
        if len(point.positions) != n_joints:
            return INIT_TRAJ_WRONG_NUMBER_OF_POSITIONS, (
                f'Point {index} does not contain a position for every joint.')
        if len(point.velocities) != n_joints:
            return INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES, (
                f'Point {index} does not contain a velocity for every joint.')
        time = _seconds(point.time_from_start)
        if previous_time is not None and time <= previous_time:
            return INIT_TRAJ_BACKWARD_TIME, (
                f'Point {index}: time_from_start must strictly increase.')
        previous_time = time

    if any(abs(v) > end_velocity_tolerance for v in points[-1].velocities):
        return INIT_TRAJ_INVALID_ENDING_VELOCITY, 'The final point must have zero velocity.'

    if current_positions is not None:
        for name, position in zip(joint_names, points[0].positions):
            current = current_positions.get(name)
            if current is None or not math.isfinite(current):
                continue
            if abs(position - current) > start_tolerance:
                return INIT_TRAJ_INVALID_STARTING_POS, (
                    f'The first point must match the current position ({name}: '
                    f'{position:.4f} rad requested, {current:.4f} rad measured).')
    return INIT_TRAJ_OK, 'OK'


def io_address_valid(address):
    """Yaskawa IO addresses: the last digit is the bit number (0 to 7)."""
    return address > 0 and address % 10 <= 7


def io_address_writable(address):
    """Writable ranges of MotoROS2 write_single_io (YRC1000 family)."""
    return io_address_valid(address) and (
        10010 <= address <= 12567 or 27010 <= address <= 29567)


def universal_output_address(n):
    """Logical address of universal output OT#n: OT#1 -> 10010, OT#9 -> 10020, OT#100 -> 10133."""
    if n < 1:
        raise ValueError('OT numbers start at 1')
    return 10000 + 10 * ((n - 1) // 8 + 1) + (n - 1) % 8
