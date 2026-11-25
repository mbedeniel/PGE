#!/usr/bin/env python3
"""Move one joint of a Yaskawa arm by a small angle through arm_controller, then stop.

    ros2 run mfja_yaskawa_bringup move_joint.py --robot right --joint 6 --delta 0.1
    ros2 run mfja_yaskawa_bringup move_joint.py --robot right --joint 6 --delta 0.2 --back --points 300

Reads the current position on <ns>/joint_state_broadcaster/joint_states (what ros2_control sees,
same topic in Gazebo, with mock hardware and on the real arms), sends a smooth (cosine) trajectory to
<ns>/arm_controller/follow_joint_trajectory and waits for the result. Works the same on the
real arms (yaskawa_real.launch.py) and in simulation (mock or Gazebo).
Ctrl+C cancels the goal: the controller then holds the current position.

Limits, refused rather than clipped: |delta| <= 0.5 rad, duration >= 2 s, peak speed <= 0.3 rad/s.
"""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

NAMESPACES = {'right': 'yaskawa_RIGHT', 'left': 'yaskawa_LEFT'}
JOINTS = [f'joint_{i}' for i in range(1, 7)]
MAX_DELTA = 0.5        # rad
MIN_DURATION = 2.0     # s
MAX_PEAK_SPEED = 0.3   # rad/s


def _duration(seconds):
    sec = int(math.floor(seconds))
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


def cosine_segment(start, joint_index, delta, duration, points, t0=0.0):
    """Points from start to start + delta on one joint, zero velocity at both ends."""
    result = []
    for i in range(1, points + 1):
        t = duration * i / points
        s = 0.5 * (1.0 - math.cos(math.pi * t / duration))
        v = 0.5 * math.pi / duration * math.sin(math.pi * t / duration)
        positions = list(start)
        velocities = [0.0] * len(start)
        positions[joint_index] = start[joint_index] + delta * s
        velocities[joint_index] = 0.0 if i == points else delta * v
        result.append(JointTrajectoryPoint(
            positions=positions, velocities=velocities, time_from_start=_duration(t0 + t)))
    return result


class MoveJoint(Node):

    def __init__(self, namespace):
        super().__init__('move_joint')
        self.namespace = namespace
        self.state = None
        self.create_subscription(
            JointState, f'/{namespace}/joint_state_broadcaster/joint_states', self._on_state,
            qos_profile_sensor_data)
        self.client = ActionClient(
            self, FollowJointTrajectory, f'/{namespace}/arm_controller/follow_joint_trajectory')

    def _on_state(self, msg):
        if all(name in msg.name for name in JOINTS):
            self.state = [msg.position[msg.name.index(name)] for name in JOINTS]

    def wait_for_state(self, timeout):
        end = time.monotonic() + timeout
        while self.state is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--robot', choices=sorted(NAMESPACES), required=True)
    parser.add_argument('--joint', type=int, choices=range(1, 7), required=True,
                        help='joint number, 1 to 6')
    parser.add_argument('--delta', type=float, required=True, help='rad, |delta| <= 0.5')
    parser.add_argument('--duration', type=float, default=5.0, help='s per move (default 5)')
    parser.add_argument('--points', type=int, default=50,
                        help='points per move (default 50); more than 200 checks that the '
                             'MotoROS2 limit no longer applies')
    parser.add_argument('--back', action='store_true', help='return to the start position')
    args = parser.parse_args()

    peak_speed = 0.5 * math.pi * abs(args.delta) / max(args.duration, 1e-6)
    if abs(args.delta) > MAX_DELTA:
        sys.exit(f'Refused: |delta| = {abs(args.delta):.3f} rad > {MAX_DELTA} rad')
    if args.duration < MIN_DURATION:
        sys.exit(f'Refused: duration {args.duration} s < {MIN_DURATION} s')
    if peak_speed > MAX_PEAK_SPEED:
        sys.exit(f'Refused: peak speed {peak_speed:.3f} rad/s > {MAX_PEAK_SPEED} rad/s')
    if args.points < 2:
        sys.exit('Refused: at least 2 points')

    rclpy.init()
    node = MoveJoint(NAMESPACES[args.robot])
    goal_handle = None
    try:
        start = node.wait_for_state(5.0)
        if start is None:
            sys.exit(f'No joint_1..joint_6 state on /{node.namespace}/joint_state_broadcaster/'
                     'joint_states within 5 s (is joint_state_broadcaster active?)')
        if not node.client.wait_for_server(timeout_sec=5.0):
            sys.exit(f'No action server /{node.namespace}/arm_controller/follow_joint_trajectory '
                     '(is the launch running?)')

        j = args.joint - 1
        points = cosine_segment(start, j, args.delta, args.duration, args.points)
        if args.back:
            target = list(start)
            target[j] += args.delta
            points += cosine_segment(target, j, -args.delta, args.duration, args.points,
                                     t0=args.duration)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = points
        goal.goal_time_tolerance = _duration(1.0)

        print(f'/{node.namespace}: joint_{args.joint} {start[j]:+.4f} -> '
              f'{start[j] + args.delta:+.4f} rad in {args.duration:.1f} s'
              + (', then back' if args.back else '')
              + f' ({len(points)} points, peak {peak_speed:.3f} rad/s)')

        future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            sys.exit('Goal REJECTED (is arm_controller active? does the trajectory start '
                     'at the current position?)')
        print('Goal accepted; Ctrl+C cancels it.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
                  GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(result.status, str(result.status))
        print(f'Result: {status}, error_code {result.result.error_code} '
              f'{result.result.error_string}'.rstrip())
        goal_handle = None
        node.state = None
        end = node.wait_for_state(2.0)
        if end is not None:
            print(f'joint_{args.joint} now {end[j]:+.4f} rad')
        return 0 if result.status == GoalStatus.STATUS_SUCCEEDED else 1
    except KeyboardInterrupt:
        if goal_handle is not None:
            print('\nCancelling the goal...')
            cancel = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
            print('Cancel request sent.')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
