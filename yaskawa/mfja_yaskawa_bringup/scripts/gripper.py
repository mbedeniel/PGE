#!/usr/bin/env python3
"""Open, close or initialise the gripper of a Yaskawa arm through gripper_controller.

    ros2 run mfja_yaskawa_bringup gripper.py --robot right close
    ros2 run mfja_yaskawa_bringup gripper.py --robot right open

The gripper reacts when a request goes from 0 to 1, and the robot controller never resets the
requests. Each command is therefore sent as a pulse:
  1. if the request already reads 1 (for example after a teach-pendant key), release it first;
  2. set the request to 1 and the others to 0, and wait until the request reads 1;
  3. keep it for --hold seconds, then set every request to 0 and wait until they read 0.
The states come from joint_state_broadcaster (dynamic_joint_states). Works on the real arms
(yaskawa_real.launch.py) and on mock hardware (yaskawa_mock.launch.py).
On error or Ctrl+C, every request is set back to 0.
"""

import argparse
import math
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NAMESPACES = {'right': 'yaskawa_RIGHT', 'left': 'yaskawa_LEFT'}
GPIO = 'gripper'
REQUESTS = ['init', 'open', 'close']  # same order as interface_names of gripper_controller


def command_vector(request=None):
    """[init, open, close] with 1.0 for `request` and 0.0 elsewhere."""
    return [1.0 if name == request else 0.0 for name in REQUESTS]


def gripper_states(msg):
    """Return {interface: value} of the gripper in a DynamicJointState message, or None."""
    if GPIO not in msg.joint_names:
        return None
    values = msg.interface_values[msg.joint_names.index(GPIO)]
    return dict(zip(values.interface_names, values.values))


class Gripper(Node):

    def __init__(self, namespace):
        super().__init__('gripper')
        self.namespace = namespace
        self.states = None
        self.publisher = self.create_publisher(
            Float64MultiArray, f'/{namespace}/gripper_controller/commands', 10)
        # Mock launch: <ns>/dynamic_joint_states. Real launch: the broadcaster uses local topics.
        for topic in (f'/{namespace}/dynamic_joint_states',
                      f'/{namespace}/joint_state_broadcaster/dynamic_joint_states'):
            self.create_subscription(DynamicJointState, topic, self._on_states, 10)

    def _on_states(self, msg):
        states = gripper_states(msg)
        if states is not None:
            self.states = states

    def send(self, values):
        self.publisher.publish(Float64MultiArray(data=list(values)))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, condition, timeout, values=None):
        """Spin until condition(states) is true; re-send `values` every 0.2 s if given."""
        end = time.monotonic() + timeout
        last_send = 0.0
        while time.monotonic() < end:
            if values is not None and time.monotonic() - last_send > 0.2:
                self.send(values)
                last_send = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.states is not None and condition(self.states):
                return True
        return False


def is_set(states, name):
    value = states.get(name, math.nan)
    return not math.isnan(value) and value >= 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--robot', choices=sorted(NAMESPACES), required=True)
    parser.add_argument('request', choices=REQUESTS)
    parser.add_argument('--hold', type=float, default=0.5,
                        help='seconds the request stays at 1 once it reads 1 (default 0.5)')
    parser.add_argument('--timeout', type=float, default=3.0,
                        help='longest wait for each step, in seconds (default 3)')
    args = parser.parse_args()

    rclpy.init()
    node = Gripper(NAMESPACES[args.robot])
    released = command_vector()
    ok = False
    try:
        end = time.monotonic() + args.timeout
        while node.publisher.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.publisher.get_subscription_count() == 0:
            print(f'No subscriber on /{node.namespace}/gripper_controller/commands '
                  '(is gripper_controller active?)')
            return 1
        if not node.wait_until(lambda s: all(n in s for n in REQUESTS), args.timeout):
            print(f'No gripper state received within {args.timeout} s '
                  '(is joint_state_broadcaster active?)')
            return 1
        print(f'/{node.namespace} gripper before: {node.states}')

        if is_set(node.states, args.request):
            print(f'"{args.request}" already reads 1: releasing it first')
            if not node.wait_until(lambda s: not is_set(s, args.request), args.timeout, released):
                print(f'"{args.request}" still reads 1 after {args.timeout} s')
                return 1

        if not node.wait_until(lambda s: is_set(s, args.request), args.timeout,
                               command_vector(args.request)):
            print(f'"{args.request}" did not read 1 within {args.timeout} s')
            return 1
        node.spin_for(args.hold)

        if not node.wait_until(lambda s: not any(is_set(s, n) for n in REQUESTS),
                               args.timeout, released):
            print(f'Requests did not read 0 within {args.timeout} s')
            return 1
        print(f'"{args.request}" sent (pulse). Gripper after: {node.states}')
        ok = True
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        if not ok and rclpy.ok():
            node.send(released)
            node.spin_for(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
