"""One real Yaskawa arm on ros2_control, through MotoROS2 (motoros2_hw_interfaces).

    ros2 launch mfja_yaskawa_bringup yaskawa_real.launch.py robot_model:=hc10

Starts, in the robot namespace (yaskawa_RIGHT for hc10, yaskawa_LEFT for hc10dt):
robot_state_publisher, ros2_control_node with motoros2_hw_interfaces/MotoROS2HardwareInterface,
joint_state_broadcaster (local topics), arm_controller (joint_trajectory_controller) and
gripper_controller (gripper requests, sent only when scripts/gripper.py publishes one).

arm_controller is loaded INACTIVE by default: nothing is sent to the robot until it is
activated, after start_point_queue_mode (procedure in mfja_yaskawa/README.md).
The micro-ROS agent must be running and MotoROS2 must publish <ns>/joint_states.
robot_model has no default, so that the arm is always named explicitly.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_NAMESPACES = {'hc10': 'yaskawa_RIGHT', 'hc10dt': 'yaskawa_LEFT'}


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    model = LaunchConfiguration('robot_model').perform(context).strip()
    if model not in DEFAULT_NAMESPACES:
        raise RuntimeError(
            f'robot_model must be hc10 (right arm) or hc10dt (left arm), got "{model}"')
    namespace = LaunchConfiguration('robot_namespace').perform(context).strip().strip('/') \
        or DEFAULT_NAMESPACES[model]
    activate = _as_bool(LaunchConfiguration('activate_arm_controller').perform(context))

    description_share = get_package_share_directory('mfja_yaskawa_description')
    controllers_file = os.path.join(
        get_package_share_directory('mfja_yaskawa_bringup'), 'config',
        'yaskawa_controllers_real.yaml')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'robots', f'{model}_robot.urdf.xacro'),
        mappings={'use_real_hardware': 'true', 'robot_namespace': namespace},
    ).toxml()

    arm_spawner_args = ['arm_controller', '--controller-manager', f'/{namespace}/controller_manager']
    if not activate:
        arm_spawner_args.append('--inactive')

    return [
        LogInfo(msg=f'Real Yaskawa {model} in namespace /{namespace}; arm_controller '
                    + ('ACTIVE' if activate else 'inactive')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
            # MotoROS2 publishes the arm's TF itself; by default ours goes to <ns>/description_tf.
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            if _as_bool(LaunchConfiguration('publish_robot_tf').perform(context))
            else [('/tf', 'description_tf'), ('/tf_static', 'description_tf_static')],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            output='screen',
            parameters=[controllers_file],
            remappings=[('~/robot_description', 'robot_description')],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            output='screen',
            arguments=['joint_state_broadcaster', 'gripper_controller',
                       '--controller-manager', f'/{namespace}/controller_manager'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            output='screen',
            arguments=arm_spawner_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model',
                              description='hc10 (right arm, blue) or hc10dt (left arm, white).'),
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='MotoROS2 namespace. Default: yaskawa_RIGHT (hc10) '
                                          'or yaskawa_LEFT (hc10dt).'),
        DeclareLaunchArgument('activate_arm_controller', default_value='false',
                              description='true: start arm_controller active. Default false: '
                                          'activate it by hand after start_point_queue_mode.'),
        DeclareLaunchArgument('publish_robot_tf', default_value='false',
                              description='true: robot_state_publisher publishes <ns>/tf '
                                          '(only if MotoROS2 does not).'),
        OpaqueFunction(function=_launch_setup),
    ])
