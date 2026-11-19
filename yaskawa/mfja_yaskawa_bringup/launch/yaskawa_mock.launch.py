"""One Yaskawa arm with mock hardware (no simulator): first test of the controllers.

    ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10

Starts, in the robot namespace: robot_state_publisher, ros2_control_node with
mock_components/GenericSystem, joint_state_broadcaster, arm_controller and the
MotoROS2 interface imitation (same names as the real robot).
The controllers are the same as in Gazebo (config/yaskawa_controllers.yaml).
Runs on wall-clock time, so MoveIt launches that use wall-clock time work against it.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_NAMESPACES = {'hc10': 'yaskawa_RIGHT', 'hc10dt': 'yaskawa_LEFT'}


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    model = LaunchConfiguration('robot_model').perform(context)
    if model not in DEFAULT_NAMESPACES:
        raise RuntimeError(f'robot_model must be one of {tuple(DEFAULT_NAMESPACES)}, got "{model}"')
    namespace = LaunchConfiguration('robot_namespace').perform(context).strip() \
        or DEFAULT_NAMESPACES[model]

    description_share = get_package_share_directory('mfja_yaskawa_description')
    controllers_file = os.path.join(
        get_package_share_directory('mfja_yaskawa_bringup'), 'config', 'yaskawa_controllers.yaml')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'robots', f'{model}_robot.urdf.xacro'),
        mappings={'use_mock_hardware': 'true'},
    ).toxml()

    actions = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
            # publish_robot_tf:=false leaves <ns>/tf to another publisher (e.g. a MoveIt launch).
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
            if _as_bool(LaunchConfiguration('publish_robot_tf').perform(context))
            else [('/tf', 'sim_tf'), ('/tf_static', 'sim_tf_static')],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            output='screen',
            # The controllers file sets use_sim_time for Gazebo; there is no /clock here.
            parameters=[controllers_file, {'use_sim_time': False}],
            remappings=[('~/robot_description', 'robot_description')],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            output='screen',
            arguments=[
                'joint_state_broadcaster',
                'arm_controller',
                '--controller-manager', f'/{namespace}/controller_manager',
            ],
        ),
    ]
    if _as_bool(LaunchConfiguration('motoros2_sim').perform(context)):
        actions.append(Node(
            package='mfja_yaskawa_motoros2_sim',
            executable='motoros2_sim',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('mfja_yaskawa_motoros2_sim'),
                             'config', 'motoros2_sim.yaml'),
                {'use_sim_time': False},
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='hc10',
                              description='hc10 (right robot) or hc10dt (left robot).'),
        DeclareLaunchArgument('motoros2_sim', default_value='true',
                              description='Start the MotoROS2 interface imitation.'),
        DeclareLaunchArgument('publish_robot_tf', default_value='true',
                              description='false: TF on <ns>/sim_tf (leave <ns>/tf to MoveIt).'),
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='Default: yaskawa_RIGHT (hc10) or yaskawa_LEFT (hc10dt).'),
        OpaqueFunction(function=_launch_setup),
    ])
