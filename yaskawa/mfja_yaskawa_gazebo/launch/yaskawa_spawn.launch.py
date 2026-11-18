"""Spawn one Yaskawa arm into a running Gazebo world and start its controllers.

Can be included several times with different namespaces (several arms in one world).

    1. xacro of robots/<robot_model>_robot.urdf.xacro with gz_sim:=true
    2. robot_state_publisher (TF on <robot_namespace>/tf)
    3. ros_gz_sim create, with the description passed as a string
    4. joint_state_broadcaster and arm_controller, once the arm is in the world
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODELS = ('hc10', 'hc10dt')


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    model = arg('robot_model')
    namespace = arg('robot_namespace')
    if model not in MODELS:
        raise RuntimeError(f'robot_model must be one of {MODELS}, got "{model}"')
    if not namespace:
        raise RuntimeError('robot_namespace must not be empty')

    controllers_file = arg('controller_params_file') or os.path.join(
        get_package_share_directory('mfja_yaskawa_bringup'), 'config', 'yaskawa_controllers.yaml')
    robot_description = xacro.process_file(
        os.path.join(get_package_share_directory('mfja_yaskawa_description'),
                     'robots', f'{model}_robot.urdf.xacro'),
        mappings={
            'gz_sim': 'true',
            'robot_namespace': namespace,
            'controller_params_file': controllers_file,
        },
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{namespace}',
        output='screen',
        arguments=[
            '-world', arg('world'),
            '-name', namespace,
            '-string', robot_description,
            '-x', arg('x'), '-y', arg('y'), '-z', arg('z'), '-Y', arg('yaw'),
        ],
    )
    spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'arm_controller',
            '--controller-manager', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '120',
        ],
    )
    return [
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawner])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', description='hc10 or hc10dt.'),
        DeclareLaunchArgument('robot_namespace', description='e.g. yaskawa_LEFT.'),
        DeclareLaunchArgument('world', default_value='empty', description='Gazebo world name.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('controller_params_file', default_value='',
                              description='Default: mfja_yaskawa_bringup/config/yaskawa_controllers.yaml'),
        OpaqueFunction(function=_launch_setup),
    ])
