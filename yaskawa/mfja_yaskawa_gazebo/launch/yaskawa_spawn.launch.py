"""Spawn one Yaskawa arm into a running Gazebo world and start its controllers.

Can be included several times with different namespaces (several arms in one world).

    1. xacro of robots/<robot_model>_robot.urdf.xacro with gz_sim:=true
    2. robot_state_publisher (TF on <robot_namespace>/tf)
    3. ros_gz_sim create, with the description passed as a string
    4. joint_state_broadcaster, gripper_controller and arm_controller, once the arm is in the world;
       arm_controller inactive, as on the real arms (activate_arm_controller:=true starts it active)
    5. motoros2_sim: the MotoROS2 interface of the real robot (follow_joint_trajectory,
       robot_status, start_traj_mode, start_point_queue_mode, ...), relayed to arm_controller

The hardware system is mfja_yaskawa_gz_hardware/YaskawaGazeboSystem (same interfaces as the real
arms). Until arm_controller is active, it holds the joints where they are, as the brakes do on the
real arms.
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


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


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
        # publish_robot_tf:=false leaves <ns>/tf to another publisher (e.g. a MoveIt launch).
        remappings=[('joint_states', 'joint_state_broadcaster/joint_states')] + ([('/tf', 'tf'), ('/tf_static', 'tf_static')]
        if _as_bool(arg('publish_robot_tf')) else [('/tf', 'sim_tf'), ('/tf_static', 'sim_tf_static')]),
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
    controller_manager = ['--controller-manager', f'/{namespace}/controller_manager',
                          '--controller-manager-timeout', '120']
    arm_spawner_args = ['arm_controller'] + controller_manager
    if not _as_bool(arg('activate_arm_controller')):
        arm_spawner_args.append('--inactive')
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            output='screen',
            arguments=['joint_state_broadcaster', 'gripper_controller'] + controller_manager,
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            output='screen',
            arguments=arm_spawner_args,
        ),
    ]
    actions = [
        robot_state_publisher,
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawners)),
    ]
    if _as_bool(arg('motoros2_sim')):
        actions.append(Node(
            package='mfja_yaskawa_motoros2_sim',
            executable='motoros2_sim',
            namespace=namespace,
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('mfja_yaskawa_motoros2_sim'),
                             'config', 'motoros2_sim.yaml'),
                {'use_sim_time': True},
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', description='hc10 or hc10dt.'),
        DeclareLaunchArgument('robot_namespace', description='e.g. yaskawa_LEFT.'),
        DeclareLaunchArgument('world', default_value='empty', description='Gazebo world name.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('motoros2_sim', default_value='true',
                              description='Start the MotoROS2 interface imitation.'),
        DeclareLaunchArgument('activate_arm_controller', default_value='false',
                              description='true: start arm_controller active. Default false, as on '
                                          'the real arms (see start_traj_mode, start_point_queue_mode).'),
        DeclareLaunchArgument('publish_robot_tf', default_value='true',
                              description='false: publish the simulated TF on <ns>/sim_tf instead of <ns>/tf.'),
        DeclareLaunchArgument('controller_params_file', default_value='',
                              description='Default: mfja_yaskawa_bringup/config/yaskawa_controllers.yaml'),
        OpaqueFunction(function=_launch_setup),
    ])
