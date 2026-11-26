"""One Yaskawa arm with mock hardware (no simulator): first test of the controllers.

    ros2 launch mfja_yaskawa_bringup yaskawa_mock.launch.py robot_model:=hc10

Starts, in the robot namespace: robot_state_publisher, ros2_control_node with
mock_components/GenericSystem, joint_state_broadcaster, gripper_controller, arm_controller and the
MotoROS2 interface imitation (same names as the real robot).
Same start-up as the real arms: arm_controller is loaded inactive. Either call start_traj_mode
(MotoROS2 trajectories; the imitation then activates arm_controller), or call
start_point_queue_mode and activate arm_controller by hand. activate_arm_controller:=true skips this.
The controllers file is the one of the real arms and of Gazebo (config/yaskawa_controllers.yaml):
the mock rehearses the real configuration.
Runs on wall-clock time, so MoveIt launches that use wall-clock time work against it.
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
    model = LaunchConfiguration('robot_model').perform(context)
    if model not in DEFAULT_NAMESPACES:
        raise RuntimeError(f'robot_model must be one of {tuple(DEFAULT_NAMESPACES)}, got "{model}"')
    namespace = LaunchConfiguration('robot_namespace').perform(context).strip() \
        or DEFAULT_NAMESPACES[model]

    description_share = get_package_share_directory('mfja_yaskawa_description')
    config = os.path.join(get_package_share_directory('mfja_yaskawa_bringup'), 'config')
    # Same controllers file as the real arms and Gazebo.
    controllers_files = [os.path.join(config, 'yaskawa_controllers.yaml')]
    activate = _as_bool(LaunchConfiguration('activate_arm_controller').perform(context))
    arm_spawner_args = ['arm_controller', '--controller-manager', f'/{namespace}/controller_manager']
    if not activate:
        arm_spawner_args.append('--inactive')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'robots', f'{model}_robot.urdf.xacro'),
        mappings={'use_mock_hardware': 'true'},
    ).toxml()

    actions = [
        LogInfo(msg=f'Mock Yaskawa {model} in namespace /{namespace}; arm_controller '
                    + ('ACTIVE' if activate else 'inactive (start_traj_mode, or '
                       'start_point_queue_mode then activation, as on the real arms)')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'robot_description': robot_description}],
            # publish_robot_tf:=false leaves <ns>/tf to another publisher (e.g. a MoveIt launch).
            remappings=[('joint_states', 'joint_state_broadcaster/joint_states')] + ([('/tf', 'tf'), ('/tf_static', 'tf_static')]
            if _as_bool(LaunchConfiguration('publish_robot_tf').perform(context))
            else [('/tf', 'sim_tf'), ('/tf_static', 'sim_tf_static')]),
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            output='screen',
            # Wall clock, as on the real arms: there is no /clock here.
            parameters=controllers_files + [{'use_sim_time': False}],
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
        DeclareLaunchArgument('activate_arm_controller', default_value='false',
                              description='true: start arm_controller active. Default false, as on '
                                          'the real arms (see start_traj_mode, start_point_queue_mode).'),
        DeclareLaunchArgument('publish_robot_tf', default_value='true',
                              description='false: TF on <ns>/sim_tf (leave <ns>/tf to MoveIt).'),
        DeclareLaunchArgument('robot_namespace', default_value='',
                              description='Default: yaskawa_RIGHT (hc10) or yaskawa_LEFT (hc10dt).'),
        OpaqueFunction(function=_launch_setup),
    ])
