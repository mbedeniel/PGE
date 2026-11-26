"""MoveIt and RViz on our arm_controller, for one Yaskawa arm.

Plan in RViz, then execute through /<ns>/arm_controller/follow_joint_trajectory: the
joint_trajectory_controller started by yaskawa_real.launch.py (real arm) or yaskawa_mock.launch.py.

The planning model (URDF, SRDF, kinematics, joint limits, RViz configuration) is the one of
motoman_hc10_moveit_config (repository mfja_ros2_yaskawa), reused without change; only the execution
differs: config/moveit_controllers.yaml of this package instead of MotoROS2's own action.

Requirements: MoveIt and motoman_hc10_moveit_config. On the PC that drives the robots they are in the
PC's own workspace, so run this file by its path from a normal terminal, while the controllers run in
the pge_shell.sh terminal:

    ros2 launch ~/pge_ws/src/PGE/yaskawa/mfja_yaskawa_bringup/launch/yaskawa_moveit.launch.py robot:=right

Before executing: start_point_queue_mode, then activate arm_controller (Yaskawa README).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# This file is used by its path as well as installed: find the controllers file next to it.
CONTROLLERS_FILE = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'moveit_controllers.yaml')


def _launch_setup(context):
    side = LaunchConfiguration('robot').perform(context).upper()
    ns = f'yaskawa_{side}'
    moveit_share = get_package_share_directory('motoman_hc10_moveit_config')
    config = os.path.join(moveit_share, 'config')

    moveit_config = (
        MoveItConfigsBuilder(ns, package_name='motoman_hc10_moveit_config')
        .robot_description(file_path=os.path.join(config, f'{ns}.urdf.xacro'))
        .robot_description_semantic(file_path=os.path.join(config, f'{ns}.srdf'))
        .robot_description_kinematics(file_path=os.path.join(config, 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(config, 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.normpath(CONTROLLERS_FILE))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params['move_group_namespace'] = f'/{ns}'

    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return [
        LogInfo(msg=f'MoveIt on /{ns}/arm_controller/follow_joint_trajectory'),
        # TF of the planning model. Its robot_description topic is renamed: the controller manager of
        # the arm already reads <ns>/robot_description.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            output='screen',
            parameters=[moveit_config.robot_description],
            remappings=tf_remappings + [('robot_description', 'moveit/robot_description')],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            namespace=ns,
            output='screen',
            parameters=[moveit_params, {'use_sim_time': False}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            namespace=ns,
            output='screen',
            arguments=['-d', os.path.join(config, f'moveit_{side}.rviz')],
            parameters=[
                moveit_params,
                {'use_sim_time': False},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            remappings=tf_remappings,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='right', choices=['left', 'right'],
                              description='Arm: right (HC10, yaskawa_RIGHT) or left (HC10DT, yaskawa_LEFT).'),
        OpaqueFunction(function=_launch_setup),
    ])
