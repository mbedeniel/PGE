"""Show one Yaskawa arm in RViz with a slider per joint (no ros2_control, no simulator).

    ros2 launch mfja_yaskawa_description show.launch.py robot_model:=hc10dt
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODELS = ('hc10', 'hc10dt')


def _launch_setup(context):
    share = get_package_share_directory('mfja_yaskawa_description')
    model = LaunchConfiguration('robot_model').perform(context)
    if model not in MODELS:
        raise RuntimeError(f'robot_model must be one of {MODELS}, got "{model}"')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    robot_description = xacro.process_file(
        os.path.join(share, 'robots', f'{model}_robot.urdf.xacro')).toxml()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui' if gui else 'joint_state_publisher',
            executable='joint_state_publisher_gui' if gui else 'joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'show.rviz')],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model', default_value='hc10',
                              description='hc10 (right robot) or hc10dt (left robot).'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the joint sliders.'),
        OpaqueFunction(function=_launch_setup),
    ])
