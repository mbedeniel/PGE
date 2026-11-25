"""Simulate the Room 315 Yaskawa arms in Gazebo Harmonic with gz_ros2_control.

* start_gazebo:=true (default): empty world (1 ms step) + /clock bridge, arms side by side.
* start_gazebo:=false: spawn the arms into a world that is already running, e.g. the mfja
  twin started with robots:=none. Both launches must use the same gz_partition.

Each arm is spawned by yaskawa_spawn.launch.py in its own namespace
(cell file: config/yaskawa_cell.yaml), with the MotoROS2 interface of the real robot.
As on the real arms, arm_controller starts inactive: call start_traj_mode (MotoROS2 trajectories),
or start_point_queue_mode and then activate arm_controller (ros2_control).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

POSE_SETS = ('standalone', 'room_315')


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    start_gazebo = _as_bool(arg('start_gazebo'))
    world = arg('world_name') or ('empty' if start_gazebo else 'room_315_only')
    poses = arg('poses') or ('standalone' if start_gazebo else 'room_315')
    if poses not in POSE_SETS:
        raise RuntimeError(f'poses must be one of {POSE_SETS}, got "{poses}"')

    cell_file = arg('cell_file')
    with open(cell_file, 'r', encoding='utf-8') as stream:
        cell = yaml.safe_load(stream)['robots']
    selected = [name.strip() for name in arg('robots').split(',') if name.strip()]
    unknown = [name for name in selected if name not in cell]
    if unknown or not selected:
        raise RuntimeError(f'robots must list entries of {cell_file} ({", ".join(cell)})')

    actions = []
    if arg('gz_partition'):
        actions.append(SetEnvironmentVariable('GZ_PARTITION', arg('gz_partition')))

    if start_gazebo:
        gz_args = '-r empty.sdf' if _as_bool(arg('gui')) else '-r -s empty.sdf'
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='yaskawa_clock_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
            ),
        ]

    spawn_launch = os.path.join(
        get_package_share_directory('mfja_yaskawa_gazebo'), 'launch', 'yaskawa_spawn.launch.py')
    for side in selected:
        robot = cell[side]
        pose = robot[f'{poses}_pose']
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                'robot_model': robot['model'],
                'robot_namespace': robot['namespace'],
                'world': world,
                'x': str(pose['x']),
                'y': str(pose['y']),
                'z': str(pose['z']),
                'yaw': str(pose['yaw']),
                'motoros2_sim': arg('motoros2_sim'),
                'activate_arm_controller': arg('activate_arm_controller'),
                'publish_robot_tf': arg('publish_robot_tf'),
            }.items(),
        ))
    return actions


def generate_launch_description():
    default_cell_file = os.path.join(
        get_package_share_directory('mfja_yaskawa_gazebo'), 'config', 'yaskawa_cell.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='left,right',
                              description='Entries of the cell file to spawn (left, right).'),
        DeclareLaunchArgument('start_gazebo', default_value='true',
                              description='true: empty world. false: spawn into a running world.'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (only with start_gazebo:=true).'),
        DeclareLaunchArgument('world_name', default_value='',
                              description='Default: empty, or room_315_only with start_gazebo:=false. '
                                          'Full floor: mfja_3rd_floor.'),
        DeclareLaunchArgument('gz_partition', default_value='',
                              description='Gazebo partition; must match the running world.'),
        DeclareLaunchArgument('poses', default_value='',
                              description='standalone or room_315 (default depends on start_gazebo).'),
        DeclareLaunchArgument('motoros2_sim', default_value='true',
                              description='Start the MotoROS2 interface imitation for each arm.'),
        DeclareLaunchArgument('activate_arm_controller', default_value='false',
                              description='true: start arm_controller active. Default false, as on '
                                          'the real arms.'),
        DeclareLaunchArgument('publish_robot_tf', default_value='true',
                              description='false: simulated TF on <ns>/sim_tf (leave <ns>/tf to MoveIt).'),
        DeclareLaunchArgument('cell_file', default_value=default_cell_file,
                              description='Arms to spawn: namespace, model, poses.'),
        OpaqueFunction(function=_launch_setup),
    ])
