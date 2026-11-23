#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg = get_package_share_directory('sentinel_gz')
    desc = get_package_share_directory('sentinel_description')
    bridge = get_package_share_directory('sentinel_bridge')
    with open(os.path.join(pkg, 'config', 'simulation.yaml'), encoding='utf-8') as f:
        config = yaml.safe_load(f)

    def include(package, launch_file, arguments=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(package, 'launch', launch_file)),
            launch_arguments=(arguments or {}).items())

    world = include(pkg, 'spawn_world.launch.py')
    description = include(desc, 'publish_sentinel_urdf.launch.py', {
        'use_sim_time': 'true',
        'enable_control': LaunchConfiguration('enable_control'),
        'physical_config_file': LaunchConfiguration('physical_config_file'),
        'enable_imu': LaunchConfiguration('enable_imu'),
        'enable_camera': LaunchConfiguration('enable_camera'),
        'enable_d455': LaunchConfiguration('enable_d455'),
        'enable_lidar_l1': LaunchConfiguration('enable_lidar_l1'),
        'enable_velodyne': LaunchConfiguration('enable_velodyne'),
        'enable_gps': LaunchConfiguration('enable_gps'),
    })
    bridge_launch = include(bridge, 'gz_bridge.launch.py')
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sentinel_control'), 'launch', 'control.launch.py')),
        condition=IfCondition(LaunchConfiguration('enable_control')))
    robot = include(pkg, 'spawn_robot.launch.py')
    models = include(pkg, 'spawn_models.launch.py')
    wait = ExecuteProcess(cmd=['python3', os.path.join(
        pkg, 'scripts', 'wait_for_gazebo_ready.py'),
        '--world', config['simulation']['world_name'], '--timeout', '120'],
        output='screen')

    def on_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Gazebo create service unavailable'))]
        return [models if config['scenery']['enabled'] else robot]

    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('physical_config_file', default_value=os.path.join(
            desc, 'config', 'sentinel_params.yaml')),
        *[DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'])
          for name, default in [('enable_imu', 'true'), ('enable_camera', 'true'),
                                ('enable_d455', 'true'), ('enable_lidar_l1', 'true'),
                                ('enable_velodyne', 'false'), ('enable_gps', 'false')]],
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=on_ready)),
        world, description, bridge_launch, control_launch, wait,
    ])
