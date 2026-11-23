#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('bridge_config_file')
    adapter = Node(
        package='sentinel_bridge', executable='sentinel_state_adapter_node',
        namespace='/sentinel/sim', name='sentinel_state_adapter',
        parameters=[{'use_sim_time': True}], output='screen'
    )
    return LaunchDescription([
        DeclareLaunchArgument('bridge_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('sentinel_bridge'), 'config', 'bridge_config.yaml'])),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='sentinel_gz_bridge', namespace='/sentinel/sim', output='screen',
             parameters=[{'config_file': config, 'use_sim_time': True}]),
        adapter,
    ])
