#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     manager = '/sentinel/sim/controller_manager'
     config = PathJoinSubstitution([
          FindPackageShare('sentinel_control'), 'config', 'locomotion.yaml'
     ])
     return LaunchDescription([
          DeclareLaunchArgument('use_sim_time', default_value='true'),
          Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', manager,
                         '--controller-manager-timeout', '120'], output='screen'),
          Node(package='controller_manager', executable='spawner',
               arguments=['joint_group_position_controller', '--controller-manager', manager,
                         '--controller-manager-timeout', '120'], output='screen'),
          Node(package='sentinel_control', executable='sentinel_locomotion_node',
               name='sentinel_locomotion', namespace='/sentinel/sim',
               parameters=[config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
               output='screen'),
     ])
