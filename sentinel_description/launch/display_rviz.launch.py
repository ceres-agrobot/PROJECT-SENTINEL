#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
     
     pkg = FindPackageShare('sentinel_description')
     use_sim_time = LaunchConfiguration('use_sim_time')
     gui = LaunchConfiguration('use_joint_state_gui')
     publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(
          PathJoinSubstitution([pkg, 'launch', 'publish_sentinel_urdf.launch.py'])),
          launch_arguments={'use_sim_time': use_sim_time}.items())

     return LaunchDescription([
          DeclareLaunchArgument('use_sim_time', default_value='false'),
          DeclareLaunchArgument('use_joint_state_gui', default_value='true'),
          publisher,
          Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
               condition=IfCondition(gui), parameters=[{'use_sim_time': use_sim_time}]),
          Node(package='joint_state_publisher', executable='joint_state_publisher',
               condition=UnlessCondition(gui), parameters=[{'use_sim_time': use_sim_time}]),
          Node(package='rviz2', executable='rviz2',
               arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'sentinel.rviz'])],
               parameters=[{'use_sim_time': use_sim_time}], output='screen'),
     ])

