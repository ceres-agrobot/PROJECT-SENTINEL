#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('sentinel_description')
    flags = ['enable_control', 'enable_imu', 'enable_camera', 'enable_d455',
             'enable_lidar_l1', 'enable_velodyne', 'enable_gps']
    params = ['xacro ', LaunchConfiguration('model'),
              ' physical_config_file:=', LaunchConfiguration('physical_config_file')]
    for flag in flags:
        params.extend([' ' + flag + ':=', LaunchConfiguration(flag)])

    robot_description = ParameterValue(Command(params), value_type=str)
    args = [
        DeclareLaunchArgument('model', default_value=PathJoinSubstitution([
            pkg, 'urdf', 'unitree_go2_robot.xacro'])),
        DeclareLaunchArgument('physical_config_file', default_value=PathJoinSubstitution([
            pkg, 'config', 'sentinel_params.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('robot_namespace', default_value='/sentinel/sim'),
    ]
    for flag in flags:
        args.append(DeclareLaunchArgument(flag,
                    default_value='false' if flag in ('enable_control', 'enable_gps', 'enable_velodyne') else 'true',
                    choices=['true', 'false']))

    return LaunchDescription(args + [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace=LaunchConfiguration('robot_namespace'),
             output='screen',
             remappings=[('/joint_states', '/sentinel/sim/joint_states')],
             parameters=[{'robot_description': robot_description,
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])