#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sentinel_gz')
    with open(os.path.join(pkg, 'config', 'simulation.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    robot, p = cfg['robot'], cfg['robot']['pose']

    args = ['-world', cfg['simulation']['world_name'], '-name', robot['entity_name'],
            '-topic', '/sentinel/sim/robot_description', '-allow_renaming', 'false']
    for flag, key in [('-x', 'x'), ('-y', 'y'), ('-z', 'z'),
                      ('-R', 'roll'), ('-P', 'pitch'), ('-Y', 'yaw')]:
        args.extend([flag, str(p[key])])

    return LaunchDescription([
        Node(package='ros_gz_sim', executable='create', name='spawn_sentinel_go2',
             output='screen', arguments=args),
    ])
