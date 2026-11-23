#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('sentinel_gz')
    with open(os.path.join(pkg, 'config', 'simulation.yaml'), encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    world = cfg['simulation']['world_name']

    def model_node(key):
        model = cfg['scenery'][key]
        p = model['pose']
        args = ['-world', world, '-name', model['entity_name'],
                '-file', os.path.join(pkg, 'models', model['model_file']),
                '-allow_renaming', 'false']
        for flag, field in [('-x', 'x'), ('-y', 'y'), ('-z', 'z'),
                            ('-R', 'roll'), ('-P', 'pitch'), ('-Y', 'yaw')]:
            args.extend([flag, str(p[field])])
        return Node(package='ros_gz_sim', executable='create',
                    name='spawn_' + key, output='screen', arguments=args)

    desk, lamp = model_node('desk'), model_node('lamp')
    robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg, 'launch', 'spawn_robot.launch.py')))

    def on_desk(event, context):
        return [lamp] if event.returncode == 0 else [
            LogInfo(msg='[SENTINEL] Desk spawn failed; inserting Go2'), robot]

    def on_lamp(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='[SENTINEL] Lamp spawn failed; inserting Go2') , robot]
        return [robot]

    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=desk, on_exit=on_desk)),
        RegisterEventHandler(OnProcessExit(target_action=lamp, on_exit=on_lamp)),
        desk,
    ])
