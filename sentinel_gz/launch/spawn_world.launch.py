#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('sentinel_gz')
    description = get_package_share_directory('sentinel_description')
    with open(os.path.join(pkg, 'config', 'simulation.yaml'), encoding='utf-8') as f:
        config = yaml.safe_load(f)['simulation']

    paths = [os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
             os.path.join(pkg, 'models'), pkg, description]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r ' if not config['paused'] else '',
            '-v ', str(config['verbose']), ' ',
            os.path.join(pkg, 'worlds', config['world_file']),
            ' --gui-config ', os.path.join(pkg, 'config', config['gui_config'])
        ]}.items())

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(filter(None, paths))),
        gazebo,
    ])
