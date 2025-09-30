#!/usr/bin/env python3

import os
import time
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    params = _read_config_file()

    scenario_replayer = Node(
        package='visualize_real_data',
        executable='scenario_replayer',
        name='scenario_replayer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, params]
    )

    send_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('visualize_real_data'),
                'launch',
                'send.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )


    return LaunchDescription([
        use_sim_time_arg,
        scenario_replayer,
        send_launch,
    ])

def _read_config_file():
    """__Explanation__:
    Read a YAML file and return its content.
    """
    yaml_file_path = Path(
        get_package_share_directory('visualize_real_data'),
        'config',
        'params.yaml'
    )

    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Merge pointcloud and entity parameters
    replayer_params = data.get('scenario_replayer', {})
    shared_params = data.get('shared', {})

    shared_params['entity_topic'] = shared_params['namespace'] + '/' + shared_params['entity_topic']

    return replayer_params | shared_params