#!/usr/bin/env python3
"""单独启动 virtual_locator（观测回放）。

    ros2 launch virtual_locator virtual_locator.launch.py
    ros2 launch virtual_locator virtual_locator.launch.py loop:=true

需要整套系统时用 tracker_launcher。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('virtual_locator')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'json_path', default_value=os.path.join(pkg, 'data', 'measurements.json')),
        DeclareLaunchArgument('loop', default_value='false'),
        Node(
            package='virtual_locator',
            executable='virtual_locator',
            name='virtual_locator',
            output='screen',
            parameters=[params_file, {
                'json_path': LaunchConfiguration('json_path'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
    ])
