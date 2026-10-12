#!/usr/bin/env python3
"""单独启动 target_tracker 核心跟踪节点。

    ros2 launch target_tracker tracker.launch.py

它订阅 target/measurement，发布 target/filtered、target/nis、gimbal/cmd。
需要 virtual_locator 提供观测才有数据。整套启动用 tracker_launcher。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('target_tracker')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='target_tracker',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
