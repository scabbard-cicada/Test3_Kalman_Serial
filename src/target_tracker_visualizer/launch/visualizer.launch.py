#!/usr/bin/env python3
"""单独启动可视化节点。

    ros2 launch target_tracker_visualizer visualizer.launch.py

它订阅 target/measurement、target/filtered、gimbal/cmd、gimbal/feedback，
发布 viz/raw_path、viz/filtered_path、viz/gimbal 以及 map→station→gimbal 的 TF。
需要配合 virtual_locator / target_tracker / gimbal_serial_driver 才有数据。
整套启动用 tracker_launcher。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('target_tracker_visualizer')
    params_file = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='target_tracker_visualizer',
            executable='visualizer_node',
            name='visualizer_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
