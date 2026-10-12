"""单独启动串口驱动（可选：同时启动软件下位机）。

    ros2 launch gimbal_serial_driver serial.launch.py port:=/dev/ttyUSB0
    ros2 launch gimbal_serial_driver serial.launch.py fake:=true

接线联调时加 log_frames:=true，会把每一帧收发的十六进制打出来。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    fake = LaunchConfiguration('fake').perform(context).lower() == 'true'
    port = '/tmp/gimbal_fake_mcu' if fake else LaunchConfiguration('port').perform(context)
    params = os.path.join(get_package_share_directory('gimbal_serial_driver'), 'config',
                          'params.yaml')
    log_frames = LaunchConfiguration('log_frames').perform(context).lower() == 'true'
    nodes = []
    if fake:
        nodes.append(Node(package='gimbal_serial_driver', executable='fake_mcu_node',
                          name='fake_mcu', output='screen',
                          parameters=[{'link_path': port, 'auto_arm': True}]))
    nodes.append(Node(package='gimbal_serial_driver', executable='gimbal_serial_driver_node',
                      name='gimbal_serial_driver', output='screen',
                      parameters=[params, {'device_name': port, 'log_frames': log_frames}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fake', default_value='false'),
        DeclareLaunchArgument('log_frames', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
