#!/usr/bin/env python3
"""一键启动整套目标跟踪系统。

三个计算节点（virtual_locator / tracker_node / visualizer_node）加载到同一个
多线程 Component 容器，开启进程内通信；串口 I/O、软件下位机和 Foxglove bridge
保持独立进程，避免阻塞串口回调影响滤波和可视化。每个包的参数各自加载，再由
override_params 统一覆盖。

用法:
    # 没有 C 板：软件下位机（默认）
    ros2 launch tracker_launcher tracker.launch.py

    # 接真 C 板（USB-TTL 接 C 板 USART6）
    ros2 launch tracker_launcher tracker.launch.py mcu:=real port:=/dev/ttyUSB0

    # 不要串口，只看 EKF
    ros2 launch tracker_launcher tracker.launch.py mcu:=none

    # 叠加参数覆盖
    ros2 launch tracker_launcher tracker.launch.py override_params:=/path/to/my.yaml

其它:
    loop:=true             循环回放（默认开）
    loop_gap:=4.0          周期间停顿（s）
    fault_drop:=0.05       软件下位机随机丢 5% 的回传帧
    fault_ber:=1e-4        软件下位机每比特以 1e-4 的概率翻转

查看可视化：启动后浏览器打开 https://app.foxglove.dev，
Open connection -> Foxglove WebSocket -> ws://localhost:8765
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _serial_nodes(context):
    mcu = LaunchConfiguration('mcu').perform(context)
    if mcu == 'none':
        return []
    if mcu not in ('fake', 'real'):
        raise RuntimeError(f"mcu:= 只能是 fake / real / none，收到 '{mcu}'")

    serial_pkg = get_package_share_directory('gimbal_serial_driver')
    serial_params = os.path.join(serial_pkg, 'config', 'params.yaml')
    fake_link = '/tmp/gimbal_fake_mcu'
    port = fake_link if mcu == 'fake' else LaunchConfiguration('port').perform(context)

    nodes = [Node(
        package='gimbal_serial_driver',
        executable='gimbal_serial_driver_node',
        name='gimbal_serial_driver',
        output='screen',
        parameters=[serial_params, {'device_name': port}],
    )]
    if mcu == 'fake':
        nodes.insert(0, Node(
            package='gimbal_serial_driver',
            executable='fake_mcu_node',
            name='fake_mcu',
            output='screen',
            parameters=[{
                'link_path': fake_link,
                'auto_arm': True,
                'drop_rate': float(LaunchConfiguration('fault_drop').perform(context)),
                'bit_error_rate': float(LaunchConfiguration('fault_ber').perform(context)),
            }],
        ))
    return nodes


def generate_launch_description():
    virtual_pkg = get_package_share_directory('virtual_locator')
    tracker_pkg = get_package_share_directory('target_tracker')
    viz_pkg = get_package_share_directory('target_tracker_visualizer')
    launcher_pkg = get_package_share_directory('tracker_launcher')

    override_params = LaunchConfiguration('override_params')

    components = [
        ComposableNode(
            package='virtual_locator',
            plugin='virtual_locator::MeasurementPublisher',
            name='virtual_locator',
            parameters=[
                os.path.join(virtual_pkg, 'config', 'params.yaml'),
                {'json_path': LaunchConfiguration('json_path'),
                 'loop': LaunchConfiguration('loop'),
                 'loop_gap': LaunchConfiguration('loop_gap')},
                override_params,
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='target_tracker',
            plugin='target_tracker::TrackerNode',
            name='tracker_node',
            parameters=[
                os.path.join(tracker_pkg, 'config', 'params.yaml'),
                override_params,
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='target_tracker_visualizer',
            plugin='target_tracker_visualizer::VisualizerNode',
            name='visualizer_node',
            parameters=[
                os.path.join(viz_pkg, 'config', 'params.yaml'),
                override_params,
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'override_params',
            default_value=os.path.join(launcher_pkg, 'config', 'empty.yaml')),
        DeclareLaunchArgument(
            'json_path',
            default_value=os.path.join(virtual_pkg, 'data', 'measurements.json')),
        DeclareLaunchArgument('mcu', default_value='fake'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fault_drop', default_value='0.0'),
        DeclareLaunchArgument('fault_ber', default_value='0.0'),
        DeclareLaunchArgument('bridge_port', default_value='8765'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('loop_gap', default_value='4.0'),
        ComposableNodeContainer(
            name='tracker_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            arguments=['--executor-type', 'multi-threaded'],
            composable_node_descriptions=components,
            output='both',
            emulate_tty=True,
        ),
        OpaqueFunction(function=_serial_nodes),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': LaunchConfiguration('bridge_port'),
                         'address': '0.0.0.0',
                         'send_buffer_limit': 10000000,
                         'use_compression': False}],
        ),
    ])
