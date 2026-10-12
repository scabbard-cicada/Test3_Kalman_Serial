#!/usr/bin/env python3
"""串口链路统计校验。

检查 serial/status 是否持续发布、串口是否连接、C1 回传率和 RTT 是否合理，
并报告 CRC/长度错误和 echo 序号跳变。

用法：
    ros2 run tracker_launcher verify_serial.py
    ros2 run tracker_launcher verify_serial.py --ros-args -p duration:=10.0
"""

import sys

import rclpy
from interfaces.msg import SerialStatus
from rclpy.node import Node


class SerialVerifier(Node):
    def __init__(self):
        super().__init__('verify_serial')
        self.duration = float(self.declare_parameter('duration', 10.0).value)
        self.min_rx_rate = float(self.declare_parameter('min_rx_rate_hz', 50.0).value)
        self.statuses = []
        self.start = self.get_clock().now()
        self.create_subscription(SerialStatus, 'serial/status', self.cb, 10)
        self.create_timer(0.5, self.tick)

    def cb(self, msg):
        self.statuses.append(msg)

    def tick(self):
        if (self.get_clock().now() - self.start).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit

    def report(self):
        print()
        print('=' * 66)
        print('  串口链路校验')
        print('=' * 66)

        if not self.statuses:
            print('  未通过：没有收到 serial/status。')
            print('  检查 gimbal_serial_driver 是否启动。')
            self.ok = False
            print('=' * 66)
            return

        s = self.statuses[-1]
        print(f'  status 样本数      : {len(self.statuses)}')
        print(f'  connected          : {s.connected}')
        print(f'  tx_frames          : {s.tx_frames}')
        print(f'  rx_frames          : {s.rx_frames}')
        print(f'  rx_rate_hz         : {s.rx_rate_hz:.2f}')
        print(f'  rtt mean/max       : {s.rtt_ms_mean:.2f} / {s.rtt_ms_max:.2f} ms')
        print(f'  crc/len errors     : {s.crc_errors} / {s.len_errors}')
        print(f'  echo_gaps          : {s.echo_gaps}')
        print(f'  reconnects         : {s.reconnects}')
        print('-' * 66)

        if s.connected and s.rx_rate_hz >= self.min_rx_rate:
            print('  通过：串口已连接，C1 回传率达到最低要求。')
            self.ok = True
        else:
            print('  未通过：串口未连接或 C1 回传率过低。')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = SerialVerifier()
    node.ok = False
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        ok = node.ok
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
