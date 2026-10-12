#!/usr/bin/env python3
"""EKF 输出的基础校验。

检查观测与滤波输出是否按时间戳配对、数值是否有限、输出是否有运动范围，
并统计 NIS。它不依赖真值旁路，适合先检查学生的 EKF 是否真正跑起来。

用法：
    ros2 run tracker_launcher verify_ekf.py
    ros2 run tracker_launcher verify_ekf.py --ros-args -p duration:=15.0
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from interfaces.msg import PolarMeasurement
from rclpy.node import Node
from std_msgs.msg import Float64


class EkfVerifier(Node):
    def __init__(self):
        super().__init__('verify_ekf')
        self.duration = float(self.declare_parameter('duration', 15.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 100).value)
        self.nis_limit = float(self.declare_parameter('nis_limit', 1.0e5).value)

        self.measurements = {}
        self.filtered = {}
        self.nis = []
        self.start = self.get_clock().now()
        self.create_subscription(PolarMeasurement, 'target/measurement', self.measurement_cb, 50)
        self.create_subscription(PointStamped, 'target/filtered', self.filtered_cb, 50)
        self.create_subscription(Float64, 'target/nis', self.nis_cb, 50)
        self.create_timer(0.5, self.tick)

    @staticmethod
    def key(msg):
        return msg.header.stamp.sec, msg.header.stamp.nanosec

    def measurement_cb(self, msg):
        x = msg.station_x + msg.range * math.cos(msg.bearing)
        y = msg.station_y + msg.range * math.sin(msg.bearing)
        self.measurements[self.key(msg)] = (x, y)

    def filtered_cb(self, msg):
        self.filtered[self.key(msg)] = (msg.point.x, msg.point.y)

    def nis_cb(self, msg):
        if math.isfinite(msg.data):
            self.nis.append(float(msg.data))

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        if elapsed >= self.duration:
            self.report()
            raise SystemExit

    def report(self):
        pairs = []
        for key, raw in self.measurements.items():
            if key in self.filtered:
                fx, fy = self.filtered[key]
                pairs.append((raw[0], raw[1], fx, fy))

        print()
        print('=' * 66)
        print('  EKF 输出校验')
        print('=' * 66)

        if len(pairs) < self.min_samples:
            print(f'  未通过：只配对到 {len(pairs)} 个样本，至少需要 {self.min_samples} 个')
            print('  请检查 virtual_locator、tracker_node 和话题名称。')
            print('=' * 66)
            self.ok = False
            return

        if not all(math.isfinite(v) for pair in pairs for v in pair):
            print('  未通过：滤波输出或原始观测包含非有限数值')
            self.ok = False
            print('=' * 66)
            return

        xs = [p[2] for p in pairs]
        ys = [p[3] for p in pairs]
        nis_bad = sum(1 for v in self.nis if v > self.nis_limit)
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))

        print(f'  时间戳配对样本     : {len(pairs)}')
        print(f'  NIS 样本数         : {len(self.nis)}')
        print(f'  NIS 均值           : {sum(self.nis) / len(self.nis):.3f}' if self.nis else
              '  NIS 均值           : 无数据')
        print(f'  NIS 超过上限       : {nis_bad}')
        print(f'  滤波 x 范围        : [{min(xs):+.3f}, {max(xs):+.3f}] m')
        print(f'  滤波 y 范围        : [{min(ys):+.3f}, {max(ys):+.3f}] m')
        print(f'  轨迹跨度           : {span:.3f} m')
        print('-' * 66)

        if nis_bad == 0 and span > 0.1:
            print('  通过：EKF 持续输出有限状态，NIS 正常，轨迹不是常量。')
            self.ok = True
        else:
            print('  未通过：NIS 异常或滤波轨迹没有有效运动。')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = EkfVerifier()
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
