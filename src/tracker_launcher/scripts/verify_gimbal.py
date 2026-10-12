#!/usr/bin/env python3
"""云台命令与反馈校验。

检查 gimbal/cmd、gimbal/feedback 是否正常、下位机是否进入 ARMED，以及命令角和
实际角之间的误差。真实电机测试前先用 fake MCU 跑一遍。

用法：
    ros2 run tracker_launcher verify_gimbal.py
    ros2 run tracker_launcher verify_gimbal.py --ros-args -p duration:=20.0
"""

import bisect
import math
import sys

import rclpy
from interfaces.msg import GimbalCmd, GimbalFeedback
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class GimbalVerifier(Node):
    def __init__(self):
        super().__init__('verify_gimbal')
        self.duration = float(self.declare_parameter('duration', 20.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 300).value)
        self.rms_tolerance_deg = float(self.declare_parameter('rms_tolerance_deg', 3.0).value)
        self.max_tolerance_deg = float(self.declare_parameter('max_tolerance_deg', 8.0).value)

        self.cmd = []
        self.feedback = []
        best_effort = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(GimbalCmd, 'gimbal/cmd', self.cmd_cb, 100)
        self.create_subscription(GimbalFeedback, 'gimbal/feedback', self.feedback_cb, best_effort)
        self.start = self.get_clock().now()
        self.create_timer(0.5, self.tick)

    def cmd_cb(self, msg):
        self.cmd.append((self.get_clock().now().nanoseconds * 1e-9,
                         math.degrees(msg.yaw), msg.mode, msg.target_valid))

    def feedback_cb(self, msg):
        self.feedback.append((self.get_clock().now().nanoseconds * 1e-9,
                              math.degrees(msg.yaw), msg.state, msg.error_flags))

    def tick(self):
        if (self.get_clock().now() - self.start).nanoseconds * 1e-9 >= self.duration:
            self.report()
            raise SystemExit

    def report(self):
        print()
        print('=' * 66)
        print('  云台命令/反馈校验')
        print('=' * 66)

        if len(self.cmd) < self.min_samples or len(self.feedback) < self.min_samples:
            print(f'  未通过：cmd={len(self.cmd)}，feedback={len(self.feedback)}，'
                  f'至少各需要 {self.min_samples} 帧')
            self.ok = False
            print('=' * 66)
            return

        fb_times = [x[0] for x in self.feedback]
        errors = []
        states = {}
        error_flags = {}
        for t, yaw, mode, valid in self.cmd:
            if mode != GimbalCmd.MODE_TRACK or not valid:
                continue
            i = bisect.bisect_left(fb_times, t)
            if i >= len(self.feedback):
                i = len(self.feedback) - 1
            if i > 0 and abs(fb_times[i - 1] - t) < abs(fb_times[i] - t):
                i -= 1
            if abs(fb_times[i] - t) <= 0.02:
                errors.append(yaw - self.feedback[i][1])

        for _, _, state, flags in self.feedback:
            states[state] = states.get(state, 0) + 1
            error_flags[flags] = error_flags.get(flags, 0) + 1

        print(f'  命令帧数           : {len(self.cmd)}')
        print(f'  反馈帧数           : {len(self.feedback)}')
        print(f'  TRACK 配对数       : {len(errors)}')
        print(f'  state 分布         : {states}')
        print(f'  error_flags 分布   : { {hex(k): v for k, v in error_flags.items()} }')

        if not errors:
            print('  未通过：没有匹配到 TRACK 命令和反馈。')
            self.ok = False
            print('=' * 66)
            return

        rms = math.sqrt(sum(e * e for e in errors) / len(errors))
        max_abs = max(abs(e) for e in errors)
        armed = states.get(GimbalFeedback.STATE_ARMED, 0)
        print(f'  跟踪误差 RMS      : {rms:.3f} deg')
        print(f'  跟踪误差最大值    : {max_abs:.3f} deg')
        print(f'  ARMED 比例        : {100.0 * armed / len(self.feedback):.1f}%')
        print('-' * 66)

        if rms <= self.rms_tolerance_deg and max_abs <= self.max_tolerance_deg and armed > 0:
            print('  通过：下位机已解锁，命令和反馈在容差内。')
            self.ok = True
        else:
            print('  未通过：检查 KEY 解锁、C1/C2、PID、前馈和电机限位。')
            self.ok = False
        print('=' * 66)


def main():
    rclpy.init()
    node = GimbalVerifier()
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
