#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判定"车为什么停不住"：先看实时倍率，再看零速指令有没有真的接到。

  real_time_factor = 仿真时间增量 / 墙钟增量。如果远大于 1，那 `settle()` 里
  "墙钟 3 s" 实际是几十秒仿真时间，斜面自滚当然能跑出 17 m —— 那不是刹车失败，
  是测量口径错了。
  另外统计每辆车 /car_N/cmd_vel 的发布者数与实测位移，确认主动刹车有没有生效。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates


class Probe(Node):
    def __init__(self):
        super().__init__('rt_probe')
        self.clock_pts = []
        self.latest = {}
        self.vel = {}
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)

    def on_clock(self, m):
        self.clock_pts.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9))

    def on_states(self, m):
        for n, p, t in zip(m.name, m.pose, m.twist):
            self.latest[n] = (p.position.x, p.position.y, p.position.z)
            self.vel[n] = math.hypot(t.linear.x, t.linear.y)


def main():
    rclpy.init()
    node = Probe()
    wall0 = time.monotonic()
    while time.monotonic() - wall0 < 3.0 and not node.clock_pts:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.clock_pts:
        print('!! 3 s 内没有 /clock，gzserver 没在跑仿真')
        return
    marks = {}
    t0w, t0s = node.clock_pts[0]
    print('起始：墙钟 %.1f 仿真 %.1f 车辆 %d 辆' % (t0w, t0s, len(node.latest)))
    while True:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        for n, p in list(node.latest.items()):
            marks.setdefault(n, []).append((now, p[0], p[1]))
        if now - t0w >= 15.0:
            break

    lastw, lasts = node.clock_pts[-1]
    rf = (lasts - t0s) / (lastw - t0w)
    print('--- 实时倍率 ---')
    print('  墙钟 %.2f s 走仿真 %.2f s → real_time_factor = %.2f'
          % (lastw - t0w, lasts - t0s, rf))
    print('  /clock 帧数 %d（约 %.1f Hz 仿真侧上报）'
          % (len(node.clock_pts), len(node.clock_pts) / (lastw - t0w)))
    print('--- 15 s 墙钟内每辆车位移 / 上报线速度 ---')
    for n in sorted(marks):
        seq = marks[n]
        a, b = seq[0], seq[-1]
        d = math.hypot(b[1] - a[1], b[2] - a[2])
        print('  %-6s (%7.2f,%7.2f)→(%7.2f,%7.2f) 位移 %6.2f m（%.2f m/s 墙钟口径）'
              ' twist %4.2f m/s 最新 z %5.2f'
              % (n, a[1], a[2], b[1], b[2], d, d / (b[0] - a[0]),
                 node.vel.get(n, float('nan')), node.latest[n][2]))
    print('--- cmd_vel 发布者（主动刹车是否在场）---')
    import subprocess
    for n in ('car_1', 'car_2', 'car_3', 'car_4', 'car_5'):
        out = subprocess.run(['ros2', 'topic', 'info', '/%s/cmd_vel'],
                             capture_output=True, text=True).stdout.strip().replace('\n', ' | ')
        print('  %s' % out)


if __name__ == '__main__':
    main()
