#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量 /clock 的发布节奏：相邻两条 /clock 的仿真时间差就是在线探针的时间分辨率上限。
用法：python3 clock_hz.py [秒=8]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)


class T(Node):
    def __init__(self, dur):
        super().__init__('clock_hz')
        self.dur = dur
        self.t0 = time.monotonic()
        self.n = 0
        self.last_wall = None
        self.max_wall_gap = 0.0
        self.last_sim = None
        self.max_sim_gap = 0.0
        self.sim0 = self.sim1 = 0.0
        self.create_subscription(Clock, '/clock', self.on, QOS)
        self.create_timer(0.5, self.done)

    def on(self, m):
        w = time.monotonic()
        s = m.clock.sec + m.clock.nanosec * 1e-9
        if self.last_wall is not None:
            self.max_wall_gap = max(self.max_wall_gap, w - self.last_wall)
            self.max_sim_gap = max(self.max_sim_gap, s - self.last_sim)
        else:
            self.sim0 = s
        self.last_wall, self.last_sim, self.sim1 = w, s, s
        self.n += 1

    def done(self):
        e = time.monotonic() - self.t0
        if e < self.dur:
            return
        print('采样 %.1f s：wall 收到 /clock %d 条 ≈%.1f Hz，wall 最大间隔 %.3f s；'
              '相邻 /clock 仿真时间差最大 %.3f s（= 探针时间分辨率上限）；'
              '仿真推进 %.2f s，实时倍率 %.3f' % (
                  e, self.n, self.n / e, self.max_wall_gap, self.max_sim_gap,
                  self.sim1 - self.sim0, (self.sim1 - self.sim0) / e))
        sys.stdout.flush()
        os._exit(0)


def main():
    rclpy.init()
    rclpy.spin(T(float(sys.argv[1]) if len(sys.argv) > 1 else 8.0))


if __name__ == '__main__':
    main()
