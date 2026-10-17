#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#108 只读对照：六车的 odom 与 /model_states 真值差分，看"odom 虚高"是不是 car_6 独有。

#104 里 car_6 含 ω 指令时 odom 线速度比真值高 7~10 倍，我把它读成"驱动轮打滑"。但这个
比值必须有对照：gazebo_ros_diff_drive 在 odometrySource=wheel 下是用轮子编码器积分里程的，
任何车在有侧滑的转弯里都会虚高 —— 如果车队五车转弯时同样虚高 7~10 倍，那这条就不是 car_6
的 traction 证据，#85 剩下的唯一异常就只有"car_6 的纯 ω 段真值转角≈0"。

做法：全程只订阅不发布。按 /clock（仿真钟）做位姿差分，把每一拍同时记 odom 与真值，
按"真值角速度"分桶（直行桶 |ω|<0.05、转弯桶 |ω|>=0.05）分别求虚高倍数的中位数。
"""

import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import yaw_of, wrap, BEST   # noqa: E402
import rclpy                                   # noqa: E402
from rclpy.node import Node                    # noqa: E402
from nav_msgs.msg import Odometry              # noqa: E402
from rosgraph_msgs.msg import Clock            # noqa: E402
from gazebo_msgs.msg import ModelStates        # noqa: E402

CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']
TURN_EPS = 0.05


def med(xs):
    if not xs:
        return float('nan')
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class OdomTruth(Node):
    def __init__(self):
        super().__init__('odom_truth108')
        self.clock = 0.0
        self.pose = {}          # car -> (sim_t, x, y, yaw)
        self.odom = {}          # car -> (v, w)
        self.rows = {c: [] for c in CARS}
        self.create_subscription(ModelStates, '/model_states', self.on_states, BEST)
        self.create_subscription(Clock, '/clock', self.on_clock, BEST)
        for c in CARS:
            self.odom[c] = (0.0, 0.0)
            self.create_subscription(
                Odometry, '/%s/odom' % c,
                lambda m, cc=c: self._odom(cc, m), 10)

    def _odom(self, car, m):
        self.odom[car] = (float(m.twist.twist.linear.x),
                          float(m.twist.twist.angular.z))

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_states(self, m):
        for i, nm in enumerate(m.name):
            if nm in CARS:
                p = m.pose[i]
                self.pose[nm] = (self.clock, p.position.x, p.position.y,
                                 yaw_of(p.orientation))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    rclpy.init()
    n = OdomTruth()
    # 上一拍的位姿：差分要跨 /model_states 帧，不能跨 odom 帧
    prev = {c: None for c in CARS}
    last_flush = {c: None for c in CARS}
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        for c in CARS:
            p = n.pose.get(c)
            if p is None:
                continue
            if prev[c] is not None:
                dt = p[0] - prev[c][0]
                if dt > 1e-9 and last_flush[c] != p[0]:
                    tv = math.hypot(p[1] - prev[c][1], p[2] - prev[c][2]) / dt
                    tw = wrap(p[3] - prev[c][3]) / dt
                    n.rows[c].append((tv, tw, n.odom[c][0], n.odom[c][1]))
                    last_flush[c] = p[0]
            prev[c] = p

    print('#108 odom vs 真值（%.0f s 窗口，桶按真值角速度分）' % secs)
    print('  %-7s %6s %8s %8s %8s %8s %8s' % (
        '车', '帧数', '真值|v|', 'odom|v|', '真值|ω|', 'odom|ω|', '直行虚高'))
    for c in CARS:
        r = n.rows[c]
        if len(r) < 5:
            print('  %-7s 行数 %d，太少' % (c, len(r)))
            continue
        straight = [x for x in r if abs(x[1]) < TURN_EPS]
        turning = [x for x in r if abs(x[1]) >= TURN_EPS]

        def gain(rows):
            """odom|v| / 真值|v| 的中位数；真值太小的拍不算（除不动点）。"""
            g = [abs(o) / max(abs(t), 1e-3) for t, _, o, _ in rows if abs(t) > 0.02]
            return med(g)

        print('  %-7s %6d %8.3f %8.3f %8.4f %8.4f %8s' % (
            c, len(r), med([abs(x[0]) for x in r]), med([abs(x[2]) for x in r]),
            med([abs(x[1]) for x in r]), med([abs(x[3]) for x in r]),
            ('%.2f x（n=%d）' % (gain(straight), len(straight))) if straight else '无直行帧'))
        if turning:
            print('          转弯帧虚高 %.2f x（n=%d，odom|ω| 中位 %.3f vs 真值 %.3f）' % (
                gain(turning), len(turning),
                med([abs(x[3]) for x in turning]), med([abs(x[1]) for x in turning])))
        else:
            print('          本窗口没有转弯帧')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
