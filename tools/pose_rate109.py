#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#109 体检差分口径本身：/model_states 与 /clock 到底各是多少 Hz，逐拍差分是不是缩了 10 倍。

#108 出来六车的"odom|v| / 真值|v|"清一色 10.00 倍，连 ω 都是 0.628 -> 0.063。物理不会这么
整齐，符合已知的"两个话题速率差导致逐拍差分缩 n 倍"。若真是这样，#85 结论③（含 ω 指令时
odom 虚高 7~10 倍）就是我自己造出来的口径 bug，得撤掉。

免疫口径：把整段窗口内每一拍的 |Δ位| 直接累加成路径长度，再除以整段仿真时长（末钟-首钟）；
转角同理累加 |Δyaw|。这样无论 /model_states 比 /clock 快还是慢都不会缩倍。拿这两个"整段均速/
均角速度"和 odom 的均值比，才知道 odom 到底虚不虚高。

注：gazebo_msgs/ModelStates 没有 header（第一发就是栽在这里），只能用订阅端的 /clock。
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


def med(xs):
    if not xs:
        return float('nan')
    s = sorted(xs)
    return s[len(s) // 2]


class Probe(Node):
    def __init__(self):
        super().__init__('pose_rate109')
        self.clock = 0.0
        self.n_states = 0
        self.n_clock = 0
        self.ms_clock = []        # 每条 model_states 到达时的钟
        self.cl_step = []         # /clock 相邻增量
        self.odom = {c: [] for c in CARS}
        self.odom_w = {c: [] for c in CARS}
        self.prev = {}
        self.first = {}
        self.last = {}
        self.path = {c: 0.0 for c in CARS}
        self.turn = {c: 0.0 for c in CARS}
        self.create_subscription(ModelStates, '/model_states', self.on_states, BEST)
        self.create_subscription(Clock, '/clock', self.on_clock, BEST)
        for c in CARS:
            self.create_subscription(Odometry, '/%s/odom' % c,
                                     lambda m, cc=c: self._odom(cc, m), 10)

    def _odom(self, car, m):
        self.odom[car].append(abs(float(m.twist.twist.linear.x)))
        self.odom_w[car].append(abs(float(m.twist.twist.angular.z)))

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock and t > self.clock:
            self.cl_step.append(t - self.clock)
        self.clock = t
        self.n_clock += 1

    def on_states(self, m):
        self.n_states += 1
        self.ms_clock.append(self.clock)
        cur = {}
        for i, nm in enumerate(m.name):
            if nm in CARS:
                p = m.pose[i]
                cur[nm] = (p.position.x, p.position.y, yaw_of(p.orientation))
        for c, v in cur.items():
            if c not in self.first:
                self.first[c] = (self.clock, v)
            pv = self.prev.get(c)
            if pv is not None:
                self.path[c] += math.hypot(v[0] - pv[0], v[1] - pv[1])
                self.turn[c] += abs(wrap(v[2] - pv[2]))
            self.prev[c] = v
            self.last[c] = (self.clock, v)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Probe()
    t0 = time.time()
    end = t0 + secs
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    wall = time.time() - t0
    print('#109 差分口径体检（壁钟 %.1f s）' % wall)
    print('  /model_states 收到 %d 条 = %.1f Hz' % (n.n_states, n.n_states / wall))
    print('  /clock        收到 %d 条 = %.1f Hz' % (n.n_clock, n.n_clock / wall))
    if n.cl_step:
        print('  /clock 相邻增量：中位 %.4f s  最小 %.4f  最大 %.4f' % (
            med(n.cl_step), min(n.cl_step), max(n.cl_step)))
    # 相邻两条 model_states 之间钟跳了多少：中位应为 1/rate，若明显大于它=钟比话题慢
    ms_d = [b - a for a, b in zip(n.ms_clock, n.ms_clock[1:]) if b - a > 1e-12]
    if ms_d:
        print('  相邻 model_states 之间的钟增量（非零者）：中位 %.4f s  最小 %.4f  最大 %.4f  占 %.0f%%' % (
            med(ms_d), min(ms_d), max(ms_d), 100.0 * len(ms_d) / max(len(n.ms_clock) - 1, 1)))
    print('  %-7s %9s %9s %9s %9s %9s' % (
        '车', '整段均速', 'odom|v|', '整段均角', 'odom|ω|', '仿真时长'))
    for c in CARS:
        if c not in n.first:
            print('  %-7s 没看到' % c)
            continue
        span = n.last[c][0] - n.first[c][0]
        if span <= 0.5:
            print('  %-7s 仿真时长只有 %.2f s，不判' % (c, span))
            continue
        gsp = n.path[c] / span
        gyr = n.turn[c] / span
        osp = med(n.odom[c]) if n.odom[c] else float('nan')
        oyr = med(n.odom_w[c]) if n.odom_w[c] else float('nan')
        print('  %-7s %9.3f %9.3f %9.4f %9.4f %9.2f s   路径 %6.2f m 累积转角 %6.3f rad' % (
            c, gsp, osp, gyr, oyr, span, n.path[c], n.turn[c]))
        if n.odom[c]:
            print('          -> 整段均速/odom中位 = %.2f x   整段均角/odom中位 = %s' % (
                gsp / osp if osp > 1e-6 else float('nan'),
                '%.2f x' % (gyr / oyr) if oyr > 1e-4 else 'odom 角速度为 0'))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
