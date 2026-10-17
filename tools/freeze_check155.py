#!/usr/bin/env python3
"""判"物理到底停没停"：10 s 内按 /model_states 算每辆车的位移，并报 /clock 走了多少。

为什么需要它：155 轮 drive_probe 的四段激励响应完全一样（转向 +0.0001 rad、平动
0.0021 m），而底盘其实在慢慢朝远离灯杆的方向被挤出 —— 这个形态既像"驱动实例没出力"
（物理还在跑，接触自己往外推），也像"世界被暂停了"（什么都冻着，那 0.002 m 只是我
差分到的最后一帧残留）。两种成因的处理完全不同，所以先看别车和时钟。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']


class Watch:
    def __init__(self):
        rclpy.init()
        self.nd = Node('freeze_check155')
        self.q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.t0 = None
        self.t1 = None
        self.a = {}
        self.b = {}
        self.clock = []
        self.nd.create_subscription(ModelStates, '/model_states', self.on_states, self.q)
        self.nd.create_subscription(Clock, '/clock', self.on_clock, self.q)

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        self.clock.append(t)
        if self.t0 is None:
            self.t0 = t
        self.t1 = t

    def on_states(self, m):
        for i, n in enumerate(m.name):
            if n in CARS:
                p = m.pose[i].position
                o = m.pose[i].orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                                 1 - 2 * (o.y ** 2 + o.z ** 2))
                cur = (p.x, p.y, yaw)
                self.b[n] = cur
                if n not in self.a:
                    self.a[n] = cur


def main():
    w = Watch()
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    t_end = time.time() + secs
    while time.time() < t_end:
        rclpy.spin_once(w.nd, timeout_sec=0.05)
    print('  采样 %.1f s 墙钟，/clock 帧数 %d，仿真时间走了 %+.3f s'
          % (secs, len(w.clock), (w.t1 - w.t0) if w.t0 is not None else float('nan')))
    if not w.a:
        print('  FAIL 没有 /model_states 帧 —— 这个结论不能当成"物理停了"，'
              '要另外查 gzserver 是否还在')
        rclpy.shutdown()
        return 1
    for n in CARS:
        if n in w.b:
            d = math.hypot(w.b[n][0] - w.a[n][0], w.b[n][1] - w.a[n][1])
            dyaw = wrapd(w.b[n][2] - w.a[n][2])
            print('  %-6s (%7.3f,%7.3f)→(%7.3f,%7.3f) 位移 %.4f m  Δyaw %+.4f rad%s'
                  % (n, w.a[n][0], w.a[n][1], w.b[n][0], w.b[n][1], d, dyaw,
                     '   <-- 被顶住/被夹住' if 0 < d < 0.02 else ''))
    moving = sum(1 for n in CARS if n in w.b and
                 math.hypot(w.b[n][0] - w.a[n][0], w.b[n][1] - w.a[n][1]) > 0.05)
    print('  结论：%d/6 辆车在 10 s 内位移 >0.05 m —— %s'
          % (moving, '物理在跑，car_6 不动是驱动侧或接触侧的问题'
             if moving else '全场地都冻着，先查世界是不是被暂停了'))
    rclpy.shutdown()
    return 0


def wrapd(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


if __name__ == '__main__':
    sys.exit(main())
