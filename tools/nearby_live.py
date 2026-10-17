#!/usr/bin/env python3
"""列出 /model_states 里离 car_6 最近的模型（活位姿，不是 world 文件里的初始位姿）。

为什么不用 world 文件筛：world 里的 <pose> 是出生位姿，而 Ring B 那几辆车一直在跑，
car_6 周围此刻站着谁，只有运行时位姿流知道。上一轮用 world 粗筛给出"1.5 m 外是
parking_booth"这种结论，对一辆会动的车根本没有意义。

判读：diff_drive 车"发 ω 有效但 yaw 10 s 只变 1e-4 rad"有三种外观一样的成因 ——
顶到实体、轮子悬空（翻车/被抬上台阶）、diff_drive 插件实例卡死。这条先把第一种
（有没有邻居贴得比两车半长之和还近）钉死，剩下两种才需要动仿真。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates

REF = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
RADIUS = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def main():
    rclpy.init()
    nd = Node('nearby_live')
    q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST)
    snap = {}

    def on_states(m):
        got = {}
        for i, n in enumerate(m.name):
            p = m.pose[i].position
            got[n] = (p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
        if REF in got:
            snap['v'] = got

    nd.create_subscription(ModelStates, '/model_states', on_states, q)
    t0 = time.time()
    while time.time() - t0 < 8.0 and 'v' not in snap:
        rclpy.spin_once(nd, timeout_sec=0.2)
    rclpy.shutdown()

    if 'v' not in snap:
        print('FAIL 8 s 内没收到含 %s 的 model_states' % REF)
        return 1
    a = snap['v'][REF]
    print('  参考 %s: (%.3f, %.3f) z=%.3f yaw=%+.2f' % (REF, a[0], a[1], a[2], a[3]))
    rows = []
    for n, p in snap['v'].items():
        if n == REF:
            continue
        d = math.hypot(p[0] - a[0], p[1] - a[1])
        if d <= RADIUS:
            rows.append((d, n, p))
    rows.sort()
    if not rows:
        print('  %.1f m 内没有任何其它模型 —— "顶到实体"这一支排除' % RADIUS)
    for d, n, p in rows:
        print('  %-28s (%7.3f,%7.3f) z=%.3f  间距 %.2f m  朝向差 %+.2f' % (
            n, p[0], p[1], p[2], d, math.atan2(math.sin(p[3] - a[3]),
                                               math.cos(p[3] - a[3]))))
    print('  （共 %d 个模型在窗口内，总模型数 %d）' % (len(rows), len(snap['v'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
