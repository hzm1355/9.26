#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两次采样 /model_states，打印每辆车的平面位置、位移和航向，用来判断车队是"在跑"
还是"卡在哪儿"。卡口长时间不出图时先跑这个：驱动节点活着不等于车在动（红灯锁死、
跟车距过小互顶、或复位后卡在建筑边上都可能让整队静止）。

用法：python3 fleet_peek.py [间隔秒=10] [车名前缀=car_]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates


class Peek(Node):
    def __init__(self):
        super().__init__('fleet_peek')
        self.snap = None
        self.create_subscription(ModelStates, '/model_states', self.on, 10)

    def on(self, m):
        # ModelStates 的朝向在 pose[i].orientation 里，没有独立的 orientation 字段；
        # 线速度取 twist[i].linear.x（差速车的前进方向）。
        self.snap = {n: (p.position.x, p.position.y, p.position.z,
                         p.orientation, t.linear.x)
                     for n, p, t in zip(m.name, m.pose, m.twist) if n.startswith(PREFIX)}

    def yaw(self, q):
        return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


PREFIX = sys.argv[2] if len(sys.argv) > 2 else 'car_'
GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

rclpy.init()
node = Peek()
t0 = time.time()
while rclpy.ok() and node.snap is None and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.snap is None:
    print('!! 20 s 内没收到 /model_states（gzserver 的 ROS 插件没发或话题没匹配）')
    raise SystemExit(1)
a = dict(node.snap)
t0 = time.time()
while rclpy.ok() and time.time() - t0 < GAP:
    rclpy.spin_once(node, timeout_sec=0.1)
b = dict(node.snap)
print('%-8s %8s %8s %6s %8s   %8s %8s %6s %8s %8s' % (
    '车', 'x', 'y', 'z', '瞬时速度', '车', 'x', 'y', '位移', '航向'))
for k in sorted(a):
    ax, ay, az, aq, av = a[k]
    bx, by, bz, bq, bv = b[k]
    d = math.hypot(bx - ax, by - ay)
    print('%-8s %8.2f %8.2f %6.2f %8.2f   %-8s %8.2f %8.2f %6.2f m %8.1f' % (
        k, ax, ay, az, av, k, bx, by, d, node.yaw(bq)))
print('（间隔 %.0f s 内位移 ~0 且瞬时速度 ~0 的车就是停着的）' % GAP)
node.destroy_node()
rclpy.shutdown()
