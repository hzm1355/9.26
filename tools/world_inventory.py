#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性列出 /model_states 里所有实体及位姿，用来抓"路上多了个不该在的东西"。

复位风暴的定位思路：驱动每次把车摆回车道，4 s 内车又被推到 |x|>=22 / |y|>=19（越界阈值
本身），说明车在车道上被某个静止碰撞体沿车头弹开。这种"多出来的碰撞体"通常是前几轮调试
遗留的实体（卡口龙门架、测试用牌架、debug 模型），它不在 world 文件里、也不在任何脚本的
清理列表里，只能从实体全名清单里认出来。

用法：python3 world_inventory.py [过滤关键字]
"""
import math
import sys

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


rclpy.init()
nd = Node('world_inventory')
snap = {}


def on(m):
    snap.update({n: (p, t) for n, p, t in zip(m.name, m.pose, m.twist)})


nd.create_subscription(ModelStates, '/model_states', on, 20)
t0 = __import__('time').monotonic()
while not snap and __import__('time').monotonic() - t0 < 25:
    rclpy.spin_once(nd, timeout_sec=0.1)
if not snap:
    print('!! 25 s 内没收到 /model_states')
    raise SystemExit(1)
__import__('time').sleep(1.0)
rclpy.spin_once(nd, timeout_sec=0.1)

KEY = sys.argv[1] if len(sys.argv) > 1 else ''
print('实体总数 %d' % len(snap))
print('%-26s %8s %8s %7s %8s' % ('名称', 'x', 'y', 'z', 'yaw°'))
for n in sorted(snap):
    if KEY and KEY not in n:
        continue
    p = snap[n][0]
    print('%-26s %8.2f %8.2f %7.2f %8.1f' % (
        n, p.position.x, p.position.y, p.position.z, yaw_of(p.orientation)))
nd.destroy_node()
rclpy.shutdown()
