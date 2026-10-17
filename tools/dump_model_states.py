#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出场景里所有实体的位姿，用于判断某个坐标附近是否已经站着别的东西。

用法：python3 tools/dump_model_states.py [x y 半径] —— 带三个数就只打印该圆邻域内
的实体；不带参数打印全部。
"""
import math
import sys

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

NEAR = [float(sys.argv[i]) for i in (1, 2, 3)] if len(sys.argv) > 3 else None


class Dump(Node):
    def __init__(self):
        super().__init__('dump_model_states')
        self.last = None
        self.create_subscription(ModelStates, '/model_states', self._cb,
                                 qos_profile_sensor_data)

    def _cb(self, msg):
        self.last = msg


def yaw_of(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def main():
    rclpy.init()
    node = Dump()
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.last is not None:
            break
    if node.last is None:
        print('!! 15 s 没收到 /model_states')
        return 1
    print('实体总数 = %d' % len(node.last.name))
    for i, n in enumerate(node.last.name):
        p = node.last.pose[i].position
        if NEAR and math.hypot(p.x - NEAR[0], p.y - NEAR[1]) > NEAR[2]:
            continue
        print('  %-22s (%8.2f, %8.2f, %6.2f)  yaw=%+7.1f' % (
            n, p.x, p.y, p.z, yaw_of(node.last.pose[i].orientation)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
