#!/usr/bin/env python3
"""读 /gazebo/model_states 里某模型的当前位姿，打印成 car_vs_colliders 能直接用的 argv。

为什么要单独一个读位姿脚本：unwedge/probe 系列里我用的是"上一次读数 + 估计"，
接触挤出会让底盘自己爬出几十厘米，拿旧姿态算 OBB 会把"已经脱开"读成"还压着"
（memory「生效要用效果复核」）。

用法: pose155.py <模型名>
"""
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates

NAME = sys.argv[1] if len(sys.argv) > 1 else 'car_6'


class Reader:
    def __init__(self):
        rclpy.init()
        self.nd = Node('pose155')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.hit = None
        self.nd.create_subscription(ModelStates, '/model_states',
                                    self.on, q)

    def on(self, m):
        for i, n in enumerate(m.name):
            if n == NAME:
                p = m.pose[i].position
                o = m.pose[i].orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                                 1 - 2 * (o.y ** 2 + o.z ** 2))
                roll = math.atan2(2 * (o.w * o.x + o.y * o.z),
                                  1 - 2 * (o.x ** 2 + o.y ** 2))
                self.hit = (p.x, p.y, p.z, yaw, roll)

    def run(self):
        t0 = math.inf
        import time
        t0 = time.time()
        while time.time() - t0 < 6.0 and self.hit is None:
            rclpy.spin_once(self.nd, timeout_sec=0.1)
        if self.hit is None:
            print('FAIL /gazebo/model_states 6 s 没有 %s 的帧' % NAME)
            rclpy.shutdown()
            return 1
        x, y, z, yaw, roll = self.hit
        print('%s 当前位姿 x=%.3f y=%.3f z=%.3f yaw=%+.3f roll=%+.3f' % (
            NAME, x, y, z, yaw, roll))
        print('argv: %.3f %.3f %.3f' % (x, y, yaw))
        rclpy.shutdown()
        return 0


if __name__ == '__main__':
    sys.exit(Reader().run())
