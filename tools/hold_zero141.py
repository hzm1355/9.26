#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 20 Hz 连发 N 秒零速，把 car_6 的轮速目标钉回 0。用法：hold_zero141.py [模型] [秒]

为什么必须有这一步：diff_drive 的 cmd_vel_timeout 实测不生效（#120），节点被 kill -9
之后插件会**保持最后一条指令继续跑**。本轮最后一条是 w=+0.35 的原地转，所以只杀摆渡车
不补零速的话，车会顶着落杆一直转下去，把轮位误差继续攒成下一次弹射。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MODEL = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0


class Hold(Node):
    def __init__(self):
        super().__init__('hold_zero141')
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % MODEL, 10)


rclpy.init()
n = Hold()
t0 = time.monotonic()
while time.monotonic() - t0 < SECS:
    n.pub.publish(Twist())
    rclpy.spin_once(n, timeout_sec=0.02)
    time.sleep(0.05)
print('已连发零速 %.1f s（%s）' % (SECS, MODEL))
rclpy.shutdown()
