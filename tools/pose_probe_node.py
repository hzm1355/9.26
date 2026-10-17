#!/usr/bin/env python3
"""采样 /model_states：输出每辆车的位姿(含 z、姿态角)与世界速度，判断是否被物理引擎弹飞/翻倒。"""
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


def euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class P(Node):
    def __init__(self):
        super().__init__('pose_probe_tmp')
        self.n = 0
        self.create_subscription(ModelStates, '/model_states', self.cb, 10)

    def cb(self, m):
        if self.n >= 6:
            return
        self.n += 1
        line = []
        for name, pose, tw in zip(m.name, m.pose, m.twist):
            if not name.startswith('car_'):
                continue
            r, p, y = euler(pose.orientation)
            v = math.hypot(tw.linear.x, tw.linear.y)
            line.append(f'{name}:z={pose.position.z:.2f} r={r:.2f} p={p:.2f} '
                        f'({pose.position.x:.1f},{pose.position.y:.1f}) |v|={v:.2f}')
        print(f'[{self.n}] ' + ' || '.join(line), flush=True)


rclpy.init()
n = P()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
