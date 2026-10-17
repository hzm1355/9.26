#!/usr/bin/env python3
"""把 car_6 从出口闸口袋的楔住位挪开，再复核实时位姿。

为什么要"前走"：现位 (7.620,-12.012) yaw=-1.920（车头南偏西），与出口闸铰点柱
(7.45,-11.05) 重叠 -0.027 m，压上去的是**车尾左角**（车尾中心 (7.90,-11.26) 再往
左 0.40 就到 (7.52,-11.12)，离柱 0.08 m）—— 柱子在车北侧，车头朝南，所以前走
才是远离它的方向；倒车反而会把它顶得更深。往前 6 s 只走约 0.6 m，落点还在
南口袋内，不碰南侧路缘 c130（y≈-14.0）。

倒完之后不直接摆位钉坐标（memory「摆位弹射」：根链接瞬移会把子链接甩成弹射能量），
只发速度指令让物理自己走这一步。

用法: backout155.py <v> <秒>   默认 v=-0.10 倒 6 s
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

V = float(sys.argv[1]) if len(sys.argv) > 1 else -0.10
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0


class Op:
    def __init__(self):
        rclpy.init()
        self.nd = Node('backout155')
        self.q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.pose = None
        self.nd.create_subscription(ModelStates, '/model_states', self.on_state, self.q)
        # 发布端保持默认 RELIABLE（QoS 不匹配会静默零投递，见 wheel_spin155 同款注释）
        self.pub = self.nd.create_publisher(Twist, '/car_6/cmd_vel', 10)

    def on_state(self, m):
        for i, n in enumerate(m.name):
            if n == 'car_6':
                p = m.pose[i].position
                o = m.pose[i].orientation
                yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                                 1 - 2 * (o.y ** 2 + o.z ** 2))
                self.pose = (p.x, p.y, yaw)

    def read(self):
        for _ in range(40):
            rclpy.spin_once(self.nd, timeout_sec=0.1)
            if self.pose:
                return self.pose
        return None

    def drive(self, v, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            m = Twist()
            m.linear.x = v
            self.pub.publish(m)
            rclpy.spin_once(self.nd, timeout_sec=0.02)


def main():
    op = Op()
    a = op.read()
    if not a:
        print('FAIL 6 s 没读到 /model_states 里的 car_6')
        return 1
    print('起点   (%.3f, %.3f) yaw=%+.3f' % a)
    op.drive(V, SECS)
    op.drive(0.0, 2.0)          # 收尾无条件连发零速（memory 规则）
    rclpy.spin_once(op.nd, timeout_sec=0.5)
    b = op.read()
    print('终点   (%.3f, %.3f) yaw=%+.3f' % b)
    print('argv: %.3f %.3f %.3f' % b)
    print('位移 %.3f m（油门 %.2f m/s x %.1f s，理论约 %.2f m）'
          % (math.hypot(b[0] - a[0], b[1] - a[1]), V, SECS, abs(V) * SECS))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
