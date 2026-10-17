#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直接问一个问题：现在发 /car_N/cmd_vel，车到底动不动。

为什么要单独问这个：本轮受控跑里 car_2 被 0.70 m/s 指令顶了 18 s，位姿**一格没动**
（横向偏差 0.00 m、终点还是起点）。而现场驱动节点活着的时候五辆车也常年"位移 0.00 m"
停在 (-2.2,-1.15) 这种车道外的位置上。这两种现象都用同一个解释才通：cmd_vel 这条路
已经不生效了，位姿流 (/model_states) 却照常 97 Hz —— 因为它们是两套插件。

判据：发 --v 指令 --sec 秒，用 10 Hz 采位姿，看平面位移。
  - 位移 ≈ v*sec（半车以上）→ cmd 通路正常，"不动"另有原因（被别的东西顶住/锁轮）；
  - 位移 ~0 → cmd 通路死了，得去 gzserver 侧查（插件没订上、模型被冻结、世界暂停）。
同时打印 cmd_vel 的发布者/订阅者数（订阅者=0 就是插件没订这个 topic，一目了然）。

用法：python3 cmd_vel_check.py [car_4] [0.70] [8]
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

CAR = sys.argv[1] if len(sys.argv) > 1 else 'car_4'
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.70
SEC = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

yaw_of = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Chk(Node):
    def __init__(self):
        super().__init__('cmd_vel_check')
        self.pose = None
        self.trace = []
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, 10)
        self.create_subscription(ModelStates, '/model_states', self.on, 20)

    def on(self, m):
        for n, p in zip(m.name, m.pose):
            if n == CAR:
                self.pose = (p.position.x, p.position.y, p.position.z,
                             yaw_of(p.orientation))
                self.trace.append((time.monotonic(), p.position.x, p.position.y,
                                   p.position.z, math.degrees(yaw_of(p.orientation))))

    def run(self, sec, v):
        self.trace = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            tw = Twist()
            tw.linear.x = v
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)


rclpy.init()
nd = Chk()
t0 = time.monotonic()
while nd.pose is None and time.monotonic() - t0 < 30:
    rclpy.spin_once(nd, timeout_sec=0.1)
if nd.pose is None:
    raise SystemExit('!! 30 s 内没收到 %s 的位姿' % CAR)
print('%s 起始 (%.2f,%.2f) z=%.2f yaw=%+.1f°' % (
    CAR, nd.pose[0], nd.pose[1], nd.pose[2], nd.pose[3]))

INFO = subprocess.run(['ros2', 'topic', 'info', '-v', '/%s/cmd_vel' % CAR],
                      capture_output=True, text=True).stdout
sub = [ln.strip() for ln in INFO.splitlines() if 'count:' in ln]
print('/%s/cmd_vel 计数（发布者/订阅者）：%s' % (CAR, ' '.join(sub)))
for ln in INFO.splitlines():
    if 'node_name' in ln or 'topic' in ln.lower() and 'type' not in ln:
        print('   ' + ln.strip()[:100])

nd.run(1.0, 0.0)                          # 先静置 1 s 攒基线
nd.run(SEC, V)
tr = nd.trace
if len(tr) < 5:
    print('!! 采样太少（%d 帧），位姿流本身有问题' % len(tr))
else:
    d = math.hypot(tr[-1][1] - tr[0][1], tr[-1][2] - tr[0][2])
    dmax = max(math.hypot(x - tr[0][1], y - tr[0][2]) for _, x, y, _, _ in tr)
    print('\n%.1f s 指令 %.2f m/s：末位移 %.2f m，最大位移 %.2f m，帧数 %d' % (
        SEC, V, d, dmax, len(tr)))
    print('   期望（cmd 通路正常）≈ %.1f m' % (V * SEC))
    print('   前 3 帧 / 后 3 帧：')
    for row in tr[:3] + tr[-3:]:
        print('     t=%+.2f (%7.2f,%7.2f) z=%.2f yaw=%+.1f°' % (
            row[0] - tr[0][0], row[1], row[2], row[3], row[4]))
    verdict = 'cmd_vel 通路正常' if dmax > V * SEC * 0.5 else '!! cmd_vel 没让车动'
    print('   判定：%s' % verdict)
nd.destroy_node()
rclpy.shutdown()
