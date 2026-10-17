#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分三段问清楚一件事：**摆位后静止的车，还吃不吃 cmd_vel（轮子还有没有附着力）**。

为什么要分三段：上一条测试里 car_2 停在 (-2.17,9.70) —— 那是沙盘北路（y=4.6）以北 5 m，
不在车道上，可能正顶在建筑/路肩上。"发指令不动"在这种状态下是**正常现象**，不能当结论。
而 place_z_probe 又显示每次摆位都会沿车头滑掉 3~20 m，所以也不能"摆完立刻发指令"。
于是按时间切成三段，各自独立判定：

  1) 摆位段：放到车道点，记录滑移（这是穿透回弹，跟附着力无关）；
  2) 静置段：不发任何指令，等滑移自己停（附着力存在的话几秒内就停）；
  3) 指令段：只发 /car_N/cmd_vel 定速，记录位移。

判读：
  - 第 2 段能停住 + 第 3 段不动 = 轮子空转（附着力没了：轮胎与路面的摩擦耦合失效）；
  - 第 3 段按 v*t 前进 = cmd 通路正常，之前"卡死 18 s"是因为落点本身顶住了；
  - 第 2 段一直滑 = 附着力已经没了（滑不动才需要摩擦）。

用法：python3 traction_test.py --car car_4 --x 8.0 --y -14.6 --yaw 0 \
        --z 7.10 --settle 12 --drive 10 --v 0.70
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState


def yaw_rad(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self):
        super().__init__('traction_test')
        self.pose = None
        self.trace = []
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, 10)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(ModelStates, '/model_states', self.on, 20)

    def on(self, m):
        for n, p in zip(m.name, m.pose):
            if n == CAR:
                self.pose = (p.position.x, p.position.y, p.position.z, yaw_rad(p.orientation))
                self.trace.append((time.monotonic(), p.position.x, p.position.y,
                                   p.position.z, yaw_rad(p.orientation)))

    def place(self, x, y, z, yaw):
        r = SetEntityState.Request()
        r.state.name = CAR
        r.state.pose.position.x, r.state.pose.position.y = x, y
        r.state.pose.position.z = z
        r.state.pose.orientation.z = math.sin(yaw / 2.0)
        r.state.pose.orientation.w = math.cos(yaw / 2.0)
        r.state.twist.linear.x = 0.0
        r.state.twist.angular.z = 0.0
        r.state.reference_frame = 'world'
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit('!! /set_entity_state 没服务')
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=20.0)
        return bool(f.result() and f.result().success)

    def segment(self, sec, v):
        """返回 (首末位移, 最大位移, 每秒位移列表, 帧数, z 首末)。"""
        self.trace = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            tw = Twist()
            tw.linear.x = v
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
        tr = self.trace
        if len(tr) < 5:
            return None
        per, prev = [], tr[0]
        for row in tr[1:]:
            if row[0] - prev[0] >= 1.0:
                per.append(round(math.hypot(row[1] - prev[1], row[2] - prev[2]), 2))
                prev = row
        d = math.hypot(tr[-1][1] - tr[0][1], tr[-1][2] - tr[0][2])
        dmax = max(math.hypot(x - tr[0][1], y - tr[0][2]) for _, x, y, _, _ in tr)
        return d, dmax, per, len(tr), tr[0][3], tr[-1][3], math.degrees(tr[-1][4])


ap = argparse.ArgumentParser()
ap.add_argument('--car', default='car_4')
ap.add_argument('--x', type=float, default=8.0)
ap.add_argument('--y', type=float, default=-14.6)
ap.add_argument('--z', type=float, default=7.10)
ap.add_argument('--yaw', type=float, default=0.0)
ap.add_argument('--settle', type=float, default=12.0)
ap.add_argument('--drive', type=float, default=10.0)
ap.add_argument('--v', type=float, default=0.70)
a = ap.parse_args()
CAR = a.car

rclpy.init()
nd = T()
t0 = time.monotonic()
while nd.pose is None and time.monotonic() - t0 < 30:
    rclpy.spin_once(nd, timeout_sec=0.1)
print('%s 接手位姿 (%.2f,%.2f) z=%.2f' % (CAR, nd.pose[0], nd.pose[1], nd.pose[2]))

ok = nd.place(a.x, a.y, a.z, a.yaw)
s1 = nd.segment(4.0, 0.0)
print('\n1) 摆位段（回执 %s，不发指令）：末位移 %.2f m / 最大 %.2f m，每秒 %s' % (
    ok, s1[0], s1[1], s1[2]))

s2 = nd.segment(a.settle, 0.0)
print('2) 静置段（%.0f s，全程零速）：末位移 %.2f m / 最大 %.2f m，每秒 %s' % (
    a.settle, s2[0], s2[1], s2[2]))

s3 = nd.segment(a.drive, a.v)
print('3) 指令段（%.0f s，%.2f m/s）：末位移 %.2f m / 最大 %.2f m，每秒 %s  期望 %.1f m' % (
    a.drive, a.v, s3[0], s3[1], s3[2], a.v * a.drive))
print('   段末 z=%.2f（摆位时 %.2f）航向 %+6.1f°（摆入 %.1f°）' % (
    s3[5], a.z, s3[6], math.degrees(a.yaw)))

moved = s3[1] > a.v * a.drive * 0.4
still = s2[1] < 0.8
print('\n判定：cmd %s / 静置段 %s' % (
    '能动（%.1f m）' % s3[1] if moved else '!! 不动',
    '停住了' if still else '!! 一直在滑'))
if moved and still:
    print('  → cmd 通路与附着力都在：之前"卡死"是落点顶住/被复位污染')
elif (not moved) and still:
    print('  → 车能停住却推不动 = 轮子被卡住或插件没把指令送到关节')
elif not still:
    print('  → 停不住 = 轮胎与路面的摩擦耦合失效（附着力没了），推不动也是同一个原因')
nd.pub.publish(Twist())
nd.destroy_node()
rclpy.shutdown()
