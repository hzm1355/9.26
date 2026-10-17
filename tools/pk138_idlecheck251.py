#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""251：车在"没人指挥"的时候自己爬 —— 分清是驱动实例卡死还是被外力顶。

现在的事实（前两台测出来的）：
  * /car_2/cmd_vel 上 Publisher count=0、5 s 内零消息，可 car_2 还是以 ~0.1 m/s 往前爬，
    并且探针发的指令一点都改不了它的行为；
  * 同一轮里 /car_2/odom 的读数在四个不同指令的相位上一字不差（v=+0.128 ω=-0.000），
    这种"跨相位恒定"说明 odom 那一层要么死了、要么在报缓存 —— 不能当真相；
  * car_3 在 ω=0.63 下四个连杆全零，却在 ω=3.0 一脚被甩出 1.34 m（之后共跑飞 ~14 m）。
这三条指向不同修法，混在一起没法定案，所以这里一次把四层读齐：
  1) 静置 12 s，一个字节都不发：看它是否**自发**位移（自发=驱动或外力在自己动）；
  2) /link_states 里轮子的实测角速度：轮子在转 = 驱动输出了力矩；轮子不转而车在走 = 外力顶；
  3) /car_X/odom 的**发布频率**（不是数值）：0 Hz 说明 odom 链路已死，它报的 0.128 是遗照
     不是现状（memory「生效要用效果复核」：静止复检不算证据）；
  4) 主动给一发**反向**指令 v=-0.30：位移反向 = 指令通道活着；继续原向 = 卡死。
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates, ModelStates
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Watch(Node):
    def __init__(self, model):
        super().__init__('idlecheck_' + model)
        self.model = model
        self.pose = None
        self.wheels = {}
        self.odom_n = 0
        self.odom_last = None
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % model, 10)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)
        self.create_subscription(LinkStates, '/link_states', self.on_links, 10)
        self.create_subscription(Odometry, '/%s/odom' % model, self.on_odom, 10)

    def on_states(self, m):
        if self.model not in m.name:
            return
        i = m.name.index(self.model)
        p = m.pose[i].position
        self.pose = (p.x, p.y, yaw_of(m.pose[i].orientation))

    def on_links(self, m):
        for i, nm in enumerate(m.name):
            if '::' not in nm:
                continue
            model, link = nm.split('::', 1)
            if model != self.model or 'wheel' not in link:
                continue
            w = m.twist[i].angular
            # 轮子的自转轴在车体坐标系里是 +y；这里只取模，够用来判"转没转"。
            self.wheels[link] = math.hypot(w.x, w.y, w.z)

    def on_odom(self, m):
        self.odom_n += 1
        self.odom_last = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)


def window(node, dur, v=None, w=0.0, label=''):
    """在 dur 秒里观测（v=None 表示完全不发指令）。返回一句结论。"""
    p0 = node.pose
    if p0 is None:
        return '%-18s 位姿流没到位' % label, None
    n0 = node.odom_n
    wheel_peak = 0.0
    wheel_turned = set()
    end = time.monotonic() + dur
    while time.monotonic() < end:
        if v is not None:
            node.cmd(v, w)
        wheel_peak = max([wheel_peak] + list(node.wheels.values()))
        for k, val in node.wheels.items():
            if val > 0.05:
                wheel_turned.add(k)
        time.sleep(0.05)
    p1 = node.pose
    moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    dyaw = ((p1[2] - p0[2] + math.pi) % (2 * math.pi)) - math.pi
    hz = (node.odom_n - n0) / dur
    print('  %-20s 位移 %.3f m 转角 %+.3f rad｜轮子实测峰值角速度 %.2f rad/s（转过 %s）'
          '｜odom %.1f Hz 末值 (%+.3f,%+.3f)' % (
              label, moved, dyaw, wheel_peak, ','.join(sorted(wheel_turned)) or '无',
              hz, node.odom_last[0] if node.odom_last else float('nan'),
              node.odom_last[1] if node.odom_last else float('nan')), flush=True)
    return label, dict(moved=moved, dyaw=dyaw, wheel=wheel_peak, hz=hz,
                       turned=sorted(wheel_turned))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_2')
    args = ap.parse_args()

    rclpy.init()
    node = Watch(args.model)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 20:
        if node.pose and node.wheels:
            break
        time.sleep(0.2)
    else:
        print('!! 20 s 读不到 %s 的位姿/轮子连杆（模型名或 world 不对）' % args.model)
        return 2

    x, y, yaw = node.pose
    print('== %s 起始位姿 (%.2f, %.2f) yaw=%+.2f｜轮子连杆 %s' % (
        args.model, x, y, yaw, sorted(node.wheels)))
    print('== 1) 静置 12 s：一个字节都不发（自发位移 = 驱动或外力在自己动）')
    _, idle = window(node, 12.0, None, 0.0, '静置')
    print('== 2) 反向指令 v=-0.30 跑 6 s（指令通道活着的话位移要反向）')
    _, rev = window(node, 6.0, -0.30, 0.0, '反向 v=-0.30')
    print('== 3) 再静置 6 s 看余速（真卡死的实例会接着按缓存速度爬）')
    _, idle2 = window(node, 6.0, None, 0.0, '再静置')
    print('== 4) 正向 v=+0.30 跑 6 s 与 5) 原地转 ω=+0.63 跑 6 s')
    _, fwd = window(node, 6.0, 0.30, 0.0, '正向 v=+0.30')
    _, spin = window(node, 6.0, 0.0, 0.63, '原地转 ω=+0.63')

    print('== 定性')
    if idle and idle['moved'] > 0.05:
        if idle['wheel'] > 0.05:
            print('   静置时轮子在转 —— 驱动实例在自己输出（缓存轮速没被新指令覆盖）')
        else:
            print('   静置时轮子不转却位移 —— 被外力顶（接触挤出/别的模型推着它）')
    else:
        print('   静置不位移 —— 没有自发爬行')
    if rev and rev['moved'] > 0.1 and rev['dyaw'] * 0 >= 0:
        print('   反向指令产出了位移 %.2f m —— 指令通道是活的' % rev['moved'])
    if fwd:
        print('   正向 v=0.30 实测位移 %.2f m（6 s 理想 1.80 m）→ 实际增益 %.2f' % (
            fwd['moved'], fwd['moved'] / 1.80))
    if spin:
        print('   原地转 ω=0.63 实测转角 %+.3f rad（6 s 理想 3.78 rad）→ ω 增益 %.2f' % (
            spin['dyaw'], abs(spin['dyaw']) / 3.78))
    for _ in range(8):
        node.cmd(0.0, 0.0)
        time.sleep(0.05)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:                            # noqa: BLE001
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
