#!/usr/bin/env python3
"""给 car_6 下 ω 指令的同时看 /link_states 里每个 link 有没有转 —— 判"轮子在不在转"。

为什么需要这一步：到上一版为止三条证据都指不到根因——
  1) cmd_vel 订阅者数 = 1，指令确实送到了；
  2) 5 s × ω=0.35 应该转 1.75 rad，实测 -0.018 rad；5 s × v=0.12 应该走 0.6 m，
     实测 0.005 m，响应约 1%；
  3) /car_6/odom 的增量和 /model_states 的底盘增量**逐段完全相同**，所以 odom 在这
     个插件里就是底盘位姿的转发，拿它当"轮速"看是错的（上一版以为能区分，实测不能）。
剩下的唯一分叉是：执行端（关节）根本没动，还是关节在动、底盘被物理顶住。
libgazebo_ros_state 发布的 /link_states 是逐 link 的世界位姿，轮子绕自己的轴转会直接
体现在该 link 的四元数上，而底盘不动 —— 这一条能干净地分开两种成因。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates

PREFIX = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
BURST_S = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
OMEGA = float(sys.argv[3]) if len(sys.argv) > 3 else 0.35


def quat_angle(a, b):
    # 两个四元数之间的转角：2*acos(|dot|)，不用管它是绕哪根轴
    d = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return 2.0 * math.acos(min(1.0, max(-1.0, d)))


class P:
    def __init__(self):
        rclpy.init()
        self.nd = Node('link_probe153')
        self.q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.cur = {}
        self.frames = 0
        self.nd.create_subscription(LinkStates, '/link_states', self.on_ls, self.q)
        self.pub = self.nd.create_publisher(
            Twist, '/' + PREFIX + '/cmd_vel', QoSProfile(depth=10))

    def on_ls(self, m):
        self.frames += 1
        for i, n in enumerate(m.name):
            if n.startswith(PREFIX):
                o = m.pose[i].orientation
                p = m.pose[i].position
                self.cur[n] = ((o.w, o.x, o.y, o.z), (p.x, p.y))

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.nd, timeout_sec=0.02)


def main():
    p = P()
    p.spin(3.0)
    if not p.cur:
        print('FAIL /link_states 3 s 内没有 %s 的 link（插件没发布？话题名不对？）' % PREFIX)
        return 1
    nsub = p.pub.get_subscription_count()
    print('  %s 的 link 数 = %d   cmd_vel 订阅者 = %d' % (PREFIX, len(p.cur), nsub))
    if nsub == 0:
        print('FAIL 指令送不到，先解决通道再判')
        return 1

    before = dict(p.cur)
    t0 = time.time()
    while time.time() - t0 < BURST_S:
        m = Twist()
        m.angular.z = OMEGA
        p.pub.publish(m)
        rclpy.spin_once(p.nd, timeout_sec=0.02)
    for _ in range(40):
        p.pub.publish(Twist())
        rclpy.spin_once(p.nd, timeout_sec=0.0)
    p.spin(1.0)

    print('  发 ω=%+.2f 共 %.1f s（应转 %.2f rad），随后连发零速' % (
        OMEGA, BURST_S, OMEGA * BURST_S))
    rows = []
    for n in sorted(before):
        if n not in p.cur:
            rows.append((n, float('nan'), float('nan')))
            continue
        da = math.degrees(quat_angle(before[n][0], p.cur[n][0]))
        dp = math.hypot(p.cur[n][1][0] - before[n][1][0],
                        p.cur[n][1][1] - before[n][1][1])
        rows.append((n, da, dp))
    for n, da, dp in rows:
        tag = ''
        if da == da and da > 5.0:
            tag = '<== 在转'
        print('    %-34s 姿态变化 %7.2f°  位移 %.4f m  %s' % (n, da, dp, tag))
    print('  /link_states 帧数 %d' % p.frames)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
