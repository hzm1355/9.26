#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#85 幽灵驱动取证：**本脚本一个字节都不发**，所以 /car_6/cmd_vel 上收到的任何一条消息
都是外来发布者；再配合每秒的位姿与每 2 s 的 gz 轮角，就能把"车自己在动"归因到
  (i) 有人在发 cmd_vel  -> 消息条数 > 0，且与运动同步
  (ii) gzserver 内部    -> 消息条数 = 0，但轮角/位姿仍在变
  (iii) 什么都不动      -> 那一轮没抓到 episode，得择时重跑

运动量一律用逐帧 wrap 累加的展开 yaw 与路径（锚在 /clock 跳变那帧），
避免 3 s 端点差把 2 rad/s 的连续自转混叠成 0.3 rad/s。
"""
import argparse
import math
import re
import subprocess
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import BEST, SKIP, wrap, yaw_of   # noqa: E402
import rclpy                                        # noqa: E402
from rclpy.node import Node                          # noqa: E402
from geometry_msgs.msg import Twist                  # noqa: E402
from rosgraph_msgs.msg import Clock                  # noqa: E402
from gazebo_msgs.msg import ModelStates              # noqa: E402
from nav_msgs.msg import Odometry                    # noqa: E402

NUM = r'[-+0-9.eE]+'


class Ghost(Node):
    def __init__(self, car):
        super().__init__('ghost_drive118')
        self.car = car
        self.clock = 0.0
        self.pose = None            # (t, x, y, z, yaw)
        self.prev = None
        self.yaw_acc = 0.0
        self.path_acc = 0.0
        self.near = ('-', 1e9)
        self.msgs = []              # (t, v, w) 本脚本不发布，收到即外来
        self.n_ms = 0
        self.odom_tw = None
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        self.create_subscription(Twist, '/%s/cmd_vel' % car, self.on_cmd, BEST)
        self.create_subscription(Odometry, '/%s/odom' % car, self.on_odom, 10)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.msgs.append((round(self.clock, 2), round(m.linear.x, 3), round(m.angular.z, 3)))

    def on_odom(self, m):
        t = m.twist.twist
        self.odom_tw = (round(self.clock, 2), round(t.linear.x, 3), round(t.angular.z, 3))

    def on_s(self, m):
        if self.car not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(self.car)
        p = m.pose[i].position
        cur = (self.clock, p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
        best = ('-', 1e9)
        for j, nm in enumerate(m.name):
            if nm == self.car or nm in SKIP:
                continue
            q = m.pose[j].position
            d = math.hypot(q.x - cur[1], q.y - cur[2])
            if d < best[1]:
                best = (nm, d)
        self.near = best
        if self.prev is None:
            self.prev = cur
        elif cur[0] - self.prev[0] > 1e-9:
            self.yaw_acc += wrap(cur[4] - self.prev[4])
            self.path_acc += math.hypot(cur[1] - self.prev[1], cur[2] - self.prev[2])
            self.prev = cur
        self.pose = cur


def gz_wheels(car):
    p = subprocess.run(['gz', 'model', '-m', car, '-i'],
                       capture_output=True, text=True, timeout=25)
    out = []
    for j in ('left_wheel_joint', 'right_wheel_joint'):
        m = re.search(r'name: "%s::%s".{0,400}?angle: (%s)' % (car, j, NUM), p.stdout, re.S)
        out.append(float(m.group(1)) if m else float('nan'))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--secs', type=float, default=90.0)
    a = ap.parse_args()

    rclpy.init()
    n = Ghost(a.model)
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.02)

    print('# 被动取证开始：本节点不发布任何东西。目标 %s，%d s' % (a.model, int(a.secs)))
    last_print, last_wheel, last_stat = 0.0, 0.0, 0.0
    ya, pa, nm0 = n.yaw_acc, n.path_acc, len(n.msgs)
    w0 = gz_wheels(a.model)
    t_start = n.clock
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.time()
        if now - last_print >= 1.0 and n.pose:
            last_print = now
            dt = n.pose[0] - last_stat if last_stat else 1.0
            print('  t=%9.2f (%6.2f,%6.2f) z=%6.4f yaw=%+.3f | 本秒 ω=%+.3f |v|=%+.3f | '
                  '最近=%s@%5.2f m | 外来cmd=%d odom_tw=%s' % (
                      n.pose[0], n.pose[1], n.pose[2], n.pose[3], n.pose[4],
                      (n.yaw_acc - ya) / dt, (n.path_acc - pa) / dt,
                      n.near[0], n.near[1], len(n.msgs) - nm0, n.odom_tw))
            ya, pa = n.yaw_acc, n.path_acc
            last_stat = n.pose[0]
        if now - last_wheel >= 2.0:
            last_wheel = now
            w = gz_wheels(a.model)
            print('    轮角 左%9.2f 右%9.2f  （自上次 Δ左 %+.2f Δ右 %+.2f）' % (
                w[0], w[1], w[0] - w0[0], w[1] - w0[1]))
            w0 = w
    span = n.clock - t_start
    print('\n# 结论行：%.1f s 仿真时长内收到 model_states %d 帧、外来 cmd_vel %d 条、'
          '展开转角 %+.2f rad、路径 %.2f m' % (span, n.n_ms, len(n.msgs), n.yaw_acc, n.path_acc))
    if n.msgs:
        print('# 外来 cmd_vel 前 30 条 (仿真钟, v, w)：%s' % (n.msgs[:30],))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
