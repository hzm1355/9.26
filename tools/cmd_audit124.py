#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#124 —— 干净栈上的 car_6 指令审计：只发指令，绝不摆位。

为什么不能摆位：#122 实测每次 SetEntityState 都带 2.3~19.9 m/s 弹射，而且 gz 报的
**轮角绝对值会跳**（右轮同一次运行 1434.935 → 29.094）。所以本脚本只在"重启后车本来
就在 world 原始位 (12.0,-12.8)"的前提下，取短窗口内的轮角增量做判据。

七段，回答四个问题：
  A 静默 5 s   —— 干净栈上没人碰车（路径应 ≈0，负基线）
  B 零速 3 s   —— 显式零速下的静止
  C 纯ω右 3 s  —— ω 通道的车体兑现率（#85 正题）
  D 静默 5 s   —— **cmd_vel_timeout 到底生不生效**：刚收过 ω 指令就彻底停发，
                  轮子继续转 = 静默不刹车（#120 现象）；轮子停 = timeout 有效
  E 纯v 3 s    —— v 通道兑现率
  F 静默 4 s   —— 同 D，换 v 指令再验一次
  G 零速 3 s   —— 收尾把目标清零，绝不让车带着最后一条指令离开
"""
import math
import sys
import time

sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools')
from cmd_pivot102 import BEST, SEP, RADIUS, wrap, yaw_of   # noqa: E402
from pose_full121 import gz_read                           # noqa: E402
import rclpy                                               # noqa: E402
from rclpy.node import Node                                 # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy         # noqa: E402
from geometry_msgs.msg import Twist                         # noqa: E402
from rosgraph_msgs.msg import Clock                         # noqa: E402
from gazebo_msgs.msg import ModelStates                     # noqa: E402

CAR = 'car_6'
RATE_HZ = 20.0


class Hub(Node):
    def __init__(self):
        super().__init__('cmd_audit124')
        self.clock = 0.0
        self.pose = None
        self.prev = None
        self.yaw_acc = 0.0
        self.path_acc = 0.0
        self.n_ms = 0
        self.n_cmd = 0
        self.sent = (0.0, 0.0)
        self.create_subscription(Clock, '/clock', self.on_c, BEST)
        self.create_subscription(ModelStates, '/model_states', self.on_s, BEST)
        self.create_subscription(Twist, '/%s/cmd_vel' % CAR, self.on_cmd, BEST)
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % CAR, rel)

    def on_c(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cmd(self, m):
        self.n_cmd += 1

    def on_s(self, m):
        if CAR not in m.name:
            return
        self.n_ms += 1
        i = m.name.index(CAR)
        p = m.pose[i].position
        cur = (self.clock, p.x, p.y, p.z, yaw_of(m.pose[i].orientation))
        if self.prev is None:
            self.prev = cur
        elif cur[0] - self.prev[0] > 1e-9:
            self.yaw_acc += wrap(cur[4] - self.prev[4])
            self.path_acc += math.hypot(cur[1] - self.prev[1], cur[2] - self.prev[2])
            self.prev = cur
        self.pose = cur

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def send(self, v, w):
        self.sent = (v, w)
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)

    def wait_pose(self, secs=25.0):
        end = time.time() + secs
        while self.pose is None and time.time() < end:
            self.tick()
            time.sleep(0.02)
        return self.pose

    def run(self, secs, publish, v, w, tag):
        """publish=False 时一条都不发 —— 这才是"静默"该有的样子。"""
        end = time.time() + secs
        last = 0.0
        ya, pa = self.yaw_acc, self.path_acc
        n = 0
        while time.time() < end:
            if publish:
                self.send(v, w)
                n += 1
            self.tick()
            time.sleep(1.0 / RATE_HZ)
            self.tick()
            if time.time() - last >= 1.0 and self.pose:
                last = time.time()
                print('    %-10s t=%9.2f (%7.3f,%7.3f) z=%6.4f 本秒 ω=%+.3f |v|=%+.3f 发%d' % (
                    tag, self.pose[0], self.pose[1], self.pose[2], self.pose[3],
                    self.yaw_acc - ya, self.path_acc - pa, n), flush=True)
                ya, pa = self.yaw_acc, self.path_acc
        return n


def body(h, tag):
    g = gz_read(CAR)
    c = h.clock
    t0 = time.time()
    while h.clock == c and time.time() - t0 < 3.0:
        h.tick()
        time.sleep(0.01)
    print('  %-10s 轮角 左%9.3f 右%9.3f 位(%7.3f,%7.3f) z=%6.4f '
          'roll=%+6.1f° pitch=%+6.1f° yaw=%+.4f 钟%9.2f 累计yaw%+8.3f 累计路%7.3f' % (
              tag, g[8], g[9], g[0], g[1], g[2], g[6], g[7], g[5], c,
              h.yaw_acc, h.path_acc), flush=True)
    return g + (c, h.yaw_acc, h.path_acc)


def seg(h, tag, publish, v, w, secs):
    print('\n--- %s  %s v=%+.2f w=%+.2f（%.1f s）---' % (
        tag, '连发' if publish else '静默停发', v, w, secs), flush=True)
    pre = body(h, '段前')
    sent = h.run(secs, publish, v, w, tag)
    post = body(h, '段后')
    dur = post[10] - pre[10]
    if dur <= 0:
        print('  !! 窗口为 0')
        return
    dl, dr = post[8] - pre[8], post[9] - pre[9]
    dyaw, dpath = post[11] - pre[11], post[12] - pre[12]
    wl, wr_ = dl / dur, dr / dur
    wheel_w = (wr_ - wl) * RADIUS / SEP
    body_w = dyaw / dur
    pl = (v - w * SEP / 2.0) / RADIUS * dur
    pr = (v + w * SEP / 2.0) / RADIUS * dur
    print('  发出 %d 条 / 段内到达 %d 条   窗口 %.2f s' % (sent, h.n_cmd, dur))
    print('  轮速 左%+7.3f 右%+7.3f rad/s（预测 %+.3f / %+.3f）  轮差反推 ω=%+.3f' % (
        wl, wr_, pl / dur, pr / dur, wheel_w))
    print('  车体真值 ω=%+.3f rad/s 兑现 %s   |v|=%+.3f m/s 路径 %.3f m' % (
        body_w, '%.2f' % (body_w / w) if abs(w) > 1e-6 else '—',
        dpath / dur, dpath))
    if publish and abs(w) > 1e-6:
        print('  >> ω 通道兑现率 %.0f%%（车体）/ %.0f%%（轮差）' % (
            100.0 * body_w / w, 100.0 * wheel_w / w))
    if not publish:
        still = abs(wl) > 0.5 or abs(wr_) > 0.5 or abs(body_w) > 0.05
        print('  >> 静默判读：%s' % (
            '轮子仍在动 ⇒ cmd_vel_timeout 不生效，静默不等于刹车'
            if still else '轮子已停 ⇒ 这一发 timeout 起作用了'))


def main():
    rclpy.init()
    h = Hub()
    if h.wait_pose() is None:
        print('!! 没拿到 /model_states')
        return 1
    print('=== 起点体检（不摆位）===')
    s = body(h, '起点')
    print('  离 world 原始位 (12.000,-12.800) %.3f m  roll=%+.1f° pitch=%+.1f° z=%6.4f' % (
        math.hypot(s[0] - 12.0, s[1] + 12.8), s[6], s[7], s[2]))
    for spec in [('A 静默基线', False, 0.0, 0.0, 5.0),
                 ('B 零速', True, 0.0, 0.0, 3.0),
                 ('C 纯ω右', True, 0.0, -0.35, 3.0),
                 ('D 静默验timeout', False, 0.0, 0.0, 5.0),
                 ('E 纯v直行', True, 0.31, 0.0, 3.0),
                 ('F 静默验timeout', False, 0.0, 0.0, 4.0),
                 ('G 收尾零速', True, 0.0, 0.0, 3.0)]:
        seg(h, *spec)
        h.n_cmd = 0
    print('\n=== 终点体检 ===')
    body(h, '终点')
    print('全程 model_states %d 帧' % h.n_ms)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
