#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""248：car_3 收不到力还是被顶住？用轮子的**实测角速度**把这两态分开。

246 台的读数是一串 -0.000：指令发了、/car_3/odom 也在线，但 odom 的 twist 是零。
问题是 odom=0 有两种完全相反的解释（memory「不动的三态定性」「生效要用效果复核」）：
  b 轮子在转、车体被接触顶住 → 轮-地之间打滑，joint 速度非零；
  c 驱动实例卡死 → 轮子自己都不动，joint 速度为零。
两者的修法互斥：b 要改航线/净空，c 只有重 spawn 该模型能复位（同一份 world 里
car_4/car_5 都正常，所以不是全局问题）。/car_X/odom 分不开这两态，
/gazebo/link_states 里**轮子连杆**的 twist 才行 —— 它是物理积分的直接读数，
不经 diff_drive 的转发。

同时带两个对照：
  * 同窗口给 car_4 读一行 odom（它正被车队驱动着走），证明 odom 链路本身没坏；
  * 每个相位前后各取一次 car_3 的位姿，位移/转角用真值算，不看节点自报。
"""
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

PHASES = [
    ('直行 0.30', 0.30, 0.00, 6.0),
    ('原地转 +0.63', 0.00, +0.63, 6.0),
    ('原地转 +3.00（超上限试扭矩）', 0.00, +3.00, 6.0),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                         capture_output=True, text=True, timeout=20)
    line = (out.stdout + out.stderr).strip().splitlines()[0]
    v = [float(t) for t in line.split()[:6]]
    return v


class Watch(Node):
    def __init__(self):
        super().__init__('wheelcheck248')
        self.cmds = {}          # model -> (v, w)
        self.odom = {}          # model -> (v, w) 来自 diff_drive
        self.wheels = {}        # (model, link) -> (lin速度, 角速度模)
        self.pose = {}
        self.pub = {}
        self.n_links = 0
        for m in ['car_3', 'car_4']:
            self.pub[m] = self.create_publisher(Twist, '/%s/cmd_vel' % m, 10)
            self.create_subscription(Odometry, '/%s/odom' % m,
                                     lambda mm, k=m: self.on_odom(k, mm), 10)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)
        # 话题名是 /link_states 而不是 /gazebo/link_states —— 这个 world 的位姿接口把
        # 状态发在全局命名空间（shuttle 订的也是 /model_states）。按 /gazebo/ 前缀订会
        # 一帧都收不到，然后读成"轮子不转"，其实是订错了名字（实测踩过）。
        from gazebo_msgs.msg import LinkStates
        self.has_links = True
        self.create_subscription(LinkStates, '/link_states', self.on_links, 10)

    def on_odom(self, model, m):
        tw = m.twist.twist
        self.odom[model] = (tw.linear.x, tw.angular.z)

    def on_states(self, m):
        for want in ('car_3', 'car_4'):
            if want in m.name:
                i = m.name.index(want)
                p = m.pose[i].position
                self.pose[want] = (p.x, p.y, yaw_of(m.pose[i].orientation))

    def on_links(self, m):
        self.n_links = len(m.name)
        for i, nm in enumerate(m.name):
            # 连杆全名形如 car_3::left_wheel
            if '::' not in nm:
                continue
            model, link = nm.split('::', 1)
            if model != 'car_3':
                continue
            if not ('wheel' in link or 'chassis' in link or 'base_link' in link
                    or 'caster' in link):
                continue
            v = m.twist[i].linear
            w = m.twist[i].angular
            self.wheels[(model, link)] = (math.hypot(v.x, v.y, v.z),
                                          math.hypot(w.x, w.y, w.z))


def finish(node):
    """收尾：零速钉住轮位目标，再关上下文。
    直接让解释器退出会在 rclpy 的守护线程上留一个 active exception（实测 core dump），
    日志读到"dumped core"像整个测试没跑完，其实数据都打出来了。"""
    try:
        for _ in range(6):
            node.pub['car_3'].publish(Twist())
            time.sleep(0.05)
    except Exception:                            # noqa: BLE001
        pass
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:                            # noqa: BLE001
        pass


def main():
    rclpy.init()
    node = Watch()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 20:
        if node.pose and (node.wheels or not node.has_links) and node.odom:
            break
        time.sleep(0.2)
    print('== link_states 订阅：%s（拿到 %d 个连杆）；位姿源 /model_states 有 %s' % (
        node.has_links, node.n_links, sorted(node.pose)))
    if node.has_links and node.n_links == 0:
        print('!! /link_states 一帧都没有 —— 判不了轮子，先别下结论')
        finish(node)
        return 2

    for label, v, w, dur in PHASES:
        p0 = pose('car_3')
        od0 = dict(node.odom)
        wheel_keys = sorted({k[1] for k in node.wheels}) if node.wheels else []
        node.wheels.clear()
        end = time.monotonic() + dur
        peak = {}
        while time.monotonic() < end:
            t = Twist()
            t.linear.x = v
            t.angular.z = w
            node.pub['car_3'].publish(t)
            for k, cur in node.wheels.items():
                key = k[1]
                prev = peak.get(key, (0.0, 0.0))
                peak[key] = (max(prev[0], cur[0]), max(prev[1], cur[1]))
            time.sleep(0.05)
        for _ in range(6):
            node.pub['car_3'].publish(Twist())
            time.sleep(0.05)
        p1 = pose('car_3')
        moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        turned = abs(((p1[5] - p0[5] + math.pi) % (2 * math.pi)) - math.pi)
        od = node.odom.get('car_3', (float('nan'),) * 2)
        ctl = node.odom.get('car_4', (float('nan'),) * 2)
        print('  %-26s 指令 v=%+.2f ω=%+.2f | car_3 odom=(%+.3f,%+.3f) | 实测 位移 %.3f m 转角 %+.3f rad'
              % (label, v, w, od[0], od[1], moved, turned))
        if peak:
            for key in sorted(peak):
                lv, av = peak[key]
                print('        轮/体连杆 %-18s 峰值 线速度 %.3f m/s  角速度 %.3f rad/s' % (key, lv, av))
        else:
            print('        （这一相没抓到 wheel 连杆的 twist；已知连杆：%s）' % wheel_keys)
        print('        对照组 car_4 odom=(%+.3f,%+.3f) ← 同窗口它若非零，链路就是好的' % ctl)

    finish(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())
