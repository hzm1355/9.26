#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量车队**逐帧位移**的分布，用来定"复位/瞬移"护栏该放在多少米。

测速节点现在只在窗口弦长 > 5 m 时判瞬移，但瞬时读数里仍冒出 20.08 / 44.77 km/h：
说明小于 5 m 的跳变在 0.3 s 窗口里同样能被算成几十 km/h（3 m 跳变 = 36 km/h）。
本脚本只订阅 /model_states，按墙钟算每帧速度，打出：
 * 每帧速度的分位数（对照巡航 0.7 m/s = 2.52 km/h）；
 * 逐帧位移超过 0.3/0.5/1/2/5 m 的帧数——这些就是护栏该拦的候选阈值；
 * 速度最大的若干帧，附车辆与位置，好核对是不是复位点附近。
用法：python3 speed_diag_jumps.py [窗口秒]
"""
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.msg import ModelStates

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
PREFIX = 'car_'


class Rec:
    def __init__(self):
        self.w = []
        self.p = []
        self.big = []      # 逐帧位移超阈值的样本


rec = {}
rclpy.init()
node = Node('jump_diag', parameter_overrides=[Parameter('use_sim_time', value=True)])


def on_states(msg):
    t = time.time()
    for i, name in enumerate(msg.name):
        if not name.startswith(PREFIX):
            continue
        r = rec.setdefault(name, Rec())
        p = msg.pose[i].position
        r.w.append(t)
        r.p.append((p.x, p.y, p.z))


node.create_subscription(ModelStates, '/model_states', on_states, 50)
t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()

span = time.time() - t0
print('窗口 %.1f s，车辆 %d 辆' % (span, len(rec)))
allspeed = []
tot_frames = 0
for th in (0.3, 0.5, 1.0, 2.0, 5.0):
    print('  逐帧位移 > %.1f m 的帧数：' % th, end='')
    for name in sorted(rec):
        r = rec[name]
        n = 0
        for a, b in zip(r.p, r.p[1:]):
            if math.hypot(b[0] - a[0], b[1] - a[1]) > th:
                n += 1
        print(' %s=%d' % (name, n), end='')
    print()

rows = []
for name in sorted(rec):
    r = rec[name]
    sp, dpl = [], []
    for i in range(1, len(r.w)):
        dt = r.w[i] - r.w[i - 1]
        if dt <= 1e-6:
            continue
        d = math.hypot(r.p[i][0] - r.p[i - 1][0], r.p[i][1] - r.p[i - 1][1])
        dz = abs(r.p[i][2] - r.p[i - 1][2])
        sp.append(3.6 * d / dt)
        dpl.append((d, dz, dt))
        rows.append((3.6 * d / dt, name, r.p[i][0], r.p[i][1], d, dz, dt))
    if not sp:
        print('%s 无有效帧' % name)
        continue
    sp.sort()
    print('%-7s 帧 %d  速度中位 %.2f  p95 %.2f  最大 %.2f km/h  单帧位移中位 %.3f m  最大 %.3f m(z %.3f)' % (
        name, len(sp), statistics.median(sp), sp[int(.95 * (len(sp) - 1))], sp[-1],
        statistics.median([d for d, _, _ in dpl]), max(d for d, _, _ in dpl),
        max(z for _, z, _ in dpl)))
    allspeed += sp

if allspeed:
    allspeed.sort()
    print('全部 %d 帧：中位 %.2f  p99 %.2f  最大 %.2f km/h；>6.0 km/h 的帧 %d，>15 km/h 的帧 %d' % (
        len(allspeed), statistics.median(allspeed), allspeed[int(.99 * (len(allspeed) - 1))],
        allspeed[-1], sum(1 for v in allspeed if v > 6.0), sum(1 for v in allspeed if v > 15.0)))
    print('-- 速度最大的 12 帧（车 / 位置 / 位移 / dt）--')
    for v, name, x, y, d, dz, dt in sorted(rows, reverse=True)[:12]:
        print('  %6.2f km/h  %-7s (%.2f,%.2f) 位移 %.3f m (z %.3f) dt %.4f s' % (
            v, name, x, y, d, dz, dt))
