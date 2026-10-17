#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断 /model_states 的采样节奏，并**按检测节点的同一套算法**复算一遍瞬时速度。

瞬时测速读数比区间均速低一个数量级（0.25 vs 2.52），嫌疑只在 Δt 与 Δp 是不是同一把尺：
 * Δp 随 /model_states 来（实测频率这里直接量，不靠 config 里的 rate_hz 猜）；
 * Δt 随 node.get_clock() 来（use_sim_time 下是 /clock 量化过的仿真时钟）。
两者频率不一致时，"跳过 dt<=0 的帧"并不能让 |Δp|/Δt 等于真速度——
时钟每跳一格期间到位姿已经走了 N 帧，被采纳的那一帧只带 1 帧的位移、却带整格的 Δt。
本脚本把这两种口径分开打出来，直接看缩放倍数。
"""
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

WINDOW = 20.0
TARGET = __import__('sys').argv[1] if len(__import__('sys').argv) > 1 else 'car_3'


class Rec:
    def __init__(self):
        self.clock_msgs = 0
        self.clock_span = [0.0, 0.0]
        self.node_now = []        # 节点时钟（与检测节点同源）
        self.wall = []
        self.pos = []


r = Rec()
rclpy.init()
node = Node('speed_diag', parameter_overrides=[Parameter('use_sim_time', value=True)])


def on_clock(m):
    t = m.clock.sec + m.clock.nanosec * 1e-9
    r.clock_msgs += 1
    if not r.clock_span[0]:
        r.clock_span[0] = t
    r.clock_span[1] = t


def on_states(m):
    if TARGET not in m.name:
        return
    i = m.name.index(TARGET)
    p = m.pose[i].position
    r.wall.append(time.time())
    r.node_now.append(node.get_clock().now().nanoseconds / 1e9)
    r.pos.append((p.x, p.y))


best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(Clock, '/clock', on_clock, best)
node.create_subscription(ModelStates, '/model_states', on_states, 50)

t0 = time.time()
while time.time() - t0 < WINDOW:
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()

W, NN, P = r.wall, r.node_now, r.pos
n = len(W)
if n < 5:
    print('只收到 %d 帧（%s 不在模型列表里？）' % (n, TARGET))
    raise SystemExit(1)
span = W[-1] - W[0]
dw = [b - a for a, b in zip(W, W[1:])]
dp = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(P, P[1:])]
print('%s：/model_states %d 帧 / %.1f s → %.1f Hz；墙钟间隔中位 %.4f s' % (
    TARGET, n, span, (n - 1) / span, statistics.median(dw)))
print('/clock：%d 条，仿真秒推进 %.2f s（墙钟 %.2f s）→ 倍率 %.3f，即约 %.1f Hz' % (
    r.clock_msgs, r.clock_span[1] - r.clock_span[0], span,
    (r.clock_span[1] - r.clock_span[0]) / span if span else 0,
    r.clock_msgs / span if span else 0))

# 两种 Δt 口径都算一遍：节点时钟（=改造前检测节点的算法）、墙钟。
# /model_states 是 gazebo_msgs/ModelStates，本身没有 header，所以消息时间戳这条无从取。
def speed(times):
    acc, dz, zero = [], [], 0
    for i in range(1, n):
        dt = times[i] - times[i - 1]
        if dt <= 1e-6:
            zero += 1
            continue
        if dt > 2.0:
            continue
        acc.append(3.6 * dp[i - 1] / dt)
        dz.append(dp[i - 1])
    return acc, dz, zero


for label, times in (('节点时钟', NN), ('墙钟', W)):
    acc, dz, zero = speed(times)
    dts = sorted(b - a for a, b in zip(times, times[1:]))
    print('%-10s Δt：中位 %.4f s（=0 的帧 %d/%d）→ |Δp|/Δt 中位 %.2f km/h  均值 %.2f  n=%d' % (
        label, dts[len(dts) // 2] if dts else 0, zero, n - 1,
        statistics.median(acc) if acc else 0.0,
        statistics.fmean(acc) if acc else 0.0, len(acc)))
print('单帧位移中位 %.4f m' % statistics.median(dp))
# 整段位移/整段时间：不受逐帧量化影响的基准
tot = sum(dp)
print('整段路程 %.2f m / 墙钟 %.2f s = %.2f km/h（基准）' % (
    tot, span, 3.6 * tot / span if span else 0))
