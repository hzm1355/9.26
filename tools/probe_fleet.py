#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车队现场只读探针：一次跑完就把"谁在发指令 / 车实际多快 / 仿真几倍速"三件事说清。

为什么要这一份：审计里出现 |v| 1.0~1.5 m/s 的样本，而 signal_drive 的指令上限只有
cruise 0.7、倒车 -0.22。要么还有第二个发布者抢发 cmd_vel（僵尸实例），要么车是被
物理弹开/在滑，要么 t 轴用的不是仿真时钟导致读数错位。这三种处置完全不同。

用法（WSL，source 过 ROS 与 ~/v2x_ws/install/setup.bash 之后）：
  python3 tools/probe_fleet.py [秒数]
"""
import math
import sys
import time
from collections import defaultdict

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5']
# gazebo_ros 的 /clock 与 /model_states 是 best_effort 发布，默认 reliable 订阅会
# 静默收到 0 条消息（只在 /clock 上给一条 QoS 警告），探针会误判成"车没在跑"。
QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


class Probe(Node):
    def __init__(self):
        super().__init__('probe_fleet', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cmd = {c: [] for c in CARS}
        self.v = defaultdict(list)
        self.yaw_rate = defaultdict(list)
        self.clock = None
        self.wall0 = time.time()
        self.sim0 = None
        self.sim_n = 0
        for c in CARS:
            self.create_subscription(Twist, f'/{c}/cmd_vel',
                                     lambda m, c=c: self.cmd[c].append(
                                         (self.st(), m.linear.x, m.angular.z)), QOS)
        self.create_subscription(ModelStates, '/model_states', self.on_ms, QOS)
        self.create_subscription(Clock, '/clock', self.on_clock, QOS)

    def st(self):
        return self.clock if self.clock is not None else 0.0

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.sim0 is None:
            self.sim0 = t
        self.clock = t
        self.sim_n += 1

    def on_ms(self, msg):
        t = self.st()
        if not t:
            return
        for name, pose, tw in zip(msg.name, msg.pose, msg.twist):
            if name not in CARS:
                continue
            q = pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.v[name].append((t, math.hypot(tw.linear.x, tw.linear.y),
                                 tw.linear.x * math.cos(yaw) + tw.linear.y * math.sin(yaw),
                                 pose.position.x, pose.position.y))
            self.yaw_rate[name].append((t, tw.angular.z))


def main():
    rclpy.init()
    p = Probe()
    t_end = time.time() + DUR
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(p, timeout_sec=0.05)
    wall = time.time() - p.wall0
    sim = (p.clock - p.sim0) if p.clock and p.sim0 is not None else 0.0
    print('采样 %.1f s（墙钟） / %.1f s（仿真时钟） → RTF = %.2f，/clock 消息 %d 条' % (
        wall, sim, (sim / wall) if wall > 0 else 0, p.sim_n))
    for c in CARS:
        vs = p.v[c]
        if not vs:
            print('%s：没见到' % c)
            continue
        vm = [x[1] for x in vs]
        vl = [x[2] for x in vs]
        cs = p.cmd[c]
        cx = [x[1] for x in cs]
        wz = [y[1] for y in p.yaw_rate[c]] if p.yaw_rate[c] else [0]
        # 指令→实测的跟随比：把每个 cmd 样本对齐到它之后 0.3 s 内的最大实测速度。
        # 比值明显 >1 说明车在超发指令跑（打滑/被推/PID 过冲），控制器的
        # "cruise 0.7 所以不会冲过角点"这个前提就不成立。
        over = []
        for i, (tc, vx, wz_) in enumerate(cs):
            if vx < 0.5:
                continue
            w = [v for (tt, v, _, _, _) in vs if tc <= tt <= tc + 0.3]
            if w:
                over.append((max(w) / vx, tc, vx, max(w)))
        over.sort(reverse=True)
        print('%s：cmd 帧 %d（vx 范围 %.2f..%.2f）  实测 |v| max %.2f 均值 %.2f  '
              '|ω| max %.2f  超指令比 max %.2f×（%.2f→%.2f @t=%.1f）' % (
                  c, len(cs), min(cx, default=0), max(cx, default=0),
                  max(vm), sum(vm) / len(vm),
                  max([abs(w) for w in wz], default=0),
                  over[0][0] if over else 0,
                  over[0][2] if over else 0, over[0][3] if over else 0,
                  over[0][1] if over else 0))
        for r, tc, vx, mv in over[:3]:
            print('     样本 t=%.1f cmd=%.2f → 实测峰值 %.2f（%.2f×）' % (tc, vx, mv, r))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
