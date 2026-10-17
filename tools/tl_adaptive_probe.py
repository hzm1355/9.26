#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自适应算法取证探针：订阅 /traffic_light/adaptive_stats 与 6 组 adaptive_factor，
   另附一个"算法输入"列（当前落在排队区内、速度<1m/s 的车辆数，来自 /model_states），
   用于证明因子不是拍脑袋，而是由虚拟场景实时车流算出来的。"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

GRPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0


class Probe(Node):
    def __init__(self):
        super().__init__('tl_adaptive_probe')
        self.stats = None
        self.fac = {}
        self.cars = {}
        self.create_subscription(String, '/traffic_light/adaptive_stats',
                                 lambda m: setattr(self, 'stats', m.data), 10)
        for g in GRPS:
            self.create_subscription(
                String, f'/traffic_light/{g}/adaptive_factor',
                lambda m, gid=g: self.fac.__setitem__(gid, m.data), 10)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)

    def on_states(self, msg):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if not name.startswith(('car_', 'v2x_', 'smart_car_')):
                continue
            sp = (twist.linear.x ** 2 + twist.linear.y ** 2) ** 0.5
            self.cars[name] = (pose.position.x, pose.position.y, sp)


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < SECS and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)

    print('  信号组       adaptive_factor        停车等待中的车(x,y,速度)')
    for g in GRPS:
        print(f'  {g:<10} {n.fac.get(g, "(无数据)"):<22}')
    print('  场景内车辆实时位姿（算法的原始车流输入，speed<1 视为排队）:')
    for k, (x, y, sp) in sorted(n.cars.items()):
        print(f'    {k:<8} x={x:7.2f} y={y:7.2f} |v|={sp:4.2f} m/s '
              f'{"排队" if sp < 1.0 else "行驶"}')
    if n.stats:
        print('  adaptive_stats 原文:')
        try:
            for line in json.dumps(json.loads(n.stats), ensure_ascii=False,
                                   indent=2).splitlines():
                print('    ' + line)
        except ValueError:
            for line in n.stats.splitlines():
                print('    ' + line)
    else:
        print('  adaptive_stats: (未收到，检查 adaptive_timing_node 是否在跑)')
    n.destroy_node()
    rclpy.shutdown()


main()
