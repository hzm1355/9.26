#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回放取证探针：在回放期间实际订阅 6 组灯态/行人灯态与车辆位姿，
把收到的相变序列和位姿样本落成 JSON，供与录制文件逐条对照。
用法：python3 replay_probe.py <采样秒数> <输出json>
"""
import json
import math
import os
import sys
import threading
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 34.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/replay_observed.json'
GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5']


class Probe(Node):

    def __init__(self):
        super().__init__('replay_probe')
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.light = []          # [t_rel, group, phase]
        self.ped = []            # [t_rel, group, phase]
        self.last_l = {}
        self.last_p = {}
        self.poses = []          # [t_rel, {name: [x, y, yaw]}]
        self.cur = {}
        self.last_snap = -9.9
        for g in GROUPS:
            self.create_subscription(
                String, f'/traffic_light/{g}/state',
                lambda m, g=g: self._edge(self.last_l, self.light, g, m.data), 10)
            self.create_subscription(
                String, f'/traffic_light/{g}/pedestrian',
                lambda m, g=g: self._edge(self.last_p, self.ped, g, m.data), 10)
        self.create_subscription(ModelStates, '/model_states', self._models, 10)

    def rel(self):
        return self.get_clock().now().nanoseconds / 1e9 - self.t0

    def _edge(self, last, bucket, gid, phase):
        if last.get(gid) != phase:
            last[gid] = phase
            bucket.append([round(self.rel(), 3), gid, phase])

    def _models(self, msg):
        now = self.rel()
        for i, name in enumerate(msg.name):
            if name in CARS:
                p = msg.pose[i].position
                q = msg.pose[i].orientation
                self.cur[name] = [round(p.x, 3), round(p.y, 3),
                                  round(2.0 * math.atan2(q.z, q.w), 3)]
        # /model_states 约 100 Hz，只按录制采样率（5 Hz）留存，避免样本被淹
        if len(self.cur) == len(CARS) and now - self.last_snap >= 0.2:
            self.last_snap = now
            self.poses.append([round(now, 3), dict(self.cur)])


def main():
    rclpy.init()
    node = Probe()
    print(f'PROBE_READY {node.t0:.3f}', flush=True)

    def report():
        time.sleep(DUR)
        with open(OUT, 'w', encoding='utf-8') as f:
            json.dump({'duration': DUR, 'light': node.light,
                       'pedestrian': node.ped, 'poses': node.poses}, f)
        print(f'  探针结束：灯态相变={len(node.light)} 行人相变={len(node.ped)} '
              f'位姿帧={len(node.poses)} -> {OUT}')
        os._exit(0)

    threading.Thread(target=report, daemon=True).start()
    # 探针与播放器都用系统时钟：打印自己的零点对齐时刻，供脚本精确扣除启动耗时
    print(f'PROBE_READY {node.t0:.3f}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
