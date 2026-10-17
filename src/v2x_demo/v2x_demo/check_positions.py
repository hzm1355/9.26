#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""快照 /model_states 中匹配前缀的模型位姿，打印后退出。
用法: check_positions.py <prefix> [prefix...]
健壮性：有界等待(timeout)，不在回调内 rclpy.shutdown（会死锁），用 spin_once 轮询。"""
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node

PREFIXES = sys.argv[1:] or ['v2x_car', 'playback_']
TIMEOUT = 15.0


def main():
    rclpy.init()
    node = Node('snap')
    box = {}

    def cb(m):
        if 'done' in box:
            return
        rows = []
        for n, p in zip(m.name, m.pose):
            if any(n.startswith(pre) for pre in PREFIXES):
                rows.append(f'{n} x={p.position.x:.2f} y={p.position.y:.2f} z={p.position.z:.2f}')
        if rows:
            box['rows'] = rows
            box['done'] = True

    node.create_subscription(ModelStates, '/model_states', cb, 10)
    t0 = time.time()
    while 'done' not in box and (time.time() - t0) < TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.5)
    for r in box.get('rows', []):
        print(r)
    if 'rows' not in box:
        print(f'(超时 {TIMEOUT}s 未收到匹配 {PREFIXES} 的模型)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
