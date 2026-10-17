#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把指定车辆摆到指定位置并**回读确认**（复位风暴留下的车会自己漂，光看服务回执不够）。

用法：python3 park_and_verify.py 车名:x,y [车名:x,y ...]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

Z = 6.70


class N(Node):
    def __init__(self):
        super().__init__('park_and_verify')
        self.snap = {}
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(ModelStates, '/model_states', self.on, 20)

    def on(self, m):
        for n, p, t in zip(m.name, m.pose, m.twist):
            self.snap[n] = (p.position.x, p.position.y, p.position.z,
                            t.linear.x, t.linear.y)

    def settle(self, dur=0.6):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.05)


def put(node, name, x, y):
    req = SetEntityState.Request()
    req.state.name = name
    req.state.pose.position.x = x
    req.state.pose.position.y = y
    req.state.pose.position.z = Z
    req.state.pose.orientation.w = 1.0
    req.state.reference_frame = 'world'
    fut = node.cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    # SetEntityState 的响应只有 bool success（没有 status_message，取它会 AttributeError
    # 并把整个脚本掀掉 —— 任务三同款坑：srv 常量挂在 Request 上，字段要以 interface show 为准）。
    print('  %s 服务回执 success=%s' % (name, r.success if r else None))


def main():
    rclpy.init()
    node = N()
    t0 = time.monotonic()
    while not node.snap and time.monotonic() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    print('摆位前：')
    for spec in sys.argv[1:]:
        name, xy = spec.split(':')
        x, y = [float(v) for v in xy.split(',')]
        print('  %-6s 现在 %s' % (name, node.snap.get(name)))
        put(node, name, x, y)
    node.settle(1.0)
    print('摆位后 1 s：')
    for spec in sys.argv[1:]:
        name, xy = spec.split(':')
        x, y = [float(v) for v in xy.split(',')]
        s = node.snap.get(name)
        err = math.hypot(s[0] - x, s[1] - y) if s else -1
        print('  %-6s 目标(%.1f,%.1f) 实测(%s) 偏差 %.2f m %s' % (
            name, x, y, ('%.2f,%.2f' % (s[0], s[1])) if s else '?', err,
            'OK' if err < 0.5 else '没挪动'))
    print('再等 5 s 看它会不会自己漂走：')
    node.settle(5.0)
    for spec in sys.argv[1:]:
        name, xy = spec.split(':')
        x, y = [float(v) for v in xy.split(',')]
        s = node.snap.get(name)
        err = math.hypot(s[0] - x, s[1] - y) if s else -1
        print('  %-6s 偏差 %.2f m  线速度(%.2f,%.2f) z=%.2f' % (
            name, err, s[3], s[4], s[2]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
