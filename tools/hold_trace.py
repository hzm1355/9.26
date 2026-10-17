#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摆一辆车，然后每秒打印它的位姿时间序列 —— 判"+x 位移 17 m"到底是自滚、被撞还是指令没断。

为什么单独做这个：受控取证连着 5 轮都在 `settle()` 上失败，每辆车每次重试的位移都
稳定在 16.5 m/3 s（≈5.5 m/s 匀速，而且都是从目标指向 +x）。自滚应该每次从静止加速、
位移随坡度变化，被撞应该方向和大小都不固定 —— 两者都对不上，需要原始时间序列判。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from rosgraph_msgs.msg import Clock


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Trace(Node):
    def __init__(self, args):
        super().__init__('hold_trace')
        self.args = args
        self.sim_t = None
        self.rows = {}
        self.pub = {}
        for n in ('car_1', 'car_2', 'car_3', 'car_4', 'car_5'):
            self.pub[n] = self.create_publisher(Twist, '/%s/cmd_vel' % n, 10)
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        if self.hold:
            self.create_timer(0.05, self.on_hold)

    @property
    def hold(self):
        return self.args.hold == '1'

    def on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def on_states(self, msg):
        for n, p, t in zip(msg.name, msg.pose, msg.twist):
            self.rows[n] = (p.position.x, p.position.y, p.position.z,
                            yaw_of(p.orientation),
                            math.hypot(t.linear.x, t.linear.y))

    def on_hold(self):
        for pub in self.pub.values():
            pub.publish(Twist())

    def spin(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)


_T0 = time.monotonic()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_4')
    ap.add_argument('--x', type=float, default=2.0)
    ap.add_argument('--y', type=float, default=-14.6)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='摆正朝向：0=车头朝+x，1.5708=朝+y（用来区分"全局力"和"沿车头自驱"）')
    ap.add_argument('--hold', default='1', help='1=全程 20Hz 灌零速 Twist')
    ap.add_argument('--sec', type=int, default=20)
    a = ap.parse_args()

    rclpy.init()
    nd = Trace(a)
    while nd.sim_t is None and time.monotonic() - _T0 < 30:
        rclpy.spin_once(nd, timeout_sec=0.2)
    print('仿真 %.1f s，可见实体 %d 个' % (nd.sim_t, len(nd.rows)))

    req = SetEntityState.Request()
    req.state.name = a.car
    req.state.pose.position.x = a.x
    req.state.pose.position.y = a.y
    req.state.pose.position.z = a.z
    req.state.pose.orientation.w = 1.0
    req.state.reference_frame = 'world'
    t0 = time.monotonic()
    while not nd.cli.wait_for_service(timeout_sec=3.0):
        if time.monotonic() - t0 > 60:
            raise SystemExit('!! /set_entity_state 60 s 没发现')
    fut = nd.cli.call_async(req)
    rclpy.spin_until_future_complete(nd, fut, timeout_sec=20.0)
    r = fut.result()
    print('摆 %s → (%.1f,%.1f,%.2f) 服务 success=%s（hold=%s）'
          % (a.car, a.x, a.y, a.z, None if r is None else r.success, a.hold))

    print('%-6s %-9s %-9s %-8s %-8s %-8s %-8s %s'
          % ('墙钟', 'x', 'y', 'z', 'yaw', '上报v', '1s位移', '他车最近'))
    prev = None
    for i in range(a.sec + 1):
        nd.spin(1.0)
        p = nd.rows.get(a.car)
        if p is None:
            print('  没有 %s 的位姿' % a.car)
            continue
        d = math.hypot(p[0] - prev[0], p[1] - prev[1]) if prev else 0.0
        near, near_n = 1e9, ''
        for n, q in nd.rows.items():
            if n == a.car or not n.startswith('car_'):
                continue
            dd = math.hypot(q[0] - p[0], q[1] - p[1])
            if dd < near:
                near, near_n = dd, n
        print('%6.1f %9.3f %9.3f %8.3f %8.3f %8.2f %8.3f %s'
              % (time.monotonic() - _T0, p[0], p[1], p[2], p[3], p[4], d,
                 ('%s %.2f m' % (near_n, near)) if near < 1e8 else '-'))
        prev = p
    rclpy.shutdown()


if __name__ == '__main__':
    main()
