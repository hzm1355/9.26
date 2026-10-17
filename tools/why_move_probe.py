#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判"车为什么以 8 m/s 往前跑"：位姿微分速度 vs 差速插件 odom，再分阶段试零速与反向指令。

  odom ≈ 位姿微分 → 轮子真在转（有 cmd 来源，或被坡/求解器带着转）；
  odom ≈ 0 而位姿在动 → 轮子没转、车身在滑，是接触面问题，不是驱动问题。
  cmd=0 刹不住、cmd=-3 能倒退 → 插件在听 cmd_vel，runaway 源头是某个发布者；
  两段都毫无反应 → 插件根本没控住轮子。
用法（配合 hold_trace.sh，它会先杀掉车队驱动）：
  SCRIPT=why_move_probe.py bash hold_trace.sh --car car_4 --x 2 --y -14.6 --yaw 1.5708
"""
import argparse
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

_T0 = time.monotonic()


class Probe(Node):
    def __init__(self, car):
        super().__init__('why_move_probe')
        self.car = car
        self.sim_t = None
        self.pose = None
        self.odom = None
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % car, 10)
        self.create_subscription(
            Clock, '/clock',
            lambda m: setattr(self, 'sim_t', m.clock.sec + m.clock.nanosec * 1e-9),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_subscription(Odometry, '/%s/odom' % car, self.on_odom, 20)

    def on_states(self, msg):
        for n, p in zip(msg.name, msg.pose):
            if n == self.car:
                self.pose = (p.position.x, p.position.y, p.position.z)

    def on_odom(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y,
                     m.twist.twist.linear.x, m.twist.twist.angular.z)

    def spin(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_4')
    ap.add_argument('--x', type=float, default=2.0)
    ap.add_argument('--y', type=float, default=-14.6)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='0=车头朝 +x，1.5708=朝 +y。换朝向后还是往 +x 跑 → 全局力；'
                         '改成沿新车头跑 → 沿车头的自驱。两种病因完全不同。')
    a = ap.parse_args()

    rclpy.init()
    nd = Probe(a.car)
    while nd.sim_t is None and time.monotonic() - _T0 < 30:
        rclpy.spin_once(nd, timeout_sec=0.2)
    print('仿真 %.1f s，%s 位姿 %s，odom %s' % (nd.sim_t, a.car, nd.pose, nd.odom))

    cli = nd.create_client(SetEntityState, '/set_entity_state')
    t0 = time.monotonic()
    while not cli.wait_for_service(timeout_sec=3.0):
        if time.monotonic() - t0 > 60:
            raise SystemExit('!! /set_entity_state 60 s 没发现')
    req = SetEntityState.Request()
    req.state.name = a.car
    req.state.pose.position.x = a.x
    req.state.pose.position.y = a.y
    req.state.pose.position.z = a.z
    req.state.pose.orientation.z = math.sin(a.yaw / 2.0)
    req.state.pose.orientation.w = math.cos(a.yaw / 2.0)
    req.state.twist.linear.x = 0.0
    req.state.twist.angular.z = 0.0
    req.state.reference_frame = 'world'
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(nd, fut, timeout_sec=20.0)
    r = fut.result()
    print('摆 %s → (%.1f,%.1f) yaw=%.2f success=%s（请求里显式把 twist 归零）'
          % (a.car, a.x, a.y, a.yaw, None if r is None else r.success))

    print('%-6s %-17s %-8s %-17s %-9s %-8s %s'
          % ('墙钟', '世界位姿 xy', 'z', 'odom xy', 'odom 线速', '位姿微分', '阶段'))
    prev = [None]

    def trace(sec, cmd, label):
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            if cmd is not None:
                t = Twist()
                t.linear.x = cmd
                nd.pub.publish(t)
            nd.spin(1.0)
            if nd.pose is None:
                continue
            v = 0.0
            if prev[0]:
                v = math.hypot(nd.pose[0] - prev[0][0], nd.pose[1] - prev[0][1])
            print('%6.1f (%7.2f,%7.2f)  %8.3f (%7.2f,%7.2f)  %9.2f %8.2f  %s'
                  % (time.monotonic() - _T0, nd.pose[0], nd.pose[1], nd.pose[2],
                     nd.odom[0] if nd.odom else float('nan'),
                     nd.odom[1] if nd.odom else float('nan'),
                     nd.odom[2] if nd.odom else float('nan'), v, label))
            prev[0] = nd.pose

    trace(10, None, '不发指令')
    trace(8, 0.0, 'cmd=0')
    trace(8, -3.0, 'cmd=-3.0')

    print('--- /%s/cmd_vel 端点 ---' % a.car)
    out = subprocess.run(['ros2', 'topic', 'info', '-v', '/%s/cmd_vel' % a.car],
                         capture_output=True, text=True, timeout=25).stdout
    print('\n'.join(out.strip().splitlines()[:14]))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
