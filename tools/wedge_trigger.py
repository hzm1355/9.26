#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""活体复位链路触发器：把一辆正在巡航的车摆到"顶住静态盒"的位置，让驱动节点自己走
   楔住 → 倒车脱困×2 → 复位 这条链，用于验收两段式复位（先刹轮→落位→静默）。

   为什么选 (5.1,2.0) 顶 tl_j3_ns_ped：这是本轮取证里实测发生过的楔住点（改前 car_5
   在这里顶住灯杆，复位时被甩到 x=32.2 出界）。而且 sandbox_fixtures_coll 的等效盒
   刻意不在 obstacles.yaml 的避让清单里，车会一直顶着不放，链路必然走完。

   用法（驱动节点必须在跑，脚本自己不发 cmd_vel，只摆位+观察）：
     python3 tools/wedge_trigger.py --car=car_5 --xy=5.1,2.0 --yaw=0 --observe=30
"""
import argparse
import math
import time

import rclpy
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class Trigger(Node):

    def __init__(self, a):
        super().__init__('wedge_trigger')
        self.a = a
        self.t = 0.0
        self.rows = []
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Clock, '/clock', self._clk,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self._st, 10)

    def _clk(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def _st(self, m):
        if self.a.car not in m.name or self.t <= 0:
            return
        i = m.name.index(self.a.car)
        p = m.pose[i].position
        q = m.pose[i].orientation
        tw = m.twist[i]
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rows.append((self.t, p.x, p.y, p.z, yaw,
                          math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z)))

    def spin_for(self, secs, label=''):
        """墙钟窗口，但打印的是仿真时刻（/clock 被量化到 10 Hz，一行就是一格）。

        30 s 有 300 格，逐行打印会把关键事件埋掉，所以只打：每 10 格一格的心跳、
        单格位移 >0.4 m 的瞬移、以及 mv 越过 0.85 的越界格。
        """
        n0 = len(self.rows)
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        rows = self.rows[n0:]
        print(f'  [{label}] {len(rows)} 格，t {rows[0][0]:.1f}→{rows[-1][0]:.1f}'
              if rows else f'  [{label}] 零格（订阅没通）')
        prev = None
        for i, r in enumerate(rows):
            jump = math.hypot(r[1] - prev[1], r[2] - prev[2]) if prev else 0.0
            if i % 10 == 0 or jump > 0.4 or r[5] > 0.85:
                mark = f'  瞬移{jump:.1f}m' if jump > 0.4 else ''
                print(f'  t={r[0]:8.2f} ({r[1]:7.2f},{r[2]:7.2f}) z={r[3]:5.2f} '
                      f'yaw={r[4]:+.2f} mv={r[5]:5.2f} mw={r[6]:4.2f}{mark}')
            prev = r
        if rows:
            print(f'  [{label}] 小峰值 mv={max(r[5] for r in rows):.2f} '
                  f'mw={max(r[6] for r in rows):.2f} '
                  f'z={min(r[3] for r in rows):.2f}..{max(r[3] for r in rows):.2f} '
                  f'末位({rows[-1][1]:.2f},{rows[-1][2]:.2f})')
        return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_5')
    ap.add_argument('--xy', default='5.1,2.0')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--observe', type=float, default=30.0)
    a = ap.parse_args()
    x, y = [float(v) for v in a.xy.split(',')]

    rclpy.init()
    node = Trigger(a)
    while not node.cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn('/set_entity_state 未就绪，继续等待')
    if not node.spin_for(2.0, '摆位前'):
        print('!! /clock 或 /model_states 一条都没收到（QoS/DDS 发现问题），本轮作废')
        return 1

    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = a.car
    req.state.pose.position.x = x
    req.state.pose.position.y = y
    req.state.pose.position.z = a.z
    req.state.pose.orientation.z = math.sin(a.yaw / 2.0)
    req.state.pose.orientation.w = math.cos(a.yaw / 2.0)
    req.state.reference_frame = 'world'
    req.state.twist.linear.x = 0.0
    req.state.twist.angular.z = 0.0
    fut = node.cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    ok = bool(fut.result() and fut.result().success)
    print(f'摆位 {a.car} -> ({x:.2f},{y:.2f}) z={a.z} yaw={a.yaw:+.2f}  {"OK" if ok else "失败"}')
    if not ok:
        return 1

    print(f'观察 {a.observe:.0f} s，等驱动节点走完 楔住→脱困×2→复位：')
    node.spin_for(a.observe)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
