#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#90 逐帧盯 150 s：car_6 被弹走的那一刻，旁边到底发生了什么。

已经排除的两件事：
  · 插件在开车 —— selfdrive_probe 里 /car_6/odom 全程 v=0.00、w=0.00，
    而 /model_states 真值冲到 6.3 m/s，轮子不转、车心在滑，纯接触推挤；
  · 落点本身有静态件顶它 —— land_occupancy_probe 摆位后 15 s 只漂 0.09 m，
    最近他模型是 0.80 m 外的闸机相机龙门架（没碰）。

剩下的嫌疑是**间歇性**的：车队行车道就在东侧 x=14.5（signal_drive_params 里
右环 route 末段 [14.5,1.5]→[14.5,-16.8]），而 car_6 每次都被弹进那条道
（12.0,-12.8 → 13.3 → 14.4 → 14.97，y 都掉在 -13.4 附近）。
signal_drive 的卡死复位会直接 SetEntityState 把某辆 he 车搬到路线点上，
搬进 car_6 里就是瞬间爆炸。所以这一发把六辆车**逐帧**一起记：
谁在哪一帧位移突跳（>0.5 m/帧）、谁跟 car_6 的间距压到接触尺度（<1.7 m）。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']
CONTACT = 1.70          # 两车中心距：1.60 m 车长的一半 ×2 再留一点


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self, model):
        super().__init__('watch90')
        self.model = model
        self.snap = {}
        self.stamp = 0.0
        self.publish = False
        self.create_subscription(ModelStates, '/model_states', self.on, 10)
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % model, 10)
        self.create_timer(0.05, self.tick)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            pass

    def tick(self):
        if self.publish:
            self.pub.publish(Twist())

    def on(self, m):
        d = {}
        for i, name in enumerate(m.name):
            if name in CARS:
                p = m.pose[i]
                d[name] = (p.position.x, p.position.y, p.position.z,
                           yaw_of(p.orientation))
        self.snap = d
        self.stamp = time.monotonic()

    def spin_to(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def place(self, x, y, z, yaw):
        r = SetEntityState.Request()
        r.state.name = self.model
        r.state.pose.position.x = x
        r.state.pose.position.y = y
        r.state.pose.position.z = z
        r.state.pose.orientation.z = math.sin(yaw / 2.0)
        r.state.pose.orientation.w = math.cos(yaw / 2.0)
        r.state.reference_frame = 'world'
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=8.0)
        return bool(f.result() and f.result().success)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--secs', type=float, default=150.0)
    ap.add_argument('--every', type=float, default=0.5)
    a = ap.parse_args()
    rclpy.init()
    n = N(a.model)
    n.spin_to(2.0)
    n.publish = True
    n.spin_to(1.5)
    n.publish = False
    print('摆位 %s → (%.2f,%.2f,%.2f) yaw=%+.4f' % (
        'ok' if n.place(a.x, a.y, a.z, a.yaw) else 'FAIL', a.x, a.y, a.z, a.yaw))

    prev = dict(n.snap)
    prev_t = n.stamp
    t0 = time.time()
    next_print = 0.0
    n_ev = 0
    print('  t     car_6(x,y,yaw)        |v|   最近他车(距离)          突跳的车')
    while time.time() - t0 < a.secs:
        st = n.stamp
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.stamp <= st or a.model not in n.snap:
            continue
        dt = st - prev_t
        if dt <= 1e-3:
            continue
        jump = []
        spd = {}
        for k, cur in n.snap.items():
            if k in prev:
                dd = math.hypot(cur[0] - prev[k][0], cur[1] - prev[k][1])
                spd[k] = dd / dt
                if dd > 0.5:
                    jump.append('%s %.2f m' % (k, dd))
        x, y, z, h = n.snap[a.model]
        near = sorted(((math.hypot(px - x, py - y), k)
                       for k, (px, py, _, _) in n.snap.items() if k != a.model))
        v = spd.get(a.model, 0.0)
        flag = v > 0.30 or jump or (near and near[0][0] < CONTACT)
        if flag:
            n_ev += 1
        if time.time() - t0 >= next_print and (flag or time.time() - t0 < 3):
            print('  %5.1f (%7.3f,%7.3f)%+.2f %5.2f  %s%s' % (
                time.time() - t0, x, y, h, v,
                '%s@%.2f m(车 v=%.2f)' % (near[0][1], near[0][0],
                                          spd.get(near[0][1], 0.0)) if near else '-',
                '   突跳: ' + ', '.join(jump) if jump else ''))
            next_print = time.time() - t0 + a.every
        prev = dict(n.snap)
        prev_t = st
    x, y, z, h = n.snap.get(a.model, (float('nan'),) * 4)
    print('  收尾 (%.3f,%.3f) yaw=%+.3f，离落点 %.2f m；事件行 %d 条' % (
        x, y, h, math.hypot(x - a.x, y - a.y), n_ev))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
