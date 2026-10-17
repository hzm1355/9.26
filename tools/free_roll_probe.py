#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判"车自己动"的力是**全局的（顺坡自滚）**还是**沿车头轴的（接触弹射/自驱）**。

背景：recover_fleet.py 把五辆车摆得干干净净（落点偏差 0.00 m、摆后 4 s 位移 0.00 m），
但驱动一起来 3.5 s 就把 car_3 从 y=-11.6 推到 y=-19.3（7.7 m / 3.45 s ≈ 2.2 m/s，
而 cruise_speed 只有 0.7），接着就是整队的"出界复位风暴"。
必须先分清这个位移是谁给的，否则改哪儿都是瞎改。

做法（同一点位跑两次，只换车头朝向）：
  阶段1 灌零速 20 Hz 把车锁住 —— 期望位移 0，用来确认"没有别的力在推它"；
  阶段2 **彻底撤掉这辆车的发布者**（其余四辆继续锁着，防止它们漂走污染现场），
        看这辆车自由 25 s 会不会自己动、往哪个方向动。
  yaw=0（车头朝 +x）与 yaw=+90°（车头朝 +y）两次结果对比：
    两次位移向量相同 → 与朝向无关 → 全局力（地面坡度）；
    只在某个朝向上动 / 位移随车头转 → 沿车体轴的力（弹射或残留驱动）。

用法：python3 free_roll_probe.py --car car_3 --x -0.8 --y -11.6 --yaw 0
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

ALL = ('car_1', 'car_2', 'car_3', 'car_4', 'car_5')
_T0 = time.monotonic()


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self, car, z):
        super().__init__('free_roll_probe')
        self.car = car
        self.z = z
        self.hold_on = set(ALL)
        self.latest = {}
        self.pub = {n: self.create_publisher(Twist, '/%s/cmd_vel' % n, 10) for n in ALL}
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_timer(0.05, self.on_hold)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')

    def on_states(self, msg):
        for n, p in zip(msg.name, msg.pose):
            if n.startswith('car_'):
                self.latest[n] = (p.position.x, p.position.y,
                                  yaw_of(p.orientation), p.position.z)

    def on_hold(self):
        for n in self.hold_on:
            self.pub[n].publish(Twist())

    def spin_wait(self, sec):
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def place(self, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = self.car
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = self.z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        t0 = time.monotonic()
        while not self.cli.wait_for_service(timeout_sec=3.0):
            if time.monotonic() - t0 > 60:
                raise SystemExit('!! /set_entity_state 60 s 没发现')
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return bool(fut.result() and fut.result().success)

    def trace(self, sec, tag):
        print('  [%s] 每秒位姿序列' % tag)
        prev = None
        first = None
        for i in range(sec):
            self.spin_wait(1.0)
            p = self.latest.get(self.car)
            if p is None:
                print('      无位姿读数')
                continue
            dx, dy = (p[0] - prev[0], p[1] - prev[1]) if prev else (0.0, 0.0)
            if prev and first is None:
                first = prev
            d = math.hypot(dx, dy)
            bearing = math.degrees(math.atan2(dy, dx)) if d > 0.02 else None
            others = min([math.hypot(q[0] - p[0], q[1] - p[1])
                          for n, q in self.latest.items() if n != self.car] or [99.0])
            print('      t=%2d s (%7.2f,%7.2f) yaw%+6.1f°  1s位移 %5.2f m 方向 %s  他车最近 %.1f m' % (
                i + 1, p[0], p[1], math.degrees(p[2]), d,
                '%+.0f°' % bearing if bearing is not None else '-', others))
            prev = p
        p = self.latest.get(self.car)
        if prev and first:
            print('  [%s] 合计位移 %.2f m，方向 %+.0f°（0°=+x，90°=+y）' % (
                tag, math.hypot(p[0] - first[0], p[1] - first[1]),
                math.degrees(math.atan2(p[1] - first[1], p[0] - first[0]))))
        return (p[0] - first[0], p[1] - first[1]) if (p and first) else (0.0, 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_3')
    ap.add_argument('--x', type=float, default=-0.8)
    ap.add_argument('--y', type=float, default=-11.6)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='车头朝向：0=+x，1.5708=+y，-1.5708=-y')
    ap.add_argument('--hold-sec', type=int, default=15)
    ap.add_argument('--free-sec', type=int, default=25)
    a = ap.parse_args()

    rclpy.init()
    nd = Probe(a.car, a.z)
    while not nd.latest and time.monotonic() - _T0 < 30:
        rclpy.spin_once(nd, timeout_sec=0.2)
    print('可见车辆 %d 辆；本车 %s 摆到 (%.2f,%.2f) 车头 %+0.1f°' % (
        len(nd.latest), a.car, a.x, a.y, math.degrees(a.yaw)))
    print('摆前先把五辆车全部锁住（20 Hz 零速），避免别的车漂进落点')
    nd.spin_wait(3.0)
    print('  摆放回执 success=%s' % nd.place(a.x, a.y, a.yaw))
    nd.spin_wait(1.0)

    d_hold = nd.trace(a.hold_sec, '阶段1 锁轮')
    nd.hold_on.discard(a.car)          # 只放掉本车的指令，其余四辆继续锁
    print('  >>> 已撤掉 /%s/cmd_vel 的发布者，其余四辆仍锁着' % a.car)
    d_free = nd.trace(a.free_sec, '阶段2 自由')

    print('\n--- 结论素材 ---')
    print('  阶段1（锁轮 %d s）位移 (%.2f, %.2f) = %.2f m' % (
        a.hold_sec, d_hold[0], d_hold[1], math.hypot(*d_hold)))
    print('  阶段2（自由 %d s）位移 (%.2f, %.2f) = %.2f m' % (
        a.free_sec, d_free[0], d_free[1], math.hypot(*d_free)))
    print('  车头朝向 %+0.1f°：如果换朝向再跑一次，阶段2 的位移向量跟着车头转，'
          '就是沿车体轴的力；两次相同就是全局力（坡度）。' % math.degrees(a.yaw))
    nd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
