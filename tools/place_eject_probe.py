#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""隔离实验：复位摆位到底是不是自带弹射？

#77 的五车同录显示：每一次"复位"摆位之后，实测线速度都以固定 2.8 m/s^2 的斜率
一路爬到 4~8.5 m/s（指令上限只有 0.70，而且摆位后指令经常是 -0.10 的刹车），
车沿着车头方向冲出路网（car_2 从 x=11.5 飞到 x=37.2，出图）。
这份脚本用来把两个来源分开：
  A) 摆位本身危险（轮子已经刹停、全程零速指令，仍然弹）；
  B) 弹射能量来自"给油不走"的驱动状态（摆位前后都发 cruise 才弹）。

用法（WSL，先杀掉 signal_drive_node，否则会抢 cmd_vel）：
  python3 tools/place_eject_probe.py --car=car_2 --xy=11.5,1.5 --yaw=0 --drive=zero
  python3 tools/place_eject_probe.py --car=car_2 --xy=11.5,1.5 --yaw=0 --drive=cruise
"""
import argparse
import math
import sys

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class Probe(Node):

    def __init__(self, a):
        super().__init__('place_eject_probe')
        self.a = a
        self.t = 0.0
        self.pose = None            # (t, x, y, yaw, mv, mw)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % a.car, 10)
        self.others = {}
        for nm in a.silence.split(','):
            if nm.strip() and nm.strip() != a.car:
                self.others[nm.strip()] = self.create_publisher(
                    Twist, '/%s/cmd_vel' % nm.strip(), 10)
        self.create_subscription(Clock, '/clock', self._clk,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        from gazebo_msgs.msg import ModelStates
        self.create_subscription(ModelStates, '/model_states', self._st, 10)

    def _clk(self, m):
        self.t = m.clock.sec + m.clock.nanosec * 1e-9

    def _st(self, m):
        if self.a.car not in m.name:
            return
        i = m.name.index(self.a.car)
        p = m.pose[i].position
        q = m.pose[i].orientation
        tw = m.twist[i]
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (self.t, p.x, p.y, yaw,
                     math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z))

    def spin_for(self, secs, cmd):
        """按 20 Hz 发 cmd（None=不发），同时抽样打印"""
        t_end = self.t + secs
        last = None
        step = 0
        while rclpy.ok() and self.t < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cmd is not None and step % 2 == 0:
                t = Twist()
                t.linear.x = cmd
                self.pub.publish(t)
            for o in self.others.values():
                o.publish(Twist())
            step += 1
            if self.pose and self.pose[0] != last:
                last = self.pose[0]
                yield self.pose
                if self.a.verbose:
                    print('  t=%8.2f (%7.2f,%7.2f) yaw=%+6.2f mv=%5.2f mw=%5.2f'
                          % self.pose)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_2')
    ap.add_argument('--xy', default='11.5,1.5')
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--drive', default='zero', choices=('zero', 'cruise'))
    ap.add_argument('--cruise', type=float, default=0.70)
    ap.add_argument('--brake', type=float, default=1.5, help='摆位前保持该指令的秒数')
    ap.add_argument('--hold', type=float, default=6.0, help='摆位后观察秒数')
    ap.add_argument('--silence', default='car_1,car_2,car_3,car_4,car_5',
                    help='这些车一律发零速（除被测车），防止实验期间撞车')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    cmd = 0.0 if a.drive == 'zero' else a.cruise
    x0, y0 = [float(v) for v in a.xy.split(',')]

    rclpy.init()
    nd = Probe(a)
    waited = 0.0
    while nd.t == 0.0 and waited < 20.0:
        waited += 0.1
        rclpy.spin_once(nd, timeout_sec=0.1)
    if nd.t == 0.0:
        print('没收到 /clock，退出（不要拿空表当结论）')
        return 1

    print('阶段1：摆位前 %s 保持 %.2f m/s（其余车零速）' % (a.drive, cmd))
    samples = list(nd.spin_for(a.brake, cmd))
    pre = samples[-1] if samples else None
    print('  摆位前实测 mv=%.2f mw=%.2f @ (%.2f,%.2f)' % (pre[4], pre[5], pre[1], pre[2]))

    req = SetEntityState.Request()
    req.state = EntityState()
    req.state.name = a.car
    req.state.pose.position.x = x0
    req.state.pose.position.y = y0
    req.state.pose.position.z = a.z
    req.state.pose.orientation.z = math.sin(a.yaw / 2.0)
    req.state.pose.orientation.w = math.cos(a.yaw / 2.0)
    req.state.reference_frame = 'world'
    req.state.twist.linear.x = 0.0
    req.state.twist.angular.z = 0.0
    fut = nd.cli.call_async(req)
    rclpy.spin_until_future_complete(nd, fut, timeout_sec=5.0)
    ok = bool(fut.result() and fut.result().success)
    print('阶段2：SetEntityState -> (%.2f,%.2f) z=%.2f  %s' % (
        x0, y0, a.z, 'OK' if ok else '失败：%s' % (fut.result().status_message if fut.result() else '?')))
    if not ok:
        return 1

    print('阶段3：摆位后 %s 观察 %.1f s' % (a.drive, a.hold))
    post = list(nd.spin_for(a.hold, cmd))
    nd.pub.publish(Twist())
    for o in nd.others.values():
        o.publish(Twist())

    if not post:
        print('摆位后一格样本都没拿到 —— 别把空表当结论')
        return 1
    d0 = post[0]
    peak_v = max(s[4] for s in post)
    peak_w = max(s[5] for s in post)
    moved = math.hypot(post[-1][1] - d0[1], post[-1][2] - d0[2])
    dt = post[-1][0] - d0[0] or 1e-3
    slope = (post[-1][4] - d0[4]) / dt
    print('\n判读（指令 %.2f m/s，物理上限就是它）：' % cmd)
    print('  摆位后首格 mv=%.2f  峰值 mv=%.2f  mw=%.2f' % (d0[4], peak_v, peak_w))
    print('  %.1f s 内位移 %.2f m（等效 %.2f m/s）  平均斜率 %.2f m/s^2'
          % (dt, moved, moved / dt, slope))
    if peak_v > cmd + 0.25:
        print('  → 弹射：实测越过指令上限 %.2f。%s' % (
            peak_v - cmd,
            '全程零速仍弹 ⇒ 摆位本身危险（模型状态/接触解算），跟驱动无关'
            if cmd == 0.0 else '给油才弹 ⇒ 能量来自驱动/楔住状态'))
    else:
        print('  → 未弹射：实测与指令一致（差 %.2f）' % (peak_v - cmd))
    nd.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
