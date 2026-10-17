#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#86 落点 occupancy 探针：摆位瞬间把周围 3 m 内的**每一个模型**都点名。

selfdrive_probe 已经把"插件在开车"排除了：摆位后 /car_6/odom 全程 v=0.00 w=0.00，
而 /model_states 真值冲到 6.3 m/s，轮子不转、车心在滑 ⇒ 是接触求解把车推出去的。
车还反复在两个簇之间来回跳：落点 (12.0,-12.7) 附近与 (15.1,-13.4) 附近 ——
后者正是 selfdrive85 收尾时 car_5 的位置 (15.02,-13.48)。

所以这一发要回答两个问题：
  1) 摆位那一刻，落点 3 m 半径内静态件/他车分别是谁、多远；
  2) 弹出过程中最小pair距离出现在哪一帧、跟谁。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self, model):
        super().__init__('land_occupancy_probe')
        self.model = model
        self.snap = {}
        self.stamp = 0.0
        self.create_subscription(ModelStates, '/model_states', self.on, 10)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            pass

    def on(self, m):
        d = {}
        for i, name in enumerate(m.name):
            p = m.pose[i]
            d[name] = (p.position.x, p.position.y, p.position.z,
                       yaw_of(p.orientation))
        self.snap = d
        self.stamp = time.monotonic()

    def wait(self, secs):
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
    ap.add_argument('--secs', type=float, default=15.0)
    a = ap.parse_args()
    rclpy.init()
    n = N(a.model)
    n.wait(2.0)
    if not n.snap:
        print('读不到 /model_states')
        return 2

    print('=== 0) 摆位前：落点 (%.2f,%.2f) 3 m 内的全部模型 ===' % (a.x, a.y))
    rows = []
    for k, (x, y, z, h) in n.snap.items():
        d = math.hypot(x - a.x, y - a.y)
        rows.append((d, k, x, y, z, h))
    rows.sort()
    print('  场景模型总数=%d' % len(rows))
    for d, k, x, y, z, h in rows:
        if d <= 3.0:
            print('   %-26s (%8.3f,%8.3f) z=%5.2f yaw=%+.3f  离落点 %.3f m'
                  % (k, x, y, z, h, d))

    print('=== 1) 摆位 ===')
    print('  place %s → (%.2f,%.2f,%.2f) yaw=%+.4f' % (
        'ok' if n.place(a.x, a.y, a.z, a.yaw) else 'FAIL', a.x, a.y, a.z, a.yaw))

    print('=== 2) 摆位后 %.0f s：每帧点名本车与最近的他模型 ===' % a.secs)
    t0 = time.time()
    last = None
    minpair = (99.0, '', 0.0)
    n_ev = 0
    while time.time() - t0 < a.secs:
        st = n.stamp
        n.wait(0.02)
        if n.stamp <= st or a.model not in n.snap:
            continue
        x, y, z, h = n.snap[a.model]
        near = min(((math.hypot(px - x, py - y), k)
                    for k, (px, py, _, _) in n.snap.items() if k != a.model),
                   default=(99.0, '-'))
        v = 0.0
        if last:
            dt = st - last[0]
            if dt > 1e-3:
                v = math.hypot(x - last[1], y - last[2]) / dt
        if v > 0.05:
            n_ev += 1
        if near[0] < minpair[0]:
            minpair = (near[0], near[1], time.time() - t0)
        if n_ev or v > 0.05 or time.time() - t0 < 3:
            print('  t=%5.2f (%7.3f,%7.3f) z=%5.2f yaw=%+.3f  |v|=%5.2f  '
                  '最近 %s@%.3f m' % (time.time() - t0, x, y, z, h, v,
                                      near[1], near[0]))
        last = (st, x, y, h)
    x, y, z, h = n.snap.get(a.model, (float('nan'),) * 4)
    print('  收尾 (%.3f,%.3f) yaw=%+.3f 离落点 %.2f m；显著运动帧 %d' % (
        x, y, h, math.hypot(x - a.x, y - a.y), n_ev))
    print('  最小他模型间距 %.3f m → %s（t=%.2f s）' % minpair)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
