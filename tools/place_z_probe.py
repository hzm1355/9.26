#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判别实验：摆位弹射是不是"摆放高度把车轮压进路面网格"造成的穿透回弹。

背景（本轮取证连输 6 轮的根因）：任何一次 /set_entity_state 摆位之后，车都会沿
**自己的车头方向**被推走 15~18 m，期间 z 全程不变、cmd_vel 发布者数为 0、odom 与位姿
微分一致（why_move_probe.py 已排除"有人在发速度"）。z 不变 + 沿车头水平弹 = 接触求解器
在做穿透修正，而不是重力/推力。那么唯一还能压进路面的量就是摆放 z。

做法：同一坐标、同一航向，只换 z，每个 z 摆一次，摆后用 10 Hz 采 4 s，统计
  - 平面位移（首末 + 最大）
  - 位移方向与车头的夹角（沿车头 = 穿透回弹的签名）
  - z 的最低值（如果摆下去先往下掉，说明原本 z 是悬空的，不该报警）
判据：如果 6.70 弹而 6.85/7.00 不弹，那么根因就是摆位高度，place_cars/recover/驱动
复位三条路径都要跟着改；如果全都弹，说明路面网格本身在该点有凸起，得换落点。

用法：python3 place_z_probe.py --car car_4 --x 16.80 --y -11.60 --yaw -1.5708 \
        --zs 6.70,6.85,7.00,7.20 [--gap 8]
必须先把 signal_drive_node 杀掉，否则它的复位会污染每一格。
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


class Probe(Node):
    def __init__(self):
        super().__init__('place_z_probe')
        self.pose = None                  # (x, y, z, yaw)
        self.trace = []
        self.everything = {}              # 全部实体名 -> (x, y, z)，用来查落点附近有什么
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(ModelStates, '/model_states', self.on, 20)

    def on(self, msg):
        for n, p in zip(msg.name, msg.pose):
            self.everything[n] = (p.position.x, p.position.y, p.position.z)
            if n == CAR:
                self.pose = (p.position.x, p.position.y, p.position.z,
                             yaw_of(p.orientation))
                self.trace.append((time.monotonic(), p.position.x, p.position.y,
                                   p.position.z))

    def near(self, x, y, radius=6.0):
        """落点 6 m 内的其它实体（含别车）：弹射如果是车-车/车-杆重叠，这里就能看到。"""
        out = [(n, math.hypot(p[0] - x, p[1] - y)) for n, p in self.everything.items()
               if n != CAR and n not in ('ground_plane', 'room_33')
               and math.hypot(p[0] - x, p[1] - y) < radius]
        return sorted(out, key=lambda t: t[1])

    def wait_data(self, budget=30.0):
        t0 = time.monotonic()
        while self.pose is None and time.monotonic() - t0 < budget:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose is not None

    def place(self, x, y, z, yaw):
        req = SetEntityState.Request()
        req.state.name = CAR
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        req.state.reference_frame = 'world'
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit('!! /set_entity_state 没服务')
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return bool(fut.result() and fut.result().success)

    def sample(self, sec):
        self.trace = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    global CAR
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', default='car_4')
    ap.add_argument('--x', type=float, default=16.80)
    ap.add_argument('--y', type=float, default=-11.60)
    ap.add_argument('--yaw', type=float, default=-1.5708)
    ap.add_argument('--zs', default='6.70,6.85,7.00,7.20')
    ap.add_argument('--gap', type=float, default=8.0, help='两格之间静置秒数')
    ap.add_argument('--obs', type=float, default=4.0, help='摆后观察秒数')
    a = ap.parse_args()
    CAR = a.car

    rclpy.init()
    nd = Probe()
    if not nd.wait_data():
        raise SystemExit('!! 30 s 内没收到 %s 的 /model_states' % CAR)
    print('%s 起始位姿 (%.2f,%.2f) z=%.2f yaw=%+.1f°' % (
        CAR, nd.pose[0], nd.pose[1], nd.pose[2], math.degrees(nd.pose[3])))

    hdr = '%-6s %8s %8s %8s %8s   %s' % ('z', '落点', '最大位移', '末位移', '末z', '签名')
    print('\n' + hdr)
    for z in [float(t) for t in a.zs.split(',')]:
        ok = nd.place(a.x, a.y, z, a.yaw)
        nd.sample(a.obs)
        if not nd.trace:
            print('%-6.2f 没有位姿样本（服务回执 %s）' % (z, ok))
            continue
        x0, y0 = nd.trace[0][1], nd.trace[0][2]
        xf, yf = nd.trace[-1][1], nd.trace[-1][2]
        dmax, dang = 0.0, 0.0
        for _, x, y, _ in nd.trace:
            d = math.hypot(x - x0, y - y0)
            if d > dmax:
                dmax = d
                dang = math.degrees(math.atan2(y - y0, x - x0))
        dend = math.hypot(xf - x0, yf - y0)
        zmin = min(t[3] for t in nd.trace)
        # 位移方向与车头夹角的绝对值：沿车头（<20°）就是穿透回弹的签名
        rel = abs((dang - math.degrees(a.yaw) + 180.0) % 360.0 - 180.0)
        tag = '沿车头弹射' if (dmax > 1.0 and rel < 25.0) else (
            '滑动/其它方向' if dmax > 1.0 else '稳住了')
        print('%-6.2f %8.2f %8.2f %8.2f %8.2f   %s（位移方向 %+.0f°，与车头差 %.0f°）' % (
            z, math.hypot(x0 - a.x, y0 - a.y), dmax, dend, zmin, tag, dang, rel))
        ns = nd.near(x0, y0)
        print('       落点 6 m 内实体：%s' % (', '.join(
            '%s %.1f m' % t for t in ns) if ns else '（空旷，无车无杆无卡口）'))
        nd.place(a.x, a.y, z, a.yaw)       # 摆回原处再静置，别把上一格的余速带进下一格
        nd.sample(a.gap)
    nd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
