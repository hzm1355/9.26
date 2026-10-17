#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 signal_drive_params.yaml 里的 start_idx 把车辆摆回各自车道。
用途：回放（player 会改写实体位姿）或现场被物理引擎搅乱之后恢复演示现场。
用法：python3 place_cars.py [配置文件] [z 高度]
注意：要在 signal_drive_node 停掉之后跑，否则驱动的 cmd_vel 会和这里的零速抢方向盘。
"""
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node

CFG = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml')
# world 的 spawn pose 写的是 7.0，但实测车辆静止在 6.70（list_models 读出来的）。
# 按 7.0 摆车等于每次摆完自由落体 0.3 m，落地瞬间 2.4 m/s 的垂直速度会被卡口
# 当成"通过速度"记进抓拍记录，速度扫参的实测列因此整体虚高。
Z = float(sys.argv[2]) if len(sys.argv) > 2 else 6.70
GAP = 5.0
# 摆位弹射（tools/place_eject_probe.py 隔离实验）：摆位时轮子还在受驱，差速控制器的
# 轮位误差会一次性释放，实测沿车头方向冲到 4~8.5 m/s 并飞出路网。零速刹轮后再摆、
# 摆完再压一段零速，同一落点实测 6 s 内 mv 恒 0.00。
BRAKE_S = 1.5
QUIET_S = 1.0


class Placer(Node):

    def __init__(self):
        super().__init__('place_cars')
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_entity_state 未就绪，继续等待')
        self.pubs = {}

    def hold(self, name, secs):
        """按 ~20 Hz 发零速 secs 秒，把轮位误差原地放掉，别留到摆位之后一次性释放"""
        pub = self.pubs.get(name)
        if pub is None:
            pub = self.pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        end = time.monotonic() + secs
        while time.monotonic() < end:
            pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    def put(self, name, x, y, yaw):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = Z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = 0.0
        req.state.twist.angular.z = 0.0
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = bool(fut.result() and fut.result().success)
        print(f'  {name:6s} -> ({x:6.2f},{y:6.2f}) yaw={yaw:+.2f}  {"OK" if ok else "失败"}')
        return (x, y)


def main():
    with open(CFG, encoding='utf-8') as f:
        cars = yaml.safe_load(f)['signal_drive']['cars']
    rclpy.init()
    node = Placer()
    placed = []
    for name in sorted(cars):
        cfg = cars[name]
        # drive:false 的车已经交给 parking_shuttle_node 分队驱动，它的"起点"是
        # parking_zones.yaml 里的航线首点（world 出生位），不是环路路线的 start_idx。
        # 这里照摆会把车从停车场瞬移回环路 —— 而摆位那一下必带 ~8 m/s 穿透回弹
        # （tools/traction_test.py 实测），回弹落点又不在 shuttle 的航线上，
        # seat_at 重新选路会把它引到一段物理上走不通的斜穿里。
        if not cfg.get('drive', True):
            print(f'  {name:6s} drive:false，摆位跳过（由 parking_shuttle 负责）')
            continue
        route = [tuple(p) for p in cfg['route']]
        idx = int(cfg.get('start_idx', 0)) % len(route)
        px, py = route[(idx - 1) % len(route)]
        tx, ty = route[idx]
        dx, dy = tx - px, ty - py
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        yaw = math.atan2(uy, ux)
        back = 3.0
        while any(math.hypot(tx - ux * back - qx, ty - uy * back - qy) < GAP
                  for qx, qy in placed) and back < L - 3.0:
            back += GAP
        x, y = tx - ux * back, ty - uy * back
        node.hold(name, BRAKE_S)
        placed.append(node.put(name, round(x, 2), round(y, 2), yaw))
        node.hold(name, QUIET_S)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
