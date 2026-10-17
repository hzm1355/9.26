#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次订阅 6 个信号组的 state_detail / stats，打印每个组"最新一帧"的生效配时。
   取代逐个 `ros2 topic echo --once`（会拿到过期/半帧数据，取证数值不可信）。"""
import sys

import rclpy
from rclpy.node import Node

from v2x_msgs.msg import TrafficLightState, TrafficLightStats

GRPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
WHAT = sys.argv[2] if len(sys.argv) > 2 else 'detail'      # detail | stats | both


class Probe(Node):
    def __init__(self):
        super().__init__('tl_detail_probe')
        self.det, self.st = {}, {}
        for g in GRPS:
            if WHAT in ('detail', 'both'):
                self.create_subscription(
                    TrafficLightState, f'/traffic_light/{g}/state_detail',
                    lambda m, gid=g: self.on_det(gid, m), 10)
            if WHAT in ('stats', 'both'):
                self.create_subscription(
                    TrafficLightStats, f'/traffic_light/{g}/stats',
                    lambda m, gid=g: self.on_stats(gid, m), 10)

    def on_det(self, gid, m):
        self.det[gid] = m

    def on_stats(self, gid, m):
        self.st[gid] = m


def main():
    rclpy.init()
    n = Probe()
    import time
    t0 = time.time()
    while time.time() - t0 < SECS and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)

    if WHAT in ('detail', 'both'):
        print('  组别     车灯      剩余    行人灯      行人剩余   G      Y     R     模式')
        for g in GRPS:
            m = n.det.get(g)
            if m is None:
                print(f'  {g:<8} (无数据)')
                continue
            print(f'  {g:<8} {m.vehicle_phase:<8} {m.phase_remaining:6.2f}  '
                  f'{m.pedestrian_phase:<10} {m.pedestrian_remaining:7.2f}  '
                  f'{m.green_duration:5.1f} {m.yellow_duration:4.1f} {m.red_duration:5.1f}  '
                  f'{m.control_mode}')
    if WHAT in ('stats', 'both'):
        print('  组别     排队  通过(累计)  平均等待   最大等待  效率(辆/分) 当前绿   系数  周期数')
        for g in GRPS:
            m = n.st.get(g)
            if m is None:
                print(f'  {g:<8} (无数据)')
                continue
            # 口径提醒：vehicle_count 与 total_vehicles 在管理器里同增同不清零
            # （traffic_light_manager.py 里没有任何一处把它归零），所以这两列恒等，
            # 是**累计**通过量而不是"本周期通过"。avg/max wait 同样是全程口径。
            print(f'  {g:<8} {m.queue_length:5d} {m.total_vehicles:9d} {m.avg_wait_time:9.2f} '
                  f'{m.max_wait_time:9.2f} {m.throughput:10.2f} '
                  f'{m.current_green_duration:8.2f} {m.adaptation_factor:6.2f} {m.cycle_count:6d}')
    n.destroy_node()
    rclpy.shutdown()


main()
