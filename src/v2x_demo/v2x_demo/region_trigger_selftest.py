#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
区域触发底座合成数据自测（不需要 Gazebo）：
  - 同进程起 region_trigger_node + 合成 /model_states 发布器；
  - 主车 v2x_prius 沿 NS-main 从 y=-30 走到 y=0（z=0.4），
    应依次穿过 SPEED-NS-A / CKPT-NS-01 / SPEED-NS-B / ETC-01_zone；
  - 干扰车 v2x_decoy_viaduct 走同一路径但 z=7.4（高架高度），应零事件（验证 z 过滤）；
  - 静态地物 road_NS-main 不应被车辆正则匹配（验证过滤）；
  - 断言事件序列完全匹配 + 结束后区域占用状态为空闲（transient_local 快照）。
用法（WSL，source install/setup.bash 后）：
  python3 src/v2x_demo/v2x_demo/region_trigger_selftest.py
退出码 0=PASS 1=FAIL
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from v2x_msgs.msg import RegionEvent, RegionState

import region_trigger_node as rtn

STEP, DT = 0.15, 0.03          # 每 tick 位移 m / 周期 s（> min_process_interval）
Y0, Y1 = -30.0, 0.0
SPEED = STEP / DT

EXPECTED = [
    ('v2x_prius', 'SPEED-NS-A', RegionEvent.EVENT_ENTER),
    ('v2x_prius', 'SPEED-NS-A', RegionEvent.EVENT_EXIT),
    ('v2x_prius', 'CKPT-NS-01', RegionEvent.EVENT_ENTER),
    ('v2x_prius', 'CKPT-NS-01', RegionEvent.EVENT_EXIT),
    ('v2x_prius', 'SPEED-NS-B', RegionEvent.EVENT_ENTER),
    ('v2x_prius', 'SPEED-NS-B', RegionEvent.EVENT_EXIT),
    ('v2x_prius', 'ETC-01_zone', RegionEvent.EVENT_ENTER),
    ('v2x_prius', 'ETC-01_zone', RegionEvent.EVENT_EXIT),
]


class Driver(Node):
    def __init__(self):
        super().__init__('region_selftest_driver')
        self.events = []
        self.states = {}
        self.y = Y0
        self.drain = 0
        self.done = False
        self.pub = self.create_publisher(ModelStates, '/model_states', 10)
        self.create_subscription(RegionEvent, 'v2x/region_events', self.on_event, 100)
        for rid in ('ETC-01_zone', 'SPEED-NS-A'):
            self.create_subscription(
                RegionState, rtn.state_topic(rid),
                lambda m, r=rid: self.states.__setitem__(r, m), 10)
        self.create_timer(DT, self.tick)

    def on_event(self, msg):
        self.events.append((msg.vehicle_id, msg.region_id, msg.event_type, msg.speed))

    def tick(self):
        if self.y > Y1:
            self.drain += 1
            if self.drain > 30:
                self.done = True
            return
        msg = ModelStates()
        msg.name = ['road_NS-main', 'v2x_prius', 'v2x_decoy_viaduct']
        msg.pose = []
        msg.twist = []
        for name, z, moving in (('road_NS-main', 0.0, False),
                                ('v2x_prius', 0.4, True),
                                ('v2x_decoy_viaduct', 7.4, True)):
            p = Pose()
            p.position.x = 0.0
            p.position.y = self.y if moving else 0.0
            p.position.z = z
            p.orientation.w = 1.0
            t = Twist()
            if moving:
                t.linear.y = SPEED
            msg.pose.append(p)
            msg.twist.append(t)
        self.pub.publish(msg)
        self.y += STEP


def main():
    rclpy.init()
    trigger = rtn.RegionTriggerNode()
    driver = Driver()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(trigger)
    ex.add_node(driver)
    while not driver.done and rclpy.ok():
        ex.spin_once(timeout_sec=0.05)
    ex.spin_once(timeout_sec=0.2)

    fails = []
    actual = [(v, r, t) for v, r, t, _ in driver.events]
    if actual != EXPECTED:
        fails.append(f'事件序列不匹配:\n  期望 {EXPECTED}\n  实际 {actual}')
    decoy = [e for e in driver.events if e[0] == 'v2x_decoy_viaduct']
    if decoy:
        fails.append(f'高架干扰车误触发: {decoy}')
    speeds = [s for _, _, _, s in driver.events]
    if speeds and any(abs(s - SPEED) > 0.01 for s in speeds):
        fails.append(f'speed 字段异常: {speeds}')
    for rid in ('ETC-01_zone', 'SPEED-NS-A'):
        st = driver.states.get(rid)
        if st is None:
            fails.append(f'{rid} 状态话题无快照')
        elif st.occupied or st.vehicle_ids:
            fails.append(f'{rid} 结束后仍占用: {st.vehicle_ids}')

    for f in fails:
        print('FAIL:', f)
    if fails:
        print('SELFTEST FAIL')
        return 1
    print(f'SELFTEST PASS: {len(driver.events)} 个事件，序列/高架过滤/占用状态全部正确')
    return 0


if __name__ == '__main__':
    try:
        rc = main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)
