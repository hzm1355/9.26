#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""等某道闸的地感变成指定电平，然后立刻退出（取证脚本用它卡"杆下正有车"那一瞬）。

为什么需要它：条目 3 要演"落杆时杆下有车会被挡回"，而 car_6 在出口核验位停留的窗口
只有几秒（核验→识别→抬杆→驶离），驱动脚本里写 sleep 卡不准 —— 卡早了车还没到，
卡晚了杆已经抬起、占用已经释放，那条 OP_LOWER 就成了"正常落杆"，看画面的人读到的是
完全相反的事。订阅 BarrierState 等电平是唯一不靠猜时间的办法。

用法（取证脚本里通常和落杆指令串成一条，让 capq 把两段的输出都收进字幕）：
  python3 pk_wait_gate.py --gate gate_out --occupied --timeout 220 \
      && python3 parking_cli.py gate lower --gate gate_out
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from v2x_msgs.msg import BarrierState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gate', default='gate_out')
    ap.add_argument('--occupied', dest='want', action='store_true', default=None)
    ap.add_argument('--free', dest='want', action='store_false')
    ap.add_argument('--timeout', type=float, default=220.0)
    a = ap.parse_args()

    got = {}

    class Waiter(Node):
        def __init__(self):
            super().__init__('pk_wait_gate')
            # RELIABLE + 深队列：manager 转发的 gate_state 是 20 Hz 的常态流，
            # 队列浅了会在调度抖动时把"占用那一拍"整段滑过去。
            self.create_subscription(BarrierState, '/parking/gate_state', self.cb, 50)

        def cb(self, m):
            if m.gate_id == a.gate:
                got[a.gate] = m

    rclpy.init()
    n = Waiter()
    t0 = time.time()
    hit = None
    while rclpy.ok() and time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        m = got.get(a.gate)
        # 必须真的收到过这一道闸的消息：只收到 gate_in 时 got 里没有 gate_out，
        # 拿 None 去比电平会抛异常，或者更坏——把"没收到"读成"不是我要的电平"。
        if m is not None and m.loop_occupied == a.want:
            hit = m
            break
    n.destroy_node()
    rclpy.shutdown()
    if hit is None:
        print('%s 的 loop_occupied 在 %.0f s 内没变成 %s（收到过该闸消息：%s）' % (
            a.gate, a.timeout, a.want, bool(got)))
        sys.exit(1)
    print('命中：%s loop_occupied=%s 时刻=%.1f s 实测角=%.2f° 目标角=%.2f° 力矩=%.1f N·m' % (
        a.gate, hit.loop_occupied, time.time() - t0,
        __import__('math').degrees(hit.angle_rad),
        __import__('math').degrees(hit.target_rad), hit.effort_nm))
    sys.exit(0)


if __name__ == '__main__':
    main()
