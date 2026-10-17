#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采样 6 个信号组灯态 2 个周期，验证机动车/行人时序与反相/绿波联动"""
import time, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
rclpy.init()
n = Node('tl_probe')
cur, ped = {}, {}
for g in GROUPS:
    n.create_subscription(String, f'/traffic_light/{g}/state',
                          lambda m, g=g: cur.__setitem__(g, m.data), 10)
    n.create_subscription(String, f'/traffic_light/{g}/pedestrian',
                          lambda m, g=g: ped.__setitem__(g, m.data), 10)
th = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
th.start()
time.sleep(3)
print('时刻   ' + ' '.join(f'{g:>13s}' for g in GROUPS))
print('       ' + ' '.join(f'{"车/人":>13s}' for g in GROUPS))
t0 = time.time()
for k in range(24):
    row = []
    for g in GROUPS:
        row.append(f'{cur.get(g, "?")[:4]}/{ped.get(g, "?")[:5]}'.center(13))
    print(f'{time.time() - t0:5.0f}s ' + ' '.join(row), flush=True)
    time.sleep(2.0)
rclpy.shutdown()
