#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出与某条航段（线段）横向距离 < R 的所有静态盒，按沿线投影排序。

用途：给活体复位触发找一个"车头正前方就顶死"的点 —— 车在自己的车道线上、航向也对，
但前方零点几米就是一个不可移动的静态盒，驱动节点会持续给油却不动，
于是"楔住 → 倒车脱困×2 → 复位"这条链必然走完。

数据结构沿用 tools/coll_near.py：json['boxes'][i] = {name, coll, link, static, aabb, lane_dist}。
用法（WSL，负数一律用位置参数或 -- 形式，这里用位置参数）：
  python3 tools/leg_boxes.py 3.0 1.8 -16.8 14.5 -16.8
"""
import json
import math
import sys

R = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
AX = float(sys.argv[2]) if len(sys.argv) > 2 else 1.8
AY = float(sys.argv[3]) if len(sys.argv) > 3 else -16.8
BX = float(sys.argv[4]) if len(sys.argv) > 4 else 14.5
BY = float(sys.argv[5]) if len(sys.argv) > 5 else -16.8
SRC = sys.argv[6] if len(sys.argv) > 6 else '/tmp/world_collisions.json'

dx, dy = BX - AX, BY - AY
L = math.hypot(dx, dy) or 1.0
ux, uy = dx / L, dy / L
print(f'航段 ({AX},{AY}) -> ({BX},{BY}) 长 {L:.1f} m，取横向 |lat|<{R} m 且 z 带相交的静态盒')

with open(SRC, encoding='utf-8') as f:
    boxes = json.load(f)['boxes']

n = 0
for b in boxes:
    if not b['static']:
        continue
    x0, x1, y0, y1, z0, z1 = b['aabb']
    if not (z0 < 7.6 and z1 > 6.6):
        continue
    lat_list, a_list = [], []
    for cx in (x0, x1):
        for cy in (y0, y1):
            rx, ry = cx - AX, cy - AY
            a_list.append(rx * ux + ry * uy)
            lat_list.append(-rx * uy + ry * ux)
    lat_lo, lat_hi = min(lat_list), max(lat_list)
    a_lo, a_hi = min(a_list), max(a_list)
    if a_hi >= -0.5 and a_lo <= L + 0.5 and lat_lo <= R and lat_hi >= -R:
        n += 1
        print(f'  {b["name"]}/{b["coll"] or b["link"]:<16} x[{x0:6.2f},{x1:6.2f}] '
              f'y[{y0:6.2f},{y1:6.2f}] z[{z0:.2f},{z1:.2f}] '
              f'沿线 {a_lo:6.2f}..{a_hi:6.2f} m  横向 {lat_lo:+6.2f}..{lat_hi:+6.2f} m  '
              f'lane={b.get("lane_dist", "-")}')
print(f'合计 {n} 个')
