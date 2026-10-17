#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车队"在动的辆数"守卫：读 signal_drive 的轨迹 CSV，按窗口比较每辆车的首末位置。

口径来自 视频取证/README.md §五："卡死时驱动节点会把某两辆车反复 teleport 回复位点，
10 s 位移能到 30 m，按最大值永远判成在动" —— 所以要数**几辆车在动**，并且把复位造成的
跳变单独列出来（跳变 >1.5 m/帧算 teleport，不计入"在动"）。
"""
import csv
import math
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
WINDOW = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0

rows = []
with open(CSV, encoding='utf-8', errors='replace') as f:
    for r in csv.reader(f):
        if len(r) < 6 or not r[1].startswith('car'):
            continue
        try:
            rows.append((float(r[0]), r[1], float(r[2]), float(r[3])))
        except ValueError:
            continue
if not rows:
    print('  CSV 里没有车辆行（节点刚起或路径不对）')
    print('MOVING=0')
    raise SystemExit(0)

t1 = rows[-1][0]
t0 = t1 - WINDOW
win = [r for r in rows if r[0] >= t0]
by = {}
for t, name, x, y in win:
    by.setdefault(name, []).append((t, x, y))

moved, jumped, quiet = [], [], []
for name, pts in sorted(by.items()):
    pts.sort()
    d = math.hypot(pts[-1][1] - pts[0][1], pts[-1][2] - pts[0][2])
    mj = 0.0
    for a, b in zip(pts, pts[1:]):
        mj = max(mj, math.hypot(b[1] - a[1], b[2] - a[2]))
    line = '%s 位移%.2f 最大帧跳%.2f' % (name, d, mj)
    if mj > 1.5:
        jumped.append(line)
    elif d > 0.15:
        moved.append(line)
    else:
        quiet.append(line)

print('  窗口 %.0f s，帧 %d 行：' % (WINDOW, len(win)))
for line in moved:
    print('    在动   ' + line)
for line in quiet:
    print('    停着   ' + line)
for line in jumped:
    print('    瞬移   ' + line + '（复位/弹射，不算在动）')
print('MOVING=%d' % len(moved))
