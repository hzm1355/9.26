#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""回读 signal_drive 的逐 tick 路径 CSV，判"车跑到出界"是连续开过去的还是被瞬移过去的。

复位风暴里每个"出界"都对应一个 3.5 s 内位移 7~8 m 的记录，而驱动最多只能给
cruise_speed=0.7 m/s。这两件事不可能同时为真，所以必须用逐 tick 轨迹把它们分开：
  - 连续大位移（相邻 tick 距离 ≈ v/rate）→ 速度真的高，指令或物理有问题；
  - 相邻 tick 出现米级跳变 → 中间发生过 /set_entity_state（复位本身），
    "出界"是复位后的落点或别的车造成的。

用法：python3 path_csv_report.py [csv] [每车打印前 N 行]
"""
import csv
import math
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
HEAD = int(sys.argv[2]) if len(sys.argv) > 2 else 12

rows = []
with open(PATH, encoding='utf-8-sig', newline='') as f:
    for r in csv.DictReader(f):
        try:
            rows.append((float(r['t']), r['car'], float(r['x']), float(r['y']),
                         float(r['yaw']), float(r['speed']), r['limited_by'], r['phase']))
        except (KeyError, ValueError):
            continue
if not rows:
    print('!! %s 没有可用行' % PATH)
    raise SystemExit(1)

by = {}
for r in rows:
    by.setdefault(r[1], []).append(r)

print('CSV %s：共 %d 行，时间跨度 %.1f s' % (PATH, len(rows), rows[-1][0] - rows[0][0]))
for name in sorted(by):
    rs = by[name]
    sp = [r[5] for r in rs]
    jumps = []
    for a, b in zip(rs, rs[1:]):
        d = math.hypot(b[2] - a[2], b[3] - a[3])
        dt = b[0] - a[0]
        if d > max(0.5, 3.0 * dt):        # 20 Hz 下相邻 tick 正常最多 0.035 m
            jumps.append((a[0], b[0], d, a[2], a[3], b[2], b[3]))
    print('\n  %s 行数 %d  speed 最小 %.2f / 中位 %.2f / 最大 %.2f m/s' % (
        name, len(rs), min(sp), sorted(sp)[len(sp) // 2], max(sp)))
    print('    首 %d 行 (t, x, y, speed, who)：' % min(HEAD, len(rs)))
    for r in rs[:HEAD]:
        print('      %7.2f  (%7.2f,%7.2f)  v=%.2f  %s' % (r[0], r[2], r[3], r[5], r[6] or '-'))
    if jumps:
        print('    !! 相邻 tick 跳变 %d 次（这些就是 /set_entity_state 瞬移，不是开车过去）：' % len(jumps))
        for j in jumps[:8]:
            print('      t %.2f→%.2f  跳 %.2f m  (%.2f,%.2f) → (%.2f,%.2f)' % j)
    else:
        print('    相邻 tick 无米级跳变 —— 位移是连续产生的')
