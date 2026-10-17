#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 /tmp/signal_drive_path.csv：统计红灯停车事件、排队事件、绿灯放行"""
import csv
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
rows = []
with open(path, encoding='utf-8') as f:
    for r in csv.DictReader(f):
        try:
            r['t'] = float(r['t']); r['speed'] = float(r['speed'])
            r['x'] = float(r['x']); r['y'] = float(r['y'])
        except ValueError:
            continue
        rows.append(r)

cars = defaultdict(list)
for r in rows:
    cars[r['car']].append(r)

for name, rs in sorted(cars.items()):
    print(f'\n===== {name}  样本 {len(rs)} 条, 时长 {rs[-1]["t"] - rs[0]["t"]:.0f}s =====')
    eps = []
    cur = None
    for r in rs:
        sig = r['limited_by'] in ('J1_ns', 'J1_ew', 'J2_ns')
        stopped = abs(r['speed']) < 0.06
        if sig and stopped:
            if cur and cur['group'] == r['limited_by'] and r['t'] - cur['end'] < 3.0:
                cur['end'] = r['t']
                cur['n'] += 1
            else:
                if cur:
                    eps.append(cur)
                cur = {'group': r['limited_by'], 'phase': r['phase'], 'start': r['t'],
                       'end': r['t'], 'x': r['x'], 'y': r['y'], 'n': 1}
        else:
            if cur and (r['t'] - cur['end'] > 3.0 or not stopped):
                eps.append(cur)
                cur = None
    if cur:
        eps.append(cur)
    red = [e for e in eps if e['phase'] == 'RED']
    yel = [e for e in eps if e['phase'] == 'YELLOW']
    print(f'  信号灯停车事件: {len(eps)} 次（红灯 {len(red)}，黄灯 {len(yel)}）')
    for e in eps:
        print(f'    {e["start"]:6.1f}s → {e["end"]:6.1f}s  停 {e["end"]-e["start"]:4.1f}s  '
              f'{e["group"]:6s} {e["phase"]:6s} 位置({e["x"]:.2f},{e["y"]:.2f})')
    # 放行：停车结束后 5s 内位移
    for e in eps:
        after = [r for r in rs if e['end'] < r['t'] <= e['end'] + 8.0]
        if after:
            d = ((after[-1]['x'] - e['x'])**2 + (after[-1]['y'] - e['y'])**2) ** 0.5
            print(f'    {e["start"]:6.1f}s 停车后 8s 内位移 {d:4.1f} m（绿灯放行）')
    q = [r for r in rs if r['limited_by'].startswith('follow:') and abs(r['speed']) < 0.06]
    print(f'  排队等候（被前车拦住且静止）: {len(q)} 个采样点')
    if q:
        b = q[0]
        print(f'    例：t={b["t"]:.1f}s 位置({b["x"]:.2f},{b["y"]:.2f}) 前车={b["limited_by"]}')
