#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从识别记录 JSONL 里统计"某辆车两次被卡口拍到的间隔"，用来定等待预算。
用法：python3 plate_pass_interval.py [car_1 ...]
"""
import json
import sys
import time
from collections import defaultdict

path = '/tmp/v2x_records/plate_records.jsonl'
want = sys.argv[1:] or ['car_1', 'car_2', 'car_3', 'car_4', 'car_5']
ts = defaultdict(list)
n = 0
for line in open(path, encoding='utf-8'):
    line = line.strip()
    if not line:
        continue
    try:
        r = json.loads(line)
    except ValueError:
        continue
    n += 1
    v = r.get('vehicle_id') or ''
    # 用记录里的墙钟秒做间隔：sim_ts 会随 Gazebo 重启回退，不能跨场次相减
    try:
        t = time.mktime(time.strptime(r['ts'], '%Y-%m-%d %H:%M:%S'))
    except (KeyError, ValueError):
        continue
    if v in want:
        ts[v].append(float(t))

print('记录总行数 %d，最近一次 %.1f s 前' % (
    n, time.time() - max((x for a in ts.values() for x in a), default=0)))
for v in want:
    a = sorted(set(ts.get(v, [])))
    if len(a) < 2:
        print('%-6s 只有 %d 次抓拍，无法算间隔' % (v, len(a)))
        continue
    g = [b - c for b, c in zip(a[1:], a[:-1])]
    g = [x for x in g if 0.1 < x < 4000]
    g.sort()
    print('%-6s 抓拍 %3d 次  间隔 min %.0f / 中位 %.0f / p90 %.0f / max %.0f s' % (
        v, len(a), g[0], g[len(g) // 2], g[int(len(g) * 0.9)] if g else 0, g[-1]))
