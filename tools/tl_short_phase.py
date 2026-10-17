#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位录制文件里"异常短的相位段"，并打印它前后各两段事件原文，用于判断是时序缺陷还是读数/截断伪影。
   用法：python3 tl_short_phase.py <录制json> [信号组] [阈值秒=1.5]"""
import json
import sys

path = sys.argv[1]
grp = sys.argv[2] if len(sys.argv) > 2 else 'J1_ns'
thr = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5

ev = json.load(open(path, encoding='utf-8'))['events']
seq, last = [], None
for e in ev:
    if e.get('type') == 'vehicle' or e.get('group') != grp:
        continue
    v = e.get('vehicle_phase')
    if not v or v == last:
        continue
    last = v
    seq.append(e)

for i in range(1, len(seq)):
    dt = seq[i]['t'] - seq[i - 1]['t']
    if dt < thr and seq[i - 1]['vehicle_phase'] == 'YELLOW':
        print(f'  异常段：{seq[i-1]["vehicle_phase"]} 只持续 {dt:.2f} s（t={seq[i-1]["t"]:.2f} -> {seq[i]["t"]:.2f}）')
        for j in range(max(0, i - 3), min(len(seq), i + 2)):
            mark = '  >> ' if j == i - 1 else '     '
            print(mark + json.dumps(seq[j], ensure_ascii=False))
        print()
