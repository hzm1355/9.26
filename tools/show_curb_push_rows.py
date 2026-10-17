#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看 curb_push CSV 里每个阶段的位置序列，找跳变发生在哪一帧。

用法：python3 tools/show_curb_push_rows.py [csv] [每阶段打印行数]
"""
import csv
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else '/mnt/d/desktop1/9.26.1/_curb_push_car_4.csv'
STEP = int(sys.argv[2]) if len(sys.argv) > 2 else 25

with open(PATH, newline='', encoding='utf-8-sig') as fh:
    rows = [r for r in csv.DictReader(fh)]

for phase in dict.fromkeys(r['phase'] for r in rows):
    sel = [r for r in rows if r['phase'] == phase]
    print('== %s  %d 行  t %.1f..%.1f ==' % (phase, len(sel), float(sel[0]['t_sim']),
                                             float(sel[-1]['t_sim'])))
    print('   idx  t_sim      x       y      |v|')
    for i in range(0, len(sel), STEP):
        r = sel[i]
        print('  %4d %8.1f %7.2f %7.2f %6.2f' % (
            i, float(r['t_sim']), float(r['x']), float(r['y']), abs(float(r['v']))))
    jumps = []
    for i in range(1, len(sel)):
        dt = float(sel[i]['t_sim']) - float(sel[i - 1]['t_sim'])
        dd = (float(sel[i]['x']) - float(sel[i - 1]['x']),
              float(sel[i]['y']) - float(sel[i - 1]['y']))
        if dt > 0 and (dd[0] ** 2 + dd[1] ** 2) ** 0.5 / dt > 1.5:
            jumps.append((i, float(sel[i]['t_sim']), dt, dd[0], dd[1]))
    print('   帧间 |v|>1.5 m/s 的跳变：%d 处' % len(jumps))
    for j, t, dt, dx, dy in jumps[:8]:
        print('     idx %d t=%.1f dt=%.3f dx=%+.2f dy=%+.2f' % (j, t, dt, dx, dy))
