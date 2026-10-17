#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""导出某辆车在一段仿真时间内的轨迹（默认每 0.5 s 一行），用于判断"是自己转向偏出去
的还是被物理推出去的"：自己转的 yaw 会连续地离开航向，被推的 yaw 不动而 x/y 侧移。

用法（WSL，负数一律 --opt=value）：
  python3 tools/trace_window.py --car=car_3 --t0=348 --t1=382 --step=0.5
"""
import argparse
import csv
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='/mnt/d/desktop1/9.26.1/_path_77.csv')
    ap.add_argument('--car', default='car_3')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e9)
    ap.add_argument('--step', type=float, default=0.5)
    a = ap.parse_args()
    nxt = a.t0
    prev = None
    print('%6s %8s %8s %7s %7s %6s  %s' % ('t', 'x', 'y', 'yaw', 'speed', 'dt', '限速来源'))
    with open(a.csv, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r['car'] != a.car:
                continue
            t = float(r['t'])
            if t < a.t0 or t > a.t1:
                continue
            if t + 1e-9 < nxt:
                continue
            nxt = t + a.step
            x, y = float(r['x']), float(r['y'])
            sp = float(r['speed'])
            jump = '' if prev is None else '  <-- 跳变 %.2f m' % (
                ((x - prev[0]) ** 2 + (y - prev[1]) ** 2) ** 0.5)
            print('%6.1f %8.2f %8.2f %7.2f %7.2f %6s  %s%s' % (
                t, x, y, float(r['yaw']), sp, '', r['limited_by'], jump))
            prev = (x, y)
    return 0


if __name__ == '__main__':
    sys.exit(main())
