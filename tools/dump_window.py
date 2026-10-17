#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按时间窗把某辆车的轨迹逐行打出来（看越界是怎么一步步发生的）。

用法（WSL，负数一律用 --opt=value 形式）：
  python3 tools/dump_window.py --car=car_2 --t0=1575 --t1=1600
"""
import argparse
import csv


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='D:/desktop1/9.26.1/_path_77b.csv')
    ap.add_argument('--car', default='car_2')
    ap.add_argument('--t0', type=float, default=0.0)
    ap.add_argument('--t1', type=float, default=1e18)
    ap.add_argument('--step', type=float, default=0.5, help='最小打印间隔（秒），0=全打')
    a = ap.parse_args()

    last = -1e18
    n = 0
    with open(a.csv, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            if r.get('car') != a.car:
                continue
            try:
                t = float(r['t'])
            except (KeyError, TypeError, ValueError):
                continue
            if not (a.t0 <= t <= a.t1):
                continue
            if t - last < a.step:
                continue
            last = t
            n += 1
            print('t=%8.2f  x=%7.2f y=%7.2f yaw=%+6.2f v=%5.2f  %s' % (
                t, float(r['x']), float(r['y']), float(r['yaw']), float(r['speed']),
                r.get('limited_by') or '-'))
    print('--- 共 %d 行 ---' % n)
    return 0 if n else 1


if __name__ == '__main__':
    raise SystemExit(main())
