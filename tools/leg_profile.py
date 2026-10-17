#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 1 m 一档的纵向分箱，看某条腿上横向偏离的分布（判断"系统性跑偏"还是"转角落点"）。

用法（WSL，负数一律用 --opt=value 形式）：
  python3 tools/leg_profile.py --car=car_3 --axis=x --lo=-15 --hi=8 --band=0.5,4.0
  python3 tools/leg_profile.py --car=car_1 --axis=y --lo=-18 --hi=-15 --band=-2.0,2.0
axis=x：沿 x 分箱，统计 y 落在 [band0,band1] 的样本；axis=y 反之。
"""
import argparse
import csv
import sys
from collections import defaultdict


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='/mnt/d/desktop1/9.26.1/_path_77.csv')
    ap.add_argument('--car', default='car_3')
    ap.add_argument('--axis', default='x', choices=('x', 'y'))
    ap.add_argument('--lo', type=float, default=-15.0)
    ap.add_argument('--hi', type=float, default=8.0)
    ap.add_argument('--band', default='0.5,4.0')
    ap.add_argument('--step', type=float, default=1.0)
    a = ap.parse_args()

    b0, b1 = [float(v) for v in a.band.split(',')]
    cross = 'y' if a.axis == 'x' else 'x'
    bins = defaultdict(list)
    with open(a.csv, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r['car'] != a.car:
                continue
            try:
                along, off = float(r[a.axis]), float(r[cross])
            except (TypeError, ValueError):
                continue
            if not (a.lo <= along <= a.hi and b0 <= off <= b1):
                continue
            bins[int(along / a.step) * a.step].append((off, float(r['speed']),
                                                       r['limited_by']))
    if not bins:
        print('该区间内没有样本（区间写错，或这辆车根本没走过这里）')
        return 1
    print('%s  %s %g..%g  横向取 %s in [%g,%g]' % (a.car, a.axis, a.lo, a.hi, cross, b0, b1))
    print('%-8s %5s %7s %7s %7s %7s  %s' % ('bin', 'n', 'off_min', 'off_max',
                                            'off_avg', '|v|avg', '限速来源'))
    for k in sorted(bins):
        vs = bins[k]
        offs = [v[0] for v in vs]
        sps = [abs(v[1]) for v in vs]
        srcs = sorted({v[2] for v in vs if v[2]})
        print('%-8g %5d %7.2f %7.2f %7.2f %7.2f  %s' % (
            k, len(vs), min(offs), max(offs), sum(offs) / len(offs),
            sum(sps) / len(sps), ','.join(srcs)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
