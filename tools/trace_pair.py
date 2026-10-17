#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五车同录对齐分析：找"实测超过指令上限"的那些格，看当时离最近的车有多远。

判据：差速盘最高只能跑出 v=0.70、ω=0.45，实测越界只可能来自外部接触。
  - 最近车 <= 1.2 m      → 车对车顶推（互锁被 ODE 解成绕着对方撬）
  - 最近车 > 2.0 m 且紧跟在复位之后 → 摆位弹射
  - 最近车 1.2~2.0 m     → 待定，需要看 fixture

用法（WSL）：
  python3 tools/trace_pair.py --dir=/mnt/d/desktop1/9.26.1/_tr77 --vpeak=0.90
"""
import argparse
import csv
import os


def load(path):
    rows = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append((float(r['t']), float(r['x']), float(r['y']),
                             float(r['cmd_v']), float(r['cmd_w']),
                             float(r['mv']), float(r['mw'])))
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/mnt/d/desktop1/9.26.1/_tr77')
    ap.add_argument('--cars', default='car_1,car_2,car_3,car_4,car_5')
    ap.add_argument('--vpeak', type=float, default=0.90, help='判定越界的实测线速')
    ap.add_argument('--wpeak', type=float, default=0.60, help='判定越界的实测角速')
    ap.add_argument('--top', type=int, default=25, help='最多列几段')
    a = ap.parse_args()

    data = {}
    for c in a.cars.split(','):
        p = os.path.join(a.dir, 'cmd_pose_%s.csv' % c)
        if not os.path.isfile(p):
            print('缺少 %s，跳过' % p)
            continue
        data[c] = load(p)
        print('%s：样本 %d' % (c, len(data[c])))
    if len(data) < 2:
        print('少于两辆车的数据，没法算最近车距')
        return 1

    # 按时间对齐：以第一辆车的时间轴为主，其它车用"最近的一格"取（10 Hz 同频，误差 <0.1 s）
    def near(rows, t):
        lo, hi = 0, len(rows) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if rows[mid][0] < t:
                lo = mid + 1
            else:
                hi = mid
        best = rows[lo]
        if lo > 0 and abs(rows[lo - 1][0] - t) < abs(best[0] - t):
            best = rows[lo - 1]
        return best if abs(best[0] - t) < 0.25 else None

    segs = []
    for c, rows in data.items():
        run = None
        for i, (t, x, y, cv, cw, mv, mw) in enumerate(rows):
            if mv > a.vpeak or mw > a.wpeak:
                if run is None:
                    run = [i, i, t, t, x, y, mv, mw, 99.0, '', 0]
                else:
                    run[1] = i
                    run[3] = t
                    if mv > run[6]:
                        run[6], run[4], run[5] = mv, x, y
                    if mw > run[7]:
                        run[7] = mw
                # 峰值那一格记最近车
                for o, orows in data.items():
                    if o == c:
                        continue
                    p = near(orows, t)
                    if p is None:
                        continue
                    d = ((p[1] - x) ** 2 + (p[2] - y) ** 2) ** 0.5
                    if d < run[8]:
                        run[8], run[9] = d, o
                        run[10] = i
            else:
                if run is not None and run[1] - run[0] + 1 >= 4:
                    segs.append((c,) + tuple(run))
                run = None
        if run is not None and run[1] - run[0] + 1 >= 4:
            segs.append((c,) + tuple(run))

    segs.sort(key=lambda s: -s[1])
    print('\n实测越界成段（>=4 格）共 %d 段，按峰值排序前 %d 段：' % (len(segs), a.top))
    print('%-6s %8s..%-8s %5s  %8s %7s %7s  %8s %6s  %8s %6s' % (
        '车', '起t', '止t', '格数', '峰值v', '峰ω', '位置', '最近车距', '是谁',
        '它当时v', '它ω'))
    for c, i0, i1, t0, t1, x, y, pv, pw, d, o, _k in segs[:a.top]:
        rows = data[c]
        # 取记录最近车那一格的时间
        tt = rows[min(_k, len(rows) - 1)][0]
        p = near(data[o], tt) if o else None
        print('%-6s %8.1f..%8.1f %5d  %8.2f %7.2f (%6.1f,%6.1f)  %8.2f %6s  %8.2f %6.2f' % (
            c, t0, t1, i1 - i0 + 1, pv, pw, x, y, d, o or '-',
            p[5] if p else -1, p[6] if p else -1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
