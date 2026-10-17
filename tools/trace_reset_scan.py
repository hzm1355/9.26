#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从五车同录 CSV 里找"瞬移"（复位/弹射），并打前后若干格的指令与实测速度演变。

为什么不用 CSV 的 jump 列：位姿流 ~97 Hz 而 /clock 10 Hz，记录器按时钟格去重，
复位正好落在同一时钟格内的帧会被丢掉，jump 永远抓不到瞬移（实测 10.4 m 的跳变，
jump 记成 0.00）。所以这里直接用相邻保留行的 x,y 差来判。

用法（WSL）：
  python3 tools/trace_reset_scan.py --dir=/mnt/d/desktop1/9.26.1/_tr77 --min-jump=0.6
"""
import argparse
import csv
import os


def load(path):
    rows = []
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append([float(r[k]) for k in ('t', 'x', 'y', 'cmd_v', 'cmd_w',
                                                   'mv', 'mw', 'jump')])
            except (KeyError, TypeError, ValueError):
                continue
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/mnt/d/desktop1/9.26.1/_tr77')
    ap.add_argument('--cars', default='car_1,car_2,car_3,car_4,car_5')
    ap.add_argument('--min-jump', type=float, default=0.6)
    ap.add_argument('--post', type=int, default=25, help='瞬移后打印多少格')
    a = ap.parse_args()

    total = 0
    for c in [x.strip() for x in a.cars.split(',') if x.strip()]:
        p = os.path.join(a.dir, 'cmd_pose_%s.csv' % c)
        if not os.path.isfile(p):
            print('缺少 %s' % p)
            continue
        rows = load(p)
        print('\n===== %s（%d 格）=====' % (c, len(rows)))
        n = 0
        for i in range(1, len(rows)):
            d = ((rows[i][1] - rows[i - 1][1]) ** 2
                 + (rows[i][2] - rows[i - 1][2]) ** 2) ** 0.5
            if d < a.min_jump:
                continue
            n += 1
            dt = rows[i][0] - rows[i - 1][0] or 1e-3
            print('  t=%8.2f 瞬移 %.1f m：(%.1f,%.1f)->(%.1f,%.1f)  等效 %.1f m/s'
                  % (rows[i][0], d, rows[i - 1][1], rows[i - 1][2], rows[i][1], rows[i][2],
                     d / dt))
            lo = max(0, i - 3)
            print('     前：%s' % '  '.join(
                't%.2f cmd(%.2f,%.2f) mv=%.2f mw=%.2f' % (r[0], r[3], r[4], r[5], r[6])
                for r in rows[lo:i]))
            seg = rows[i:i + a.post]
            print('     后：mv %s' % ' '.join('%.2f' % r[5] for r in seg))
            print('         cmd %s' % ' '.join('%+.2f' % r[3] for r in seg))
            mx = max(r[5] for r in seg)
            print('         瞬移后 %0.1f s 内实测峰值 %.2f m/s（指令上限 0.70）'
                  % (seg[-1][0] - seg[0][0], mx))
        total += n
        if n == 0:
            print('  没有瞬移')
    print('\n瞬移合计 %d 次' % total)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
