#!/usr/bin/env python3
# 从驱动节点自己写的轨迹 CSV 里量"车队实际贴在哪一条线上"，逐条车道给中位数。
#
# 为什么用 CSV 而不是现场抓 10 个点：CSV 是 signal_drive_node 每 tick 落的
# [t, 车名, x, y, yaw, 速度, 原因, 标签]，归档那份有 267 万行、跨好几个小时 ——
# 一次就能分清"车队一直贴 y≈-18.95 跑（慢性、世界里有挡件）"和"只有重启之后才这样
# （我这次摆位/重启引发的）"。
# 只统计**在走**的行（0.3<=速度<=1.2）：停着的行会把中位数拽偏，速度 >1.5 m/s 的行
# 是被复位弹射那一瞬（指令上限 0.70，弹射期读数无几何意义）。
# 用法：python3 tools/pk211_bandscan320.py [csv ...]
import csv
import statistics
import sys

DEFAULT = ['/tmp/signal_drive_path.pre_homeall.csv', '/tmp/signal_drive_path.csv']
# 车道取样窗口：(标签, 取哪个坐标做中位数, x 允许范围, y 允许范围)
BANDS = [
    ('南环西行（配置 y=-16.80）', 'y', (-21.0, 20.0), (-22.0, -14.0)),
    ('北环东行（配置 y=+1.50）', 'y', (-21.0, 21.0), (0.2, 2.8)),
    ('西环北行（配置 x=-21.80）', 'x', (-25.0, -19.0), (-16.0, 1.0)),
    ('东环南行（配置 x=+20.80）', 'x', (18.5, 25.0), (-16.0, 1.0)),
    ('中轴北行（配置 x=-0.80/+1.80）', 'x', (-3.0, 3.0), (-16.0, 1.0)),
]
V_LO, V_HI = 0.3, 1.2


def scan(path):
    acc = {tag: [] for tag, _k, _xr, _yr in BANDS}
    n_rows = n_bad = 0
    tmin = tmax = None
    with open(path, newline='', encoding='utf-8', errors='replace') as fh:
        for row in csv.reader(fh):
            if len(row) < 6:
                continue
            try:
                t = float(row[0])
                x = float(row[2])
                y = float(row[3])
                v = float(row[5])
            except ValueError:
                n_bad += 1          # 表头行
                continue
            n_rows += 1
            tmin = t if tmin is None else min(tmin, t)
            tmax = t if tmax is None else max(tmax, t)
            if not (V_LO <= v <= V_HI):
                continue
            for tag, key, (x0, x1), (y0, y1) in BANDS:
                if x0 <= x <= x1 and y0 <= y <= y1:
                    acc[tag].append(x if key == 'x' else y)
    print('  文件 %s' % path)
    print('    可用行 %d（跳过表头/坏行 %d），t 跨度 %.0f~%.0f s' % (n_rows, n_bad, tmin or 0, tmax or 0))
    for tag, _k, _xr, _yr in BANDS:
        vals = acc[tag]
        if len(vals) < 30:
            print('    %-32s 在走的样本只有 %d 行 —— 不下结论' % (tag, len(vals)))
            continue
        vals.sort()
        p = lambda q: vals[int(q * (len(vals) - 1))]  # noqa: E731 只用于分位展示
        print('    %-32s 样本 %6d  中位 %+7.2f  p10 %+7.2f  p90 %+7.2f  跨度 %+7.2f' % (
            tag, len(vals), statistics.median(vals), p(0.10), p(0.90), vals[-1] - vals[0]))


def main():
    for path in sys.argv[1:] or DEFAULT:
        try:
            scan(path)
        except OSError as exc:
            print('  文件 %s 读不到：%s' % (path, exc))
    return 0


if __name__ == '__main__':
    sys.exit(main())
