#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在落地障碍高度图上查一个矩形：里面有没有东西、离车道走廊多远。

用途：挪停车场/卡口/任何要占路面的设施之前，先确认新位置不是"看着有空、实际压在
景观构筑物上"。判据与 route_corridor_scan.py 完全一致（同一张 /tmp/route_height_map.json、
同一"落地"口径），所以这里的结论和穿楼体检不会因为两套算法而互相矛盾。

用法（WSL，仓库根目录，需先跑过 `python3 tools/route_corridor_scan.py --rebuild`）：
  python3 tools/lot_clearance.py --rect=5.80,13.00,-13.80,-5.50
  python3 tools/lot_clearance.py --rect=... --hmin=0.45
"""
import json
import sys

MAP = '/tmp/route_height_map.json'


def opt(name, default, cast=str):
    for a in sys.argv[1:]:
        if a.startswith('--' + name + '='):
            return cast(a.split('=', 1)[1])
    return default


def main():
    rect = [float(v) for v in opt('rect', '', str).split(',')]
    if len(rect) != 4:
        sys.stderr.write('必须给 --rect=x0,x1,y0,y1\n')
        return 2
    x0, x1, y0, y1 = rect
    hmin = float(opt('hmin', 0.45, float))
    with open(MAP, encoding='utf-8') as fh:
        mp = json.load(fh)
    cell, gx0, gy0 = mp['cell'], mp['x0'], mp['y0']
    hit = []
    for ix, iy, h, grp in mp['cells']:
        if h < hmin:
            continue
        cx, cy = gx0 + (ix + 0.5) * cell, gy0 + (iy + 0.5) * cell
        if x0 <= cx <= x1 and y0 <= cy <= y1:
            hit.append((cx, cy, h, grp))
    n = int((x1 - x0) / cell) * int((y1 - y0) / cell)
    print('矩形 x %.2f..%.2f y %.2f..%.2f，格 %.2f m 共约 %d 格' % (x0, x1, y0, y1, cell, n))
    if not hit:
        print('  落地障碍（h>=%.2f m）：0 格 —— 这块地面是空的，可以摆设施' % hmin)
        return 0
    print('  落地障碍（h>=%.2f m）：%d 格，最高 %.2f m' % (
        hmin, len(hit), max(p[2] for p in hit)))
    by_grp = {}
    for cx, cy, h, grp in hit:
        g = by_grp.setdefault(grp or '(无名)', [1e9, -1e9, 1e9, -1e9, 0])
        g[0] = min(g[0], cx)
        g[1] = max(g[1], cx + cell)
        g[2] = min(g[2], cy)
        g[3] = max(g[3], cy + cell)
        g[4] += 1
    for grp, g in sorted(by_grp.items(), key=lambda kv: -kv[1][4]):
        print('    %-28s %4d 格  x %.2f..%.2f y %.2f..%.2f' % (grp, g[4], g[0], g[1], g[2], g[3]))
    return 1


if __name__ == '__main__':
    sys.exit(main())
