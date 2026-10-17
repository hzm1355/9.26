#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位"停车场前面"的沙盘建筑实体。

为什么不用 /tmp/route_height_map.json：那张图只在**五车环线路线走廊带**内留格，
停车场这块空地根本没进图，拿它查会得出"周边没有建筑"的假绿。这里改用
tools/mesh_intrusion_audit.py 生成的全图栅格 /tmp/mesh_tall_cells.json
（0.5 m 格，覆盖 x -25..25 / y -21..21，只存高出路面 >=0.3 m 的格）。

聚类按**连通块**而不是材质：同一个材质会贴遍全城，按材质聚类会把整张图连成一块。

用法（WSL）：
  python3 tools/pk170_lotbld308.py [hmin=1.2] [离场上限m=20] [面积下限m2=2]
产物：STDOUT 建筑块表 + 停车场邻域俯视 ASCII 图。
"""
import json
import math
import os
import sys

import yaml

ROOT = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
LOT = (6.50, 16.50, -12.80, -7.00)          # 停车场占地（含东扩后的 P3）
CAR_L, CAR_W = 0.85, 0.50
MAP = '/tmp/mesh_tall_cells.json'


def rects_of(legs):
    """把正交航段扩成车体走廊矩形。"""
    out = []
    for p, q in legs:
        if abs(q[0] - p[0]) < 1e-9:
            out.append((p[0] - CAR_W, p[0] + CAR_W,
                        min(p[1], q[1]) - CAR_L, max(p[1], q[1]) + CAR_L))
        else:
            out.append((min(p[0], q[0]) - CAR_L, max(p[0], q[0]) + CAR_L,
                        p[1] - CAR_W, p[1] + CAR_W))
    return out


def gap(r, x0, x1, y0, y1):
    dx = max(0.0, x0 - r[1], r[0] - x1)
    dy = max(0.0, y0 - r[3], r[2] - y1)
    return math.hypot(dx, dy)


def main():
    hmin = float(sys.argv[1]) if len(sys.argv) > 1 else 1.2
    dnear = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    amin = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    with open(MAP, encoding='utf-8') as fh:
        mp = json.load(fh)
    cell, x0g, y0g, nc, nr = mp['cell'], mp['x0'], mp['y0'], mp['nc'], mp['nr']

    grid = {}
    for ix, iy, h, grp in mp['cells']:
        if h >= hmin and 0 <= ix < nc and 0 <= iy < nr:
            grid[(ix, iy)] = max(grid.get((ix, iy), 0.0), h)
    print('全图栅格 %s：格 %.2f m，建筑级(>=%.2f m)落地格 %d 个' % (MAP, cell, hmin, len(grid)))

    with open(os.path.join(ROOT, 'src/v2x_demo/config/parking_zones.yaml'),
              encoding='utf-8') as fh:
        z = yaml.safe_load(fh)['parking']['shuttle']['fleets']
    legs, wpts = [], []
    for mname, fleet in sorted(z.items()):
        for key, route in sorted(fleet.items()):
            if not key.endswith('_route'):
                continue
            r = [tuple(p[:2]) for p in route]
            wpts.extend((mname, key, p) for p in r)
            for j in range(1, len(r)):
                legs.append((r[j - 1], r[j]))
    corr = rects_of(legs)
    print('摆渡航线 %d 条腿 / 车体走廊 %d 段' % (len(legs), len(corr)))

    seen, comps = set(), []
    for start in grid:
        if start in seen:
            continue
        stack, blk = [start], []
        seen.add(start)
        while stack:
            ix, iy = stack.pop()
            blk.append((ix, iy))
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
                k = (ix + d[0], iy + d[1])
                if k in grid and k not in seen:
                    seen.add(k)
                    stack.append(k)
        comps.append(blk)

    rows = []
    for blk in comps:
        xs = [c[0] for c in blk]
        ys = [c[1] for c in blk]
        x0, x1 = x0g + min(xs) * cell, x0g + (max(xs) + 1) * cell
        y0, y1 = y0g + min(ys) * cell, y0g + (max(ys) + 1) * cell
        area = len(blk) * cell * cell
        if area < amin:
            continue
        dx = max(0.0, LOT[0] - x1, x0 - LOT[1])
        dy = max(0.0, LOT[2] - y1, y0 - LOT[3])
        dlot = math.hypot(dx, dy)
        if dlot > dnear:
            continue
        gcor = min((gap(c, x0, x1, y0, y1) for c in corr), default=99.0)
        rows.append(dict(dlot=dlot, x0=x0, x1=x1, y0=y0, y1=y1, area=area,
                         hmax=max(grid[c] for c in blk), gcor=gcor, cells=blk))
    rows.sort(key=lambda r: r['dlot'])

    print('\n== 停车场 %.0f m 内的沙盘建筑块（连通块，面积>=%.0f m²）==' % (dnear, amin))
    print('%-4s %-24s %8s %8s %8s %8s %7s %s' % ('块', 'x / y 范围', '离场m', '面积m²',
                                                  '最高m', '离航线m', '格数', '判定'))
    for i, r in enumerate(rows):
        print('B%-3d x %6.2f..%6.2f y %6.2f..%6.2f %7.2f %8.1f %7.2f %8.2f %5d  [%s]' % (
            i + 1, r['x0'], r['x1'], r['y0'], r['y1'], r['dlot'], r['area'], r['hmax'],
            r['gcor'], len(r['cells']),
            '压航线' if r['gcor'] <= 0.0 else ('贴边' if r['gcor'] <= 0.5 else '安全')))
    if not rows:
        print('（无）')

    # 俯视 ASCII 图：x 向右、y 向上；每列 1 格
    WX0, WX1 = 3.0, 20.0
    WY0, WY1 = -17.0, -4.0
    print('\n== 俯视 ASCII 图（x %.0f..%.0f / y %.0f..%.0f，每格 %.1f m；'
          '# 建筑  o 航线走廊  L 停车场界  + 闸机/岗亭/龙门架所在 x=9.4）==' % (
              WX0, WX1, WY0, WY1, cell))
    own = {}
    for i, r in enumerate(rows):
        for c in r['cells']:
            own[c] = str(i + 1) if i < 9 else '#'
    cset = set()
    for c0, c1, y0, y1 in corr:
        ix = int(round((c0 - x0g) / cell))
        jx = int(round((c1 - x0g) / cell))
        iy = int(round((y0 - y0g) / cell))
        jy = int(round((y1 - y0g) / cell))
        for a in range(ix, jx + 1):
            for b in range(iy, jy + 1):
                cset.add((a, b))
    header = '     ' + ''.join('%d' % (int(x) % 10) for x in
                               [WX0 + i * cell for i in range(int((WX1 - WX0) / cell))])
    print(header)
    for iy in range(int((WY1 - WY0) / cell) - 1, -1, -1):
        y = WY0 + iy * cell
        line = []
        for ix in range(int((WX1 - WX0) / cell)):
            x = WX0 + ix * cell
            k = (int(round((x - x0g) / cell)), int(round((y - y0g) / cell)))
            ch = ' '
            if k in own:
                ch = own[k]
            elif k in cset:
                ch = 'o'
            if abs(x - LOT[0]) < cell / 2 or abs(x - LOT[1]) < cell / 2:
                if ch == ' ' or ch == 'o':
                    ch = '|'
            if abs(y - LOT[2]) < cell / 2 or abs(y - LOT[3]) < cell / 2:
                if ch == ' ' or ch == 'o':
                    ch = '-'
            if abs(x - 9.4) < cell / 2:
                if ch == ' ' or ch == 'o':
                    ch = '+'
            line.append(ch)
        print('y%6.1f ' % y + ''.join(line))
    return 0


if __name__ == '__main__':
    sys.exit(main())
