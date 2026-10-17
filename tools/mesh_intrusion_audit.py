#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 room_33 沙盘网格里"高出路面 >=0.3 m"的部分栅格化成高度图，再拿路线去撞它。

为什么需要它：tools/gen_fixture_collisions.py 只在"车道中心线 ±3.5 m"的环带里补等效盒，
而且 ±2.0 m 禁入带内一律不补（防止长出压在护栏上的隐形墙）。这个前提意味着
**路线本身画进建筑时，那条路上永远不会有盒** —— 车是照着路线穿楼的，补盒补不住。
所以要先量化"路线到底压在哪几块建筑上"，改路线，再重生成盒。

几何换算与 gen_fixture_collisions.py 完全一致：
  模型 pose = 0 0 6.7 1.5708 0 0，mesh scale = 10
  → wx = 10*x，wy = -10*z，高出路面 h = 10*y

用法（WSL，仓库根目录）：
  python3 tools/mesh_intrusion_audit.py            # 建图（缓存）+ 路线体检 + 出 PNG
  python3 tools/mesh_intrusion_audit.py --rebuild   # 强制重扫 OBJ
产物：
  /tmp/mesh_tall_cells.json   高度图缓存（0.5 m 栅格，只存 >=0.3 m 的格）
  /tmp/mesh_intrusion.txt     路线穿楼路段表
  D:/desktop1/9.26.1/_mesh_map.png  俯视标注图（建筑灰阶 + 五车路线 + 穿楼段标红）
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.obj')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')
MAP = '/tmp/mesh_tall_cells.json'
OUT = '/tmp/mesh_intrusion.txt'
PNG = '/mnt/d/desktop1/9.26.1/_mesh_map.png'

S = 10.0
CELL = 0.5
HMIN = 0.3
X0, X1 = -25.0, 25.0
Y0, Y1 = -21.0, 21.0
NC = int((X1 - X0) / CELL)
NR = int((Y1 - Y0) / CELL)
PX = 6                        # 每格 6 像素 -> 0.5 m = 6 px
CAR_LEN_HALF = 0.85           # 车体 1.6 m + 一点余量
CAR_WIDE_HALF = 0.5           # 车宽 0.8 m + 余量

COLORS = {'car_1': (60, 180, 255), 'car_2': (255, 120, 60), 'car_3': (200, 80, 200),
          'car_4': (80, 220, 80), 'car_5': (0, 255, 255)}


def point_in_tri(px, py, ax, ay, bx, by, cx, cy):
    d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
    d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
    return (d1 >= 0 and d2 >= 0 and d3 >= 0) or (d1 <= 0 and d2 <= 0 and d3 <= 0)


def seg_tri(ax, ay, bx, by, cx, cy, rx0, ry0, rx1, ry1):
    for (px_, py_) in ((ax, ay), (bx, by), (cx, cy)):
        if rx0 <= px_ <= rx1 and ry0 <= py_ <= ry1:
            return True
    for (px_, py_) in ((rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1)):
        if point_in_tri(px_, py_, ax, ay, bx, by, cx, cy):
            return True
    return False


def build():
    vx, vy, vz = [], [], []
    cells = {}
    cur = '?'
    ntri = 0
    skipped = 0
    t0 = time.time()
    with open(OBJ, 'rb', buffering=1 << 22) as fh:
        for raw in fh:
            if not raw:
                continue
            c0 = raw[0]
            if c0 == 118 and raw[1:2] == b' ':
                p = raw[2:].split()
                vx.append(float(p[0]))
                vy.append(float(p[1]))
                vz.append(float(p[2]))
            elif c0 == 102:
                if not vx:
                    continue
                idx = []
                for tok in raw[1:].split():
                    if tok[:1] == b'/':
                        continue
                    s = tok.split(b'/')[0]
                    try:
                        i = int(s)
                    except ValueError:
                        continue
                    idx.append(len(vx) + i + 1 if i < 0 else i)
                if len(idx) < 3:
                    continue
                a = idx[0] - 1
                for j in range(1, len(idx) - 1):
                    b, c = idx[j] - 1, idx[j + 1] - 1
                    ntri += 1
                    ht = S * max(vy[a], vy[b], vy[c])
                    if ht < HMIN:
                        # 贴地涂装/路面，扫过就丢，这是最大的一个提速点
                        skipped += 1
                        continue
                    if (ntri & 0x3FFFF) == 0:
                        sys.stderr.write('  高面 %d  格 %d  %.0fs\n' % (
                            ntri, len(cells), time.time() - t0))
                        sys.stderr.flush()
                    axw, bxw, cxw = S * vx[a], S * vx[b], S * vx[c]
                    ayw, byw, cyw = -S * vz[a], -S * vz[b], -S * vz[c]
                    x0 = min(axw, bxw, cxw)
                    x1 = max(axw, bxw, cxw)
                    y0 = min(ayw, byw, cyw)
                    y1 = max(ayw, byw, cyw)
                    i0 = max(0, int((x0 - X0) / CELL))
                    i1 = min(NC - 1, int((x1 - X0) / CELL))
                    j0 = max(0, int((y0 - Y0) / CELL))
                    j1 = min(NR - 1, int((y1 - Y0) / CELL))
                    ncell = (i1 - i0 + 1) * (j1 - j0 + 1)
                    for iy in range(j0, j1 + 1):
                        ry0 = Y0 + iy * CELL
                        ry1 = ry0 + CELL
                        for ix in range(i0, i1 + 1):
                            rx0 = X0 + ix * CELL
                            rx1 = rx0 + CELL
                            # 大面（整片屋面/楼板）用格心包含测试，细长墙仍用相交测试：
                            # 墙的投影可能窄到盖不住任何格心，只判格心就会漏成一条虚线。
                            hit = (point_in_tri(rx0 + CELL / 2, ry0 + CELL / 2,
                                                axw, ayw, bxw, byw, cxw, cyw)
                                   if ncell > 100 else
                                   seg_tri(axw, ayw, bxw, byw, cxw, cyw,
                                           rx0, ry0, rx1, ry1))
                            if hit:
                                k = (ix, iy)
                                old = cells.get(k)
                                if old is None or ht > old[0]:
                                    cells[k] = (ht, cur)
            elif c0 == 117 or c0 == 103:
                sp = raw.split(None, 1)
                if len(sp) > 1:
                    cur = sp[1].decode('utf-8', 'replace').strip()
    sys.stderr.write('  扫完：面 %d（贴地丢弃 %d）、高格 %d、%.0fs\n' % (
        ntri, skipped, len(cells), time.time() - t0))
    out = {'cell': CELL, 'x0': X0, 'y0': Y0, 'nc': NC, 'nr': NR,
           'hmin': HMIN,
           'cells': [[k[0], k[1], round(v[0], 2), v[1]]
                     for k, v in sorted(cells.items())]}
    with open(MAP, 'w', encoding='utf-8') as fh:
        json.dump(out, fh)
    return out


def load(force=False):
    if not force and os.path.exists(MAP):
        with open(MAP, encoding='utf-8') as fh:
            return json.load(fh)
    return build()


def grid(m):
    g = np.zeros((m['nr'], m['nc']), dtype=np.float32)
    names = {}
    for ix, iy, h, mat in m['cells']:
        g[iy, ix] = max(g[iy, ix], h)
        names[(ix, iy)] = mat
    return g, names


def tall_in(g, x0, x1, y0, y1):
    """矩形内最高障碍（世界坐标，自动排序并夹到图内）。"""
    a, b = sorted((x0, x1))
    c, d = sorted((y0, y1))
    i0 = max(0, int((a - X0) / CELL))
    i1 = min(g.shape[1] - 1, int((b - X0) / CELL))
    j0 = max(0, int((c - Y0) / CELL))
    j1 = min(g.shape[0] - 1, int((d - Y0) / CELL))
    if i0 > i1 or j0 > j1:
        return 0.0
    return float(g[j0:j1 + 1, i0:i1 + 1].max())


def audit_routes(g, names):
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    lines = []
    hits_all = []
    for name in sorted(cars):
        route = [tuple(p[:2]) for p in cars[name]['route']]
        lines.append('== %s  航点 %d 个  %s' % (name, len(route), route))
        nseg_hit = 0
        for s in range(len(route)):
            p, q = route[s - 1], route[s]
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            if L < 1e-6:
                continue
            ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
            # 车体是沿航段方向 1.6 m、横向 0.8 m 的矩形（路线都是轴对齐折线，
            # 所以直接按方向拆半宽，不用做旋转矩形）
            hx = max(CAR_LEN_HALF * abs(ux), CAR_WIDE_HALF * abs(uy))
            hy = max(CAR_LEN_HALF * abs(uy), CAR_WIDE_HALF * abs(ux))
            run = None
            t = 0.0
            while t <= L + 1e-9:
                cx, cy = p[0] + ux * t, p[1] + uy * t
                h = tall_in(g, cx - hx, cx + hx, cy - hy, cy + hy)
                if h >= HMIN:
                    if run is None:
                        run = {'t0': t, 't1': t, 'h': h,
                               'mat': names.get((int((cx - X0) / CELL),
                                                 int((cy - Y0) / CELL)), '?'),
                               'p0': (cx, cy), 'p1': (cx, cy),
                               'seg': ((s - 1) % len(route), s)}
                    else:
                        run['t1'] = t
                        run['p1'] = (cx, cy)
                        if h > run['h']:
                            run['h'] = h
                elif run:
                    nseg_hit += 1
                    lines.append('   段 %d->%d (%.1f,%.1f)-(%.1f,%.1f)  穿楼 %.1f~%.1f m '
                                 '共 %.1f m  最高 %.2f m  材质 %s' % (
                                     run['seg'][0], run['seg'][1], p[0], p[1], q[0], q[1],
                                     run['t0'], run['t1'], run['t1'] - run['t0'],
                                     run['h'], run['mat'][:34]))
                    hits_all.append((name, run))
                    run = None
                t += 0.25
            if run:
                nseg_hit += 1
                lines.append('   段 %d->%d 末端仍压楼 %.1f m  最高 %.2f m  材质 %s' % (
                    run['seg'][0], run['seg'][1], run['t1'] - run['t0'],
                    run['h'], run['mat'][:34]))
                hits_all.append((name, run))
        lines.append('   该路线压楼航段数 = %d' % nseg_hit)
    return lines, hits_all


def render(g, hits):
    img = np.full((NR * PX, NC * PX, 3), 28, dtype=np.uint8)
    for iy in range(NR):
        for ix in range(NC):
            h = g[iy, ix]
            if h > 0:
                v = min(255, int(60 + h * 26))
                img[(NR - 1 - iy) * PX:(NR - iy) * PX,
                    ix * PX:(ix + 1) * PX] = (v, v, int(min(255, 60 + h * 40)))
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']

    def w2p(x, y):
        return (int((x - X0) / CELL * PX), int((NR - (y - Y0) / CELL) * PX))

    for name in sorted(cars):
        r = [tuple(p[:2]) for p in cars[name]['route']] + \
            [tuple(cars[name]['route'][0][:2])]
        col = COLORS.get(name, (255, 255, 255))
        for a, b in zip(r, r[1:]):
            cv2.line(img, w2p(*a), w2p(*b), col, 2)
        for k, (wx, wy) in enumerate(r[:-1]):
            pxp = w2p(wx, wy)
            cv2.circle(img, pxp, 5, col, -1)
            cv2.putText(img, str(k), (pxp[0] + 6, pxp[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
    for name, run in hits:
        a = w2p(*run['p0'])
        b = w2p(*run['p1'])
        cv2.line(img, a, b, (0, 0, 255), 5)
        cv2.putText(img, '%s %.0fm h%.1f' % (name, run['t1'] - run['t0'], run['h']),
                    (b[0] + 4, b[1] - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 0, 255), 1)
    cv2.imwrite(PNG, img)


def main():
    force = '--rebuild' in sys.argv
    m = load(force=force)
    g, names = grid(m)
    lines, hits = audit_routes(g, names)
    lines.insert(0, '高度图：%d 个 >=%.1f m 的格子（栅格 %.1f m，缓存 %s）' % (
        len(m['cells']), HMIN, CELL, MAP))
    lines.append('== 汇总 ==')
    lines.append('压楼航段总数 %d；涉及车辆 %s' % (
        len(hits), sorted({h[0] for h in hits}) or '无'))
    txt = '\n'.join(lines)
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(txt + '\n')
    render(g, hits)
    print('压楼航段 %d 个；明细写 %s，俯视图写 %s' % (len(hits), OUT, PNG))


if __name__ == '__main__':
    main()
