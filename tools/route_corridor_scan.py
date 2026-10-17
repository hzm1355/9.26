#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 room_33 沙盘网格建"落地障碍高度图"，体检现有路线并扫描可挪的车道走廊。

和 tools/gen_fixture_collisions.py 的区别（为什么不能拿它的产物代替）：
  生成器只在"车道中心线 ±3.5 m"环带里补盒、±2.0 m 禁入带内不补 ——
  路线本身画进建筑时那条路上永远不会有盒，所以补盒补不住穿楼，必须先改路线。
  本脚本不关心禁入带，只回答两个问题：
    1) 现在这 8 条航段各压在哪些"落地的高物体"上（穿楼明细）；
    2) 把这条航段横向平移多少米能全程净空（走廊候选）。

判"落地"的口径（与生成器一致）：三角形最低点 lo <= 0.35 m 才算障碍。
  只判顶高会把屋檐、过街龙门、高架、大牌面全算成墙 —— 上一版审计 33 段"穿楼"
  就是这么虚报的，车从下面过得去的结构被当成了建筑。

几何换算（与 world 里 room_33 的定义一致）：
  模型 pose = 0 0 6.7 1.5708 0 0，mesh scale = 10
  → wx = 10*x，wy = -10*z，高出路面 h = 10*y

用法（WSL，仓库根目录）：
  python3 tools/route_corridor_scan.py --rebuild      # 扫 OBJ 建图 + 全部报告
  python3 tools/route_corridor_scan.py                # 复用 /tmp/route_height_map.json
产物：
  /tmp/route_height_map.json   0.25 m 栅格落地障碍高度图
  /tmp/route_scan.txt          穿楼明细 + 走廊候选表
  /mnt/d/desktop1/9.26.1/_route_map.png  俯视体检图
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
MAP = '/tmp/route_height_map.json'
OUT = '/tmp/route_scan.txt'
PNG = '/mnt/d/desktop1/9.26.1/_route_map.png'

S = 10.0
CELL = 0.25
X0, X1 = -25.0, 25.0
Y0, Y1 = -21.0, 21.0
NC = int((X1 - X0) / CELL)
NR = int((Y1 - Y0) / CELL)

GROUND_LO = 0.35     # 三角形最低点低于此才算"落地"
H_KEEP = 0.15        # 低于此的高度对车没影响（路面纹理/薄涂装）
H_BLOCK = 1.20       # >=1.2 m 才算建筑：车体顶 0.8 m，1.2 m 以上必然撞

CAR_L = 0.85         # 沿行进方向半长（1.6 m + 余量）
CAR_W = 0.50         # 横向半宽（0.8 m + 余量）

PX = 6               # 每格 6 像素


def argv_opt(name, default, cast=str):
    for i, a in enumerate(sys.argv):
        if a.startswith(name + '='):
            return cast(a.split('=', 1)[1])
    return default


def point_in_tri(px, py, ax, ay, bx, by, cx, cy):
    d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
    d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
    d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
    return (d1 >= 0 and d2 >= 0 and d3 >= 0) or (d1 <= 0 and d2 <= 0 and d3 <= 0)


def seg_tri(ax, ay, bx, by, cx, cy, rx0, ry0, rx1, ry1):
    for pt in ((ax, ay), (bx, by), (cx, cy)):
        if rx0 <= pt[0] <= rx1 and ry0 <= pt[1] <= ry1:
            return True
    for pt in ((rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1)):
        if point_in_tri(pt[0], pt[1], ax, ay, bx, by, cx, cy):
            return True
    return False


def build():
    """流式扫 OBJ，返回 {(ix,iy): (hi, mat)}，只保留落地且高于 H_KEEP 的格。"""
    vx, vy, vz = [], [], []
    cells = {}
    cur = '?'
    ntri = 0
    nf_ground = 0
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
                    lo = S * min(vy[a], vy[b], vy[c])
                    ht = S * max(vy[a], vy[b], vy[c])
                    if lo > GROUND_LO or ht < H_KEEP:
                        continue
                    nf_ground += 1
                    if (ntri & 0x3FFFFF) == 0:
                        sys.stderr.write('  面 %d  落地面 %d  格 %d  %.0fs\n' % (
                            ntri, nf_ground, len(cells), time.time() - t0))
                        sys.stderr.flush()
                    axw, bxw, cxw = S * vx[a], S * vx[b], S * vx[c]
                    ayw, byw, cyw = -S * vz[a], -S * vz[b], -S * vz[c]
                    x0 = min(axw, bxw, cxw)
                    x1 = max(axw, bxw, cxw)
                    y0 = min(ayw, byw, cyw)
                    y1 = max(ayw, byw, cyw)
                    if x1 < X0 or x0 > X1 or y1 < Y0 or y0 > Y1:
                        continue
                    i0 = max(0, int((x0 - X0) / CELL))
                    i1 = min(NC - 1, int((x1 - X0) / CELL))
                    j0 = max(0, int((y0 - Y0) / CELL))
                    j1 = min(NR - 1, int((y1 - Y0) / CELL))
                    ncell = (i1 - i0 + 1) * (j1 - j0 + 1)
                    if ncell > 400:
                        # 整片楼板/铺装：只按顶点+重心+边中点落格，避免逐格测试吃时间
                        pts = ((axw, ayw), (bxw, byw), (cxw, cyw),
                               ((axw + bxw + cxw) / 3.0, (ayw + byw + cyw) / 3.0),
                               ((axw + bxw) / 2.0, (ayw + byw) / 2.0),
                               ((bxw + cxw) / 2.0, (byw + cyw) / 2.0),
                               ((cxw + axw) / 2.0, (cyw + ayw) / 2.0))
                        for px_, py_ in pts:
                            k = (int((px_ - X0) / CELL), int((py_ - Y0) / CELL))
                            if 0 <= k[0] < NC and 0 <= k[1] < NR:
                                old = cells.get(k)
                                if old is None or ht > old[0]:
                                    cells[k] = (ht, cur)
                        continue
                    for iy in range(j0, j1 + 1):
                        ry0 = Y0 + iy * CELL
                        ry1 = ry0 + CELL
                        for ix in range(i0, i1 + 1):
                            rx0 = X0 + ix * CELL
                            rx1 = rx0 + CELL
                            # 大面用格心包含、细长墙用相交：墙的投影可能窄到
                            # 盖不住任何格心，只判格心会漏成一条虚线
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
    sys.stderr.write('  扫完：面 %d  落地面 %d  高格 %d  %.0fs\n' % (
        ntri, nf_ground, len(cells), time.time() - t0))
    sys.stderr.flush()
    out = {'cell': CELL, 'x0': X0, 'y0': Y0, 'nc': NC, 'nr': NR,
           'ground_lo': GROUND_LO,
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


def worst_in(g, names, x0, x1, y0, y1):
    """矩形内最高落地障碍 -> (h, mat)。"""
    a, b = sorted((x0, x1))
    c, d = sorted((y0, y1))
    i0 = max(0, int((a - X0) / CELL))
    i1 = min(g.shape[1] - 1, int((b - X0) / CELL))
    j0 = max(0, int((c - Y0) / CELL))
    j1 = min(g.shape[0] - 1, int((d - Y0) / CELL))
    if i0 > i1 or j0 > j1:
        return 0.0, ''
    sub = g[j0:j1 + 1, i0:i1 + 1]
    h = float(sub.max())
    if h <= 0:
        return 0.0, ''
    iy, ix = np.unravel_index(int(sub.argmax()), sub.shape)
    return h, names.get((i0 + int(ix), j0 + int(iy)), '?')


def profile(g, names, fixed, lo, hi, vertical, hblock):
    """沿一条轴对齐航段按 0.25 m 采样车体矩形，返回 [(位置, 最高障碍, 材质)]。"""
    out = []
    t = lo
    while t <= hi + 1e-9:
        if vertical:
            h, mat = worst_in(g, names, fixed - CAR_W, fixed + CAR_W,
                              t - CAR_L, t + CAR_L)
        else:
            h, mat = worst_in(g, names, t - CAR_L, t + CAR_L,
                              fixed - CAR_W, fixed + CAR_W)
        out.append((t, h, mat))
        t += 0.25
    return out


def runs_of(prof, hblock):
    """把连续超阈值区间合并成 [(t0,t1,hmax,mat)]。"""
    runs = []
    cur = None
    for t, h, mat in prof:
        if h >= hblock:
            if cur is None:
                cur = [t, t, h, mat]
            else:
                cur[1] = t
                if h > cur[2]:
                    cur[2] = h
        elif cur is not None:
            runs.append(tuple(cur))
            cur = None
    if cur is not None:
        runs.append(tuple(cur))
    return runs


# 8 条航段（与 config 两条环线逐腿对应；改路线后这里同步，否则扫描的是旧走廊）
LEGS = [
    ('A_west',  'v', -21.8, -16.8, 1.5),
    ('A_north', 'h', 1.5, -21.8, -0.8),
    ('A_mid_s', 'v', -0.8, -16.8, 1.5),
    ('A_south', 'h', -16.8, -21.8, -0.8),
    ('B_mid_n', 'v', 1.8, -16.8, 1.5),
    ('B_north', 'h', 1.5, 1.8, 14.5),
    ('B_east',  'v', 14.5, -16.8, 1.5),
    ('B_south', 'h', -16.8, 1.8, 14.5),
]


def scan_corridor(g, names, leg, hblock, reach, step):
    """横向平移该腿，返回每个候选偏移的最差净空。"""
    kind, fixed, lo, hi = leg[1], leg[2], leg[3], leg[4]
    res = []
    # 候选轴按方向取对应范围：竖腿挪的是 x，不能被 y 的边界卡掉
    a_lo, a_hi = (X0 + 1.0, X1 - 1.0) if kind == 'v' else (Y0 + 1.0, Y1 - 1.0)
    off = -reach
    while off <= reach + 1e-9:
        cand = fixed + off
        if cand < a_lo or cand > a_hi:
            off += step
            continue
        prof = profile(g, names, cand, min(lo, hi), max(lo, hi),
                       kind == 'v', hblock)
        bad = runs_of(prof, hblock)
        L = abs(hi - lo)
        blocked = sum(r[1] - r[0] for r in bad)
        hmax = max((p[1] for p in prof), default=0.0)
        res.append((round(off, 2), blocked / max(L, 1e-6), hmax, bad))
        off += step
    return res


def main():
    force = '--rebuild' in sys.argv
    hblock = argv_opt('--hblock', 1.2, float)
    reach = argv_opt('--reach', 6.0, float)
    step = argv_opt('--step', 0.2, float)
    m = load(force=force)
    g, names = grid(m)
    lines = ['落地障碍高度图：%d 格（lo<=%.2f 且 h>=%.2f，栅格 %.2f m，缓存 %s）' % (
        len(m['cells']), GROUND_LO, H_KEEP, CELL, MAP),
        '建筑阈值 hblock=%.2f m（车体顶 0.8 m）；走廊扫描 ±%.1f m、步长 %.1f m' % (
            hblock, reach, step), '']

    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']

    lines.append('【一】现有路线逐车体检（只算 >=%.1f m 的落地障碍）' % hblock)
    n_hit = 0
    hit_marks = []
    for name in sorted(cars):
        route = [tuple(p[:2]) for p in cars[name]['route']]
        tot = 0.0
        detail = []
        for s in range(len(route)):
            p, q = route[s - 1], route[s]
            L = math.hypot(q[0] - p[0], q[1] - p[1])
            if L < 1e-6:
                continue
            ux, uy = (q[0] - p[0]) / L, (q[1] - p[1]) / L
            prof = []
            t = 0.0
            while t <= L + 1e-9:
                cx, cy = p[0] + ux * t, p[1] + uy * t
                hx = max(CAR_L * abs(ux), CAR_W * abs(uy))
                hy = max(CAR_L * abs(uy), CAR_W * abs(ux))
                h, mat = worst_in(g, names, cx - hx, cx + hx, cy - hy, cy + hy)
                prof.append((t, h, mat))
                t += 0.25
            for r in runs_of(prof, hblock):
                tot += r[1] - r[0]
                n_hit += 1
                detail.append('   段 %d->%d (%.1f,%.1f)-(%.1f,%.1f)  撞 %.1f~%.1f m'
                              ' 共 %.1f m  最高 %.2f m  材质 %s' % (
                                  (s - 1) % len(route), s, p[0], p[1], q[0], q[1],
                                  r[0], r[1], r[1] - r[0], r[2], r[3][:30]))
                a = (p[0] + ux * r[0], p[1] + uy * r[0])
                b = (p[0] + ux * r[1], p[1] + uy * r[1])
                hit_marks.append((name, a, b, r[2]))
        lines.append('== %s  撞楼总长 %.1f m / 环长 %.1f m' % (name, tot, sum(
            math.hypot(route[i][0] - route[i - 1][0],
                       route[i][1] - route[i - 1][1])
            for i in range(len(route)))))
        lines.extend(detail)

    lines.append('')
    lines.append('【二】按腿扫描走廊（blocked=撞楼占比，hmax=该偏移全程最高障碍）')
    for leg in LEGS:
        kind, fixed, lo, hi = leg[1], leg[2], leg[3], leg[4]
        lines.append('== %s  %s 固定 %.1f  跨度 %.1f~%.1f' % (
            leg[0], '竖(x=)' if kind == 'v' else '横(y=)', fixed, lo, hi))
        res = scan_corridor(g, names, leg, hblock, reach, step)
        clear = [r for r in res if r[1] <= 1e-9]
        for off, frac, hmax, bad in res:
            tag = '净空' if frac <= 1e-9 else ('撞 %.0f%%' % (frac * 100))
            lines.append('   偏移 %+5.2f -> %6.2f  %s  hmax %.2f  %s' % (
                off, fixed + off, tag, hmax,
                '；'.join('%.1f~%.1f(h%.1f)' % (b[0], b[1], b[2])
                          for b in bad[:6])))
        if clear:
            best = min(clear, key=lambda r: r[2])
            lines.append('   >>> 全程净空候选 %d 个，最贴近原线：偏移 %+.2f（%s=%.2f，'
                         'hmax %.2f）' % (len(clear), best[0],
                                          'x' if kind == 'v' else 'y',
                                          fixed + best[0], best[2]))
            # 连续净空区间，便于直接选车道
            runs = []
            cur = None
            for off, frac, hmax, bad in res:
                if frac <= 1e-9:
                    if cur is None:
                        cur = [off, off, hmax]
                    else:
                        cur[1] = off
                        cur[2] = max(cur[2], hmax)
                elif cur is not None:
                    runs.append(tuple(cur))
                    cur = None
            if cur is not None:
                runs.append(tuple(cur))
            for a, b, hm in runs:
                if b - a >= 0.6:
                    lines.append('       净空带 %+.2f~%+.2f（%s=%.2f~%.2f，'
                                 '全程最高 %.2f m）' % (
                                     a, b, 'x' if kind == 'v' else 'y',
                                     fixed + a, fixed + b, hm))
        else:
            lines.append('   >>> ±%.1f m 内没有全程净空偏移，需要拆段或绕行' % reach)

    txt = '\n'.join(lines)
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(txt + '\n')
    render(g, names, cars, hit_marks)
    print('撞楼段 %d 个；报告写 %s，俯视图写 %s' % (n_hit, OUT, PNG))


def render(g, names, cars, hit_marks):
    img = np.full((NR * PX, NC * PX, 3), 20, dtype=np.uint8)
    for iy in range(NR):
        for ix in range(NC):
            h = float(g[iy, ix])
            if h > 0:
                v = min(250, int(50 + h * 22))
                img[(NR - 1 - iy) * PX:(NR - iy) * PX,
                    ix * PX:(ix + 1) * PX] = (v, v, v)
    # 与 world 里各车车身 diffuse 一致的 BGR（car_1/car_4 都是蓝，一个偏亮一个偏深）
    col = {'car_1': (204, 102, 26), 'car_2': (64, 153, 38),
           'car_3': (26, 115, 217), 'car_4': (255, 255, 255),
           'car_5': (64, 38, 179)}

    def w2p(x, y):
        return (int((x - X0) / CELL * PX), int((NR - (y - Y0) / CELL) * PX))

    for name in sorted(cars):
        r = [tuple(p[:2]) for p in cars[name]['route']]
        r = r + [r[0]]
        c = col.get(name, (255, 255, 255))
        for a, b in zip(r, r[1:]):
            cv2.line(img, w2p(*a), w2p(*b), c, 1)
        for k, (wx, wy) in enumerate(r[:-1]):
            p = w2p(wx, wy)
            cv2.circle(img, p, 4, c, -1)
            cv2.putText(img, '%s%d' % (name[-1], k), (p[0] + 5, p[1] - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, c, 1)
    for name, a, b, h in hit_marks:
        cv2.line(img, w2p(*a), w2p(*b), (0, 0, 255), 4)
    # 坐标网格：每 5 m 一条线并标数字，便于直接读出新车道坐标
    for wv in range(-25, 26, 5):
        cv2.line(img, w2p(wv, Y0), w2p(wv, Y1), (90, 90, 90), 1)
        cv2.line(img, w2p(X0, wv), w2p(X1, wv), (90, 90, 90), 1)
        cv2.putText(img, 'x%d' % wv, (w2p(wv, Y1)[0] + 2, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (160, 160, 160), 1)
        cv2.putText(img, 'y%d' % wv, (4, w2p(X0, wv)[1] - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (160, 160, 160), 1)
    cv2.imwrite(PNG, img)


if __name__ == '__main__':
    main()
