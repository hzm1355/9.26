#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 world 里手写的隐形建筑盒（bldg_*）对到 room_33 网格的"落地障碍高度图"上。

背景：这些盒是给没有碰撞的沙盘建筑补的物理屏障（#76，只用等效盒不用三角网格）。
盒的轮廓当年是估的，一旦车道挪位，估值就会反过来咬人 —— 例如 bldg_school 名义
北边到 y=1.0，而北侧新车道在 y=1.5，车体半宽 0.5 m，于是盒与走廊间隙 0.00：
网格在那个位置根本没有墙（高度图实测 ≤0.36 m），盒却会把车拦住/刮到。

本脚本逐盒回答三件事：
  1) 盒与"车体走廊"（航段 ±CAR_W，两端 ±CAR_L）的最短间隙，是否挡车；
  2) 盒覆盖范围（可外扩 --pad）内，网格真正 ≥--hmin（默认 1.2 m，建筑级）的格子
     外接矩形 —— 这就是盒应该改成的尺寸；
  3) 盒里有多少格子在网格里其实是空的（盒画大了），空在外面的（盒画小了）。

用法（WSL，仓库根目录，需先有 /tmp/route_height_map.json，见 route_corridor_scan.py）：
  python3 tools/bldg_box_fit.py                 # 审计全部 bldg_*
  python3 tools/bldg_box_fit.py --pad=3         # 外扩 3 m 找实影
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')
MAP = '/tmp/route_height_map.json'

CAR_L = 0.85
CAR_W = 0.50
GROUND = 6.7


def opt(name, default, cast=str):
    for a in sys.argv[1:]:
        if a.startswith('--' + name + '='):
            return cast(a.split('=', 1)[1])
    return default


def parse_pose(e):
    if e is None or e.text is None:
        return [0.0] * 6
    v = [float(x) for x in e.text.replace('\n', ' ').split()]
    while len(v) < 6:
        v.append(0.0)
    return v[:6]


def rot(p):
    z, y, x = p[5], p[4], p[3]
    cz, sz = math.cos(z), math.sin(z)
    cy, sy = math.cos(y), math.sin(y)
    cx, sx = math.cos(x), math.sin(x)
    return ((cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
            (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
            (-sy, cy * sx, cy * cx))


def apply(m, t, v):
    x, y, z = v
    return (m[0][0] * x + m[0][1] * y + m[0][2] * z + t[0],
            m[1][0] * x + m[1][1] * y + m[1][2] * z + t[1],
            m[2][0] * x + m[2][1] * y + m[2][2] * z + t[2])


def mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def corridors(legs):
    """把正交航段扩成"车体走廊"矩形列表。"""
    out = []
    for p, q in legs:
        if abs(q[0] - p[0]) < 1e-9:
            out.append((p[0] - CAR_W, p[0] + CAR_W,
                        min(p[1], q[1]) - CAR_L, max(p[1], q[1]) + CAR_L))
        else:
            out.append((min(p[0], q[0]) - CAR_L, max(p[0], q[0]) + CAR_L,
                        p[1] - CAR_W, p[1] + CAR_W))
    return out


def gap(rects, x0, x1, y0, y1):
    best = 1e9
    for r in rects:
        dx = max(0.0, x0 - r[1], r[0] - x1)
        dy = max(0.0, y0 - r[3], r[2] - y1)
        best = min(best, math.hypot(dx, dy))
    return best


def main():
    pad = float(opt('pad', 2.0, float))
    hmin = float(opt('hmin', 1.2, float))
    hlow = float(opt('hlow', 0.45, float))
    name_pat = opt('name', 'bldg')
    with open(MAP, encoding='utf-8') as fh:
        mp = json.load(fh)
    cell, x0g, y0g = mp['cell'], mp['x0'], mp['y0']
    cells = [(x0g + ix * cell, y0g + iy * cell, h, grp)
             for ix, iy, h, grp in mp['cells']]
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    legs = []
    for cc in cars.values():
        r = [tuple(p[:2]) for p in cc['route']]
        for j in range(len(r)):
            legs.append((r[(j - 1) % len(r)], r[j]))
    rects = corridors(legs)

    tree = ET.parse(W)
    root = tree.getroot()
    worlds = root.findall('world') or ([root] if root.tag == 'world' else [])
    models = []
    for wd in worlds:
        models.extend(wd.findall('model'))
        for inc in wd.findall('include'):
            models.extend(inc.findall('model'))

    print('高度图 %s：格 %.2f m，落地图格 %d 个' % (MAP, cell, len(cells)))
    print('判据：建筑级 h>=%.2f m，护栏级 h>=%.2f m；走廊=航段 ±%.2f m、两端 ±%.2f m' % (
        hmin, hlow, CAR_W, CAR_L))
    n_bad = 0
    for m in models:
        name = m.get('name') or ''
        if name_pat not in name:
            continue
        mp6 = parse_pose(m.find('pose'))
        Mm = rot(mp6)
        tm = tuple(mp6[:3])
        boxes = []
        for l in list(m) + [lk for lk in m.findall('link')]:
            lp = parse_pose(l.find('pose'))
            Ml = mul(Mm, rot(lp))
            tl = apply(Mm, tm, lp[:3])
            for c in l.findall('collision'):
                g = c.find('geometry')
                b = None if g is None else g.find('box')
                if b is None:
                    continue
                half = [float(v) * 0.5 for v in b.find('size').text.split()]
                gp = parse_pose(c.find('pose'))
                Mc = mul(Ml, rot(gp))
                tc = apply(Ml, tl, gp[:3])
                xs, ys, zs = [], [], []
                for i in (-1, 1):
                    for j in (-1, 1):
                        for k in (-1, 1):
                            p = apply(Mc, tc, (i * half[0], j * half[1], k * half[2]))
                            xs.append(p[0])
                            ys.append(p[1])
                            zs.append(p[2])
                boxes.append((min(xs), max(xs), min(ys), max(ys),
                              min(zs), max(zs)))
        if not boxes:
            print('\n== %s  无盒碰撞（B 类，本来就拦不住）' % name)
            continue
        bx0 = min(b[0] for b in boxes)
        bx1 = max(b[1] for b in boxes)
        by0 = min(b[2] for b in boxes)
        by1 = max(b[3] for b in boxes)
        bz1 = max(b[5] for b in boxes)
        g = gap(rects, bx0, bx1, by0, by1)
        flag = '挡车' if g <= 0.0 else ('贴边' if g <= 0.5 else '安全')
        if g <= 0.5:
            n_bad += 1
        print('\n== %s  盒 x %.2f..%.2f y %.2f..%.2f（顶离地 %.2f m）  走廊间隙 %.2f m  [%s]'
              % (name, bx0, bx1, by0, by1, bz1 - GROUND, g, flag))
        # 网格实影：盒范围外扩 pad 内、落地高度 >= hmin 的格子
        for tag, h in (('建筑级 %.1f m+' % hmin, hmin), ('护栏级 %.1f m+' % hlow, hlow)):
            sel = [(cx, cy) for cx, cy, hh, grp in cells
                   if hh >= h and bx0 - pad <= cx <= bx1 + pad and by0 - pad <= cy <= by1 + pad]
            if not sel:
                print('   %s：外扩 %.1f m 内无网格实影（盒是凭空画的）' % (tag, pad))
                continue
            sx0 = min(p[0] for p in sel)
            sx1 = max(p[0] + cell for p in sel)
            sy0 = min(p[1] for p in sel)
            sy1 = max(p[1] + cell for p in sel)
            inside = sum(1 for cx, cy, hh, grp in cells
                         if hh >= h and bx0 <= cx <= bx1 and by0 <= cy <= by1)
            print('   %s：实影 x %.2f..%.2f y %.2f..%.2f（%d 格，其中 %d 格在盒内）' % (
                tag, sx0, sx1, sy0, sy1, len(sel), inside))
            if h == hmin:
                print('   建议盒（留出 %.2f m 余量后）：x %.2f..%.2f y %.2f..%.2f  '
                      '→ 中心 %.2f,%.2f 尺寸 %.2f×%.2f' % (
                          0.3, sx0 - 0.3, sx1 + 0.3, sy0 - 0.3, sy1 + 0.3,
                          0.5 * (sx0 + sx1), 0.5 * (sy0 + sy1),
                          (sx1 - sx0) + 0.6, (sy1 - sy0) + 0.6))
    print('\n走廊间隙 <=0.5 m 的手写盒共 %d 个' % n_bad)


if __name__ == '__main__':
    main()
