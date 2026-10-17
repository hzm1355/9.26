#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫原始沙盘网格里"贴地色块"：在指定世界矩形内，按材质列出每一片的颜色、
三角形数与世界 AABB。

为什么要这一发：用户说"停车位是原始模型中已经标记的部分，有 3 个，包括现在两个
黑色的，还有旁边绿色的那个"。pk107/pk108 只按"绿色 Kd"筛，拿到一块 9.07×4.30 m
的深绿贴地矩形（x 7.30..16.37 × y −11.48..−7.18），但它和 Ring B 东环车道
（x=14.5，car_2/4/5 都在跑）整段重叠 —— 把车位摆进去就是往行车道里停车。
所以"绿色那块"到底是用户指的那一块，还是另有所指，不能靠猜：把这一带**所有**
贴地的、成片的色块按材质列出来，看是不是恰好"两块黑的 + 一块绿的"并排。

判"贴地"用三角形三个顶点的高度都 <= FLAT_H（沙盘路面 h=0，即世界 z=6.7）；
判"成片"用 AABB 面积下限，滤掉划线碎面。

用法（WSL）：
  cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
  python3 tools/pk110_patches217.py [obj] [x0 x1 y0 y1]
"""
import os
import sys
from array import array

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = sys.argv[1] if len(sys.argv) > 1 else '/tmp/room_33_214.obj'
MTL = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.mtl')
S, FLAT_H, MIN_AREA = 10.0, 0.12, 0.5
_X0, _X1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 4 else (2.0, 22.0)
_Y0, _Y1 = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 6 else (-20.0, -2.0)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pk107_green214 import load_mtl       # noqa: E402


def main():
    kd = load_mtl(MTL)
    vx, vy, vz = array('d'), array('d'), array('d')
    groups = {}
    cur = '?'
    n_v = 0
    with open(OBJ, 'rb', buffering=1 << 22) as fh:
        for raw in fh:
            c0 = raw[:1]
            if c0 == b'v' and raw[1:2] == b' ':
                p = raw[2:].split()
                vx.append(float(p[0]))
                vy.append(float(p[1]))
                vz.append(float(p[2]))
                n_v += 1
            elif c0 == b'f' and n_v:
                idx = []
                for tok in raw[1:].split():
                    try:
                        i = int(tok.split(b'/')[0])
                    except ValueError:
                        continue
                    idx.append(n_v + i + 1 if i < 0 else i)
                if len(idx) < 3:
                    continue
                a = idx[0] - 1
                for j in range(1, len(idx) - 1):
                    b, c = idx[j] - 1, idx[j + 1] - 1
                    xs = [S * vx[k] for k in (a, b, c)]
                    ys = [-S * vz[k] for k in (a, b, c)]
                    hs = [S * vy[k] for k in (a, b, c)]
                    if max(hs) > FLAT_H:
                        continue
                    if min(xs) > _X1 or max(xs) < _X0 or min(ys) > _Y1 or max(ys) < _Y0:
                        continue
                    g = groups.setdefault(cur, {
                        'n': 0, 'x': [1e9, -1e9], 'y': [1e9, -1e9]})
                    g['n'] += 1
                    g['x'] = [min(g['x'][0], min(xs)), max(g['x'][1], max(xs))]
                    g['y'] = [min(g['y'][0], min(ys)), max(g['y'][1], max(ys))]
            elif c0 == b'u' or c0 == b'g':
                sp = raw.split(None, 1)
                if len(sp) > 1:
                    cur = sp[1].decode('utf-8', 'replace').strip()

    print('区域 x %.1f..%.1f × y %.1f..%.1f 内的贴地材质组（按 AABB 面积降序）' % (
        _X0, _X1, _Y0, _Y1))
    print('  %-34s %-18s %6s  %-28s  %s' % ('材质组', 'Kd', 'n', 'AABB x', 'AABB y'))
    rows = []
    for nm, g in groups.items():
        area = (g['x'][1] - g['x'][0]) * (g['y'][1] - g['y'][0])
        rows.append((area, nm, g))
    for area, nm, g in sorted(rows, key=lambda r: -r[0]):
        if area < MIN_AREA:
            continue
        c = kd.get(nm)
        cs = '%.2f,%.2f,%.2f' % c if c else '?'
        print('  %-34s %-18s %6d  %7.2f..%7.2f  %8.2f..%8.2f  面积 %6.1f' % (
            nm[:34], cs, g['n'], g['x'][0], g['x'][1], g['y'][0], g['y'][1], area))
    return 0


if __name__ == '__main__':
    sys.exit(main())
