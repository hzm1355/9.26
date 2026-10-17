#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按材质统计 room_33.obj 的三角形分布，挑出该补碰撞的"B 类设施"。

room_33 沙盘只给了一个等效盒碰撞（顶面 z=6.7 的平板），画在网格上的路缘石、
标志牌、绿化岛、树木物理上全是空气 → 车在画面里穿过去。要补等效盒，先得知道
这些物体归在哪些 usemtl 组下、各自高出路面多少、离车道有多近。

坐标系换算（与 v2x_city_scene.world 里 room_33 的定义一致）：
  模型 pose = 0 0 6.7 1.5708 0 0，mesh scale = 10
  → wx = 10*x，wy = -10*z，wz = 10*y + 6.7
  即"高出路面" h = wz - 6.7 = 10*y（局部 y=0 就是路面）。

用法（WSL）：
  cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
  python3 tools/mesh_probe.py [obj路径] [输出json]
"""
import json
import math
import os
import sys
from array import array

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    ROOT, 'src', 'smart_car_sim', 'models', 'room_33', 'meshes', 'room_33.obj')
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/room33_mats.json'
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')

S = 10.0
Z0 = 6.7
LANE_MAX = 3.0        # 距车道中心线 <=3 m 才算"跟车有关"
HBINS = (0.05, 0.2, 0.5, 1.0, 2.0, 3.0)


def lanes():
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    legs = []
    for cc in cars.values():
        r = [tuple(p[:2]) for p in cc['route']]
        for j in range(len(r)):
            legs.append((r[(j - 1) % len(r)], r[j]))
    return legs


def main():
    legs = lanes()

    def dlane(x, y):
        best = 1e9
        for p, q in legs:
            dx, dy = q[0] - p[0], q[1] - p[1]
            L = math.hypot(dx, dy) or 1.0
            ux, uy = dx / L, dy / L
            t = (x - p[0]) * ux + (y - p[1]) * uy
            t = 0.0 if t < 0 else (L if t > L else t)
            d = math.hypot(x - p[0] - ux * t, y - p[1] - uy * t)
            if d < best:
                best = d
        return best

    vx, vy, vz = array('d'), array('d'), array('d')
    mats = {}
    cur = '?'

    def bucket(name):
        b = mats.get(name)
        if b is None:
            b = mats[name] = {
                'n': 0, 'x': [1e9, -1e9], 'y': [1e9, -1e9], 'z': [1e9, -1e9],
                'hb': [0] * (len(HBINS) + 1), 'n_above': 0, 'n_nearlane': 0,
                'h_nearlane': [1e9, -1e9]}
        return b

    n_v = n_f = 0
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
                n_v += 1
            elif c0 == 102:
                if n_v == 0:
                    continue
                parts = raw[1:].split()
                idx = []
                for tok in parts:
                    if tok[:1] == b'/':        # 可能存在的 f v//vt 之外的材质组 token
                        continue
                    s = tok.split(b'/')[0]
                    try:
                        i = int(s)
                    except ValueError:
                        continue
                    idx.append(n_v + i + 1 if i < 0 else i)
                if len(idx) < 3:
                    continue
                a = idx[0] - 1
                for j in range(1, len(idx) - 1):
                    b, cc = idx[j] - 1, idx[j + 1] - 1
                    xs = (vx[a], vx[b], vx[cc])
                    ys = (vy[a], vy[b], vy[cc])
                    zs = (vz[a], vz[b], vz[cc])
                    wx0, wx1 = S * min(xs), S * max(xs)
                    wy0, wy1 = -S * max(zs), -S * min(zs)
                    h0, h1 = S * min(ys), S * max(ys)
                    bm = bucket(cur)
                    bm['n'] += 1
                    if wx0 < bm['x'][0]:
                        bm['x'][0] = wx0
                    if wx1 > bm['x'][1]:
                        bm['x'][1] = wx1
                    if wy0 < bm['y'][0]:
                        bm['y'][0] = wy0
                    if wy1 > bm['y'][1]:
                        bm['y'][1] = wy1
                    k = 0
                    while k < len(HBINS) and h1 > HBINS[k]:
                        k += 1
                    bm['hb'][k] += 1
                    if h1 > 0.05:
                        bm['n_above'] += 1
                        cx = 0.5 * (wx0 + wx1)
                        cy = 0.5 * (wy0 + wy1)
                        if dlane(cx, cy) <= LANE_MAX:
                            bm['n_nearlane'] += 1
                            if h0 < bm['h_nearlane'][0]:
                                bm['h_nearlane'][0] = h0
                            if h1 > bm['h_nearlane'][1]:
                                bm['h_nearlane'][1] = h1
                n_f += 1
            elif c0 == 117 or c0 == 103:        # 'u'(usemtl) / 'g'
                sp = raw.split(None, 1)
                if len(sp) > 1:
                    cur = sp[1].decode('utf-8', 'replace').strip()

    for k, b in mats.items():
        for a in ('x', 'y', 'z'):
            if b[a][0] > b[a][1]:
                b[a] = [0.0, 0.0]
        if b['h_nearlane'][0] > b['h_nearlane'][1]:
            b['h_nearlane'] = [0.0, 0.0]
    with open(OUT, 'w', encoding='utf-8') as fh:
        json.dump({'vertices': n_v, 'faces': n_f, 'hbins': list(HBINS),
                   'lane_max': LANE_MAX, 'materials': mats}, fh,
                  ensure_ascii=False, indent=1)

    print('OBJ %s' % OBJ)
    print('顶点 %d，面 %d，材质组 %d' % (n_v, n_f, len(mats)))
    rows = []
    for k, b in mats.items():
        d = dict(b)
        d['name'] = k
        rows.append(d)
    print('\n【离车道中心线 <= %.1f m、且高出路面】按该类三角形数排序（这些才需要补碰撞）' % LANE_MAX)
    print('  材质组                                          n_近道   高度带分布(<0.05|0.2|0.5|1|2|3|m)')
    for v in sorted(rows, key=lambda v: -v['n_nearlane'])[:40]:
        if v['n_nearlane'] == 0:
            continue
        print('  %-44s %8d  h %.2f..%.2f  %s' % (
            v['name'][:44], v['n_nearlane'], v['h_nearlane'][0], v['h_nearlane'][1],
            '|'.join(str(x) for x in v['hb'])))
    print('\n【全部材质组 TOP30】按三角形总数')
    for v in sorted(rows, key=lambda v: -v['n'])[:30]:
        print('  %-44s n=%9d above0.05=%8d 近道=%7d  x %.1f..%.1f y %.1f..%.1f' % (
            v['name'][:44], v['n'], v['n_above'], v['n_nearlane'],
            v['x'][0], v['x'][1], v['y'][0], v['y'][1]))
    print('\nJSON 已写到 %s' % OUT)


if __name__ == '__main__':
    main()
