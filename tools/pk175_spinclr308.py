#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摆渡车"原地转净空"审计：把 parking_zones.yaml 里每一次原地转向拿去撞 world 的
每一个碰撞盒，用的是节点自己那套口径（绕**轮轴**扫掠、半径 hypot(半长+axle, 半宽)
+ SPIN_MARGIN），不是中心距。

为什么要它：car_3 挂起在 (6.32,-13.37) 给 ω 不动，查下来不是 diff_drive 卡死，而是
exit 末点->reset#0 那一格 -90°->0° 的转向天生扫不进净空 —— 隐形盒
sandbox_fixtures_coll/c130..c132 是沙盘西南角那栋 2.7 m 楼的北边缘
（x 4.25..7.00、y -14.75..-14.00、高 1.0 m），车在 (6.80,-12.80) 车头朝南时轮轴在
(6.80,-13.20)，扫掠圆最远够到 y=-14.565，**吃进楼体 0.565 m**。按中心距看是
"1.2 m 外的盒子"，按接触几何看是撞墙。节点里 spin_clear/escape_ok 这套机制是为此
而写的，但 parking_zones.yaml 的 keepout 表是空的，于是 spin_clear 恒真、脱困永不
触发，只能一路加 ω 直到挂起。

接缝也算：周期是 entry -> exit -> reset -> entry…，"同址换角"最容易发生在两段交界
处，只查段内会正好漏掉卡死的那一格。

用法（WSL，仓库根目录）：
  python3 tools/pk175_spinclr308.py                    # 审计一个周期的所有原地转
  python3 tools/pk175_spinclr308.py --probe=x,y,th     # 单点：原地转得开吗
  python3 tools/pk175_spinclr308.py --find=x0,y0,x1,y1 # 网格搜"朝南转成朝东"的落点
"""
import json
import math
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'parking_zones.yaml')
NUM = r'[-+]?(?:\d+\.?\d*|\.\d+)'
# 台面/地面：room_33 的等效盒顶面就是路面，按"盒"算会把所有点判成"压在车下"
SKIP_MODELS = {'room_33', 'room_20', 'room_12', 'ground_plane'}
SKIP_PREFIX = ('car_',)
GROUND = 6.70


def load_boxes():
    """返回 [(key, wx, wy, th, hx, hy, top)]，几何叠加口径照 fixtures_near.py：
    model pose -> link pose -> collision pose。"""
    with open(W, encoding='utf-8', errors='replace') as f:
        txt = f.read()
    out = []
    for mm in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S):
        mname, mbody = mm.group(1), mm.group(2)
        if mname in SKIP_MODELS or mname.startswith(SKIP_PREFIX):
            continue
        head = re.split(r'<link\b', mbody, 1)[0]
        mp = re.search(r'<pose>\s*((?:%s\s*){6})</pose>' % NUM, head)
        mv = [float(v) for v in mp.group(1).split()] if mp else [0.0] * 6
        mx, my, mth = mv[0], mv[1], mv[5]

        def place(lx, ly, lth):
            return (mx + lx * math.cos(mth) - ly * math.sin(mth),
                    my + lx * math.sin(mth) + ly * math.cos(mth), mth + lth)

        for lm in re.finditer(r'<link name="([^"]+)">(.*?)</link>', mbody, re.S):
            lname, lbody = lm.group(1), lm.group(2)
            # link 级 pose 只认在第一个子元素之前那几个数。早先按"collision 之前"切，
            # 于是 parking_booth 这种 link 自己不写 pose、第一个 visual 写 z=1.1 的，
            # 会把 visual 的 pose 当成 link 的 pose 叠第二遍 —— 顶面抬到 7.8 m，
            # 被判成架空件整根丢掉，净空表就"假绿"了（岗亭、龙门架立杆都不在表里）。
            lhead = re.split(r'<(?:visual|collision|inertial|sensor|camera)\b',
                             lbody, 1)[0]
            lp = re.search(r'<pose>\s*((?:%s\s*){6})</pose>' % NUM, lhead)
            lv = [float(v) for v in lp.group(1).split()] if lp else [0.0] * 6
            for cm in re.finditer(r'<collision name="([^"]+)">(.*?)</collision>',
                                  lbody, re.S):
                cname, cbody = cm.group(1), cm.group(2)
                cp = re.search(r'<pose>\s*((?:%s\s*){6})</pose>' % NUM, cbody)
                cv = [float(v) for v in cp.group(1).split()] if cp else [0.0] * 6
                wx, wy, th = place(lv[0] + cv[0], lv[1] + cv[1], lv[5] + cv[5])
                z = mv[2] + lv[2] + cv[2]
                b = re.search(r'<box>\s*<size>\s*((?:%s\s*){3})</size>' % NUM, cbody)
                c = re.search(r'<cylinder>\s*<radius>\s*(%s)\s*</radius>' % NUM, cbody)
                if b:
                    sx, sy, sz = [float(v) for v in b.group(1).split()][:3]
                    hx, hy = sx / 2.0, sy / 2.0
                elif c:
                    hx = hy = float(c.group(1))
                    sz = float(re.search(r'<length>\s*(%s)\s*</length>' % NUM,
                                         cbody).group(1))
                else:
                    continue
                if z - sz / 2.0 > GROUND + 0.30:
                    continue      # 架空件（龙门架横梁、灯头），车从下面过
                top = z + sz / 2.0 - GROUND
                if top <= 0.05:
                    continue      # 地板 slab（parking_floor 那类），车压在上面走
                out.append(('%s/%s/%s' % (mname, lname, cname), wx, wy, th, hx, hy,
                            top))
    return out


def dist_box(px, py, bx):
    """点到旋转矩形的边距；在内部返回 0。"""
    _, wx, wy, th, hx, hy, _h = bx
    dx, dy = px - wx, py - wy
    u = dx * math.cos(-th) - dy * math.sin(-th)
    v = dx * math.sin(-th) + dy * math.cos(-th)
    ox = max(0.0, abs(u) - hx)
    oy = max(0.0, abs(v) - hy)
    if ox == 0.0 and oy == 0.0:
        return 0.0
    return math.hypot(ox, oy)


def spin_report(boxes, x, y, th0, th1, axle, r):
    """从 th0 转到 th1，每 5 度取一次轮轴位置，找最坏的那个盒。"""
    worst = (1e9, None, None)
    n = max(1, int(abs(th1 - th0) / math.radians(5)) + 1)
    for i in range(n + 1):
        t = th0 + (th1 - th0) * i / n
        px, py = x + axle * math.cos(t), y + axle * math.sin(t)
        for bx in boxes:
            g = dist_box(px, py, bx) - r
            if g < worst[0]:
                worst = (g, bx, t)
    return worst


def fmt(bx):
    """把违规件打成可读占地，省得对着 world 行号猜是哪一根。"""
    key, wx, wy, th, hx, hy, top = bx
    if abs(th) < 1e-6:
        return '%s x %.2f..%.2f y %.2f..%.2f 顶%.2f' % (
            key, wx - hx, wx + hx, wy - hy, wy + hy, top)
    return '%s 中心(%.2f,%.2f) yaw=%+.2f 半%.2f×%.2f 顶%.2f' % (
        key, wx, wy, th, hx, hy, top)


MESH = '/tmp/mesh_tall_cells.json'


def load_mesh(hmin=1.0):
    """全图 0.5 m 栅格里高出路面 >=hmin 的格心（沙盘楼体，**纯视觉无碰撞盒**）。
    净空判据只看碰撞盒会漏它：车会"从楼里开过去"，画面就是用户说的贴着墙画圈。"""
    if not os.path.exists(MESH):
        return None
    with open(MESH, encoding='utf-8') as fh:
        mp = json.load(fh)
    cell, x0g, y0g = mp['cell'], mp['x0'], mp['y0']
    return [(x0g + (ix + 0.5) * cell, y0g + (iy + 0.5) * cell, h)
            for ix, iy, h, _g in mp['cells'] if h >= hmin]


def mesh_worst(mesh, x, y, th0, th1, axle, r):
    """扫掠圆与楼体格心的最坏重叠量（负=吃进）。格按中心点算，偏乐观 0.25 m。"""
    if not mesh:
        return (1e9, None)
    n = max(1, int(abs(th1 - th0) / math.radians(5)) + 1)
    worst = (1e9, None)
    for i in range(n + 1):
        t = th0 + (th1 - th0) * i / n
        px, py = x + axle * math.cos(t), y + axle * math.sin(t)
        for cx, cy, h in mesh:
            g = math.hypot(cx - px, cy - py) - r
            if g < worst[0]:
                worst = (g, (cx, cy, h))
    return worst


def main():
    axle = 0.40
    margin = 0.10
    bl, bw = 1.60, 0.80
    r = math.hypot(bl / 2.0 + axle, bw / 2.0) + margin
    with open(CFG, encoding='utf-8') as fh:
        cfg = yaml.safe_load(fh)['parking']['shuttle']
    fleets = cfg['fleets']
    boxes = load_boxes()
    mesh = load_mesh()
    print('碰撞盒 %d 个（已排台面/地面/车辆/架空件/地板 slab）；扫掠判据 R=%.3f m'
          '（半长%.2f+轮轴%.2f、半宽%.2f、余量%.2f）' % (
              len(boxes), r, bl / 2, axle, bw / 2, margin))
    print('沙盘楼体：' + ('未生成 %s（先跑 mesh_intrusion_audit.py）' % MESH
                        if mesh is None else
                        '%d 格（纯视觉无碰撞盒，格心距按 0.35 m 补半格余量）' % len(mesh)))

    argv = sys.argv[1:]
    if any(a.startswith('--probe') for a in argv):
        v = [a.split('=', 1)[1] for a in argv if a.startswith('--probe')][0]
        x, y, th = [float(t) for t in v.split(',')]
        g, bx, t = spin_report(boxes, x, y, th, th, axle, r)
        mg, mc = mesh_worst(mesh, x, y, th, th, axle, r)
        print('单点 (%.2f,%.2f) 车头 %+.2f：碰撞盒最坏间隙 %s%.3f m ← %s' % (
            x, y, th, '吃进 ' if g < 0 else '', g, fmt(bx) if bx else '-'))
        if mc:
            print('               楼体格最坏间隙 %s%.3f m ← (%.2f,%.2f) 高%.2f' % (
                '吃进 ' if mg < 0 else '', mg, mc[0], mc[1], mc[2]))
        return 0
    if any(a.startswith('--find') for a in argv):
        v = [a.split('=', 1)[1] for a in argv if a.startswith('--find')][0]
        parts = [float(t) for t in v.split(',')]
        x0, y0, x1, y1 = parts[:4]
        th0, th1 = (parts[4], parts[5]) if len(parts) > 5 else (-math.pi / 2, 0.0)
        print('在 x %.2f..%.2f / y %.2f..%.2f 上以 0.2 m 步长搜 "%+.2f->%+.2f" 的合法落点：'
              % (x0, x1, y0, y1, th0, th1))
        ok = []
        X = x0
        while X <= x1 + 1e-9:
            Y = y0
            while Y <= y1 + 1e-9:
                g, bx, t = spin_report(boxes, X, Y, th0, th1, axle, r)
                mg, mc = mesh_worst(mesh, X, Y, th0, th1, axle, r)
                if g >= 0 and mg >= 0.35:
                    ok.append((X, Y, min(g, mg - 0.35)))
                Y += 0.2
            X += 0.2
        for X, Y, g in sorted(ok, key=lambda o: -o[2])[:60]:
            print('  (%.2f, %.2f) 余 %.3f m' % (X, Y, g))
        print('  合法落点 %d 个 / 共扫 %d 格' % (
            len(ok), int(round((x1 - x0) / 0.2) + 1) * int(round((y1 - y0) / 0.2) + 1)))
        return 0

    n_bad = 0
    for mname, fl in sorted(fleets.items()):
        order = [k for k in ('entry_route', 'exit_route', 'reset_route') if k in fl]
        seq = [(key, wp) for key in order for wp in fl[key]]
        if seq:
            seq.append((u'↩entry', seq[0][1]))
        print('\n== %s ==' % mname)
        for j in range(1, len(seq)):
            key, wp = seq[j]
            pkey, prev = seq[j - 1]
            x, y, th = wp[0], wp[1], wp[2]
            dth = ((th - prev[2] + math.pi) % (2 * math.pi)) - math.pi
            same = abs(prev[0] - x) < 1e-6 and abs(prev[1] - y) < 1e-6
            if same:
                if abs(dth) < 1e-6:
                    continue
                g, bx, t = spin_report(boxes, x, y, prev[2], th, axle, r)
                mg, mc = mesh_worst(mesh, x, y, prev[2], th, axle, r)
                ms = '' if not mc else '  楼%s%.2f@%.1f' % (
                    '吃进' if mg < 0.35 else '贴', mg, mc[2])
                if g < 0:
                    n_bad += 1
                    print('  撞  %-11s#%d %+.2f->%+.2f @ (%.2f,%.2f) 盒吃进 %.3f m ← %s%s'
                          % (key, j, prev[2], th, x, y, -g, fmt(bx), ms))
                elif g < 0.15 or (mc and mg < 0.35):
                    print('  紧  %-11s#%d %+.2f->%+.2f @ (%.2f,%.2f) 盒余 %.3f m ← %s%s'
                          % (key, j, prev[2], th, x, y, g, fmt(bx), ms))
            else:
                L = math.hypot(x - prev[0], y - prev[1])
                seg = max(1, int(L / 0.1))
                wg = (1e9, None)
                for i in range(seg + 1):
                    sx = prev[0] + (x - prev[0]) * i / seg
                    sy = prev[1] + (y - prev[1]) * i / seg
                    for bx in boxes:
                        g = dist_box(sx, sy, bx) - (bw / 2.0)
                        if g < wg[0]:
                            wg = (g, bx)
                if wg[0] < 0.15:
                    print('  廊  %-11s#%d (%.2f,%.2f)->(%.2f,%.2f) 余 %.3f m ← %s'
                          % (key, j, prev[0], prev[1], x, y, wg[0], fmt(wg[1])))
                if mesh:
                    lx, ly = x - prev[0], y - prev[1]
                    LL = math.hypot(lx, ly) or 1.0
                    wm = (1e9, None)
                    for cx, cy, h in mesh:
                        # 点到线段的最近距离，再减车半宽与半格
                        t = max(0.0, min(1.0, ((cx - prev[0]) * lx
                                               + (cy - prev[1]) * ly) / (LL * LL)))
                        d = math.hypot(cx - prev[0] - t * lx,
                                       cy - prev[1] - t * ly) - (bw / 2.0 + 0.35)
                        if d < wm[0]:
                            wm = (d, (cx, cy, h))
                    if wm[0] < 0.0:
                        print('  穿  %-11s#%d (%.2f,%.2f)->(%.2f,%.2f) 吃进楼体 %.2f m'
                              ' @ (%.2f,%.2f) 高%.2f'
                              % (key, j, prev[0], prev[1], x, y, -wm[0],
                                 wm[1][0], wm[1][1], wm[1][2]))
    print('\n原地转判"撞"的航点共 %d 个' % n_bad)
    return 0


if __name__ == '__main__':
    sys.exit(main())
