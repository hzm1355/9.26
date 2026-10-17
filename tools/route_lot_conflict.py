#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航线—停车场占地冲突体检：车流里每一辆车的每一段航段，量它与停车场占地的关系。

为什么要有这个：停车场是**后塞进沙盘的一块空地**，而车队航线是任务二/三/四时期画的
矩形环。两者只靠"我记得东腿在场外"是守不住的 —— 2026-10-14 那一版把东腿从 14.5
挪到 16.20，理由是"让出车位"，实测 16.20 的车体包络 15.75..16.65 仍然横穿第三个
原始车位（PKG-P3 画到 x=16.4525），也就是说那次"让位"根本没让开。这类事不能靠读
注释，要有一个把两条几何放在同一张图上算的判据。

口径：
  · 占地 = parking_zones.yaml 的 parking.lot（地坪/划线范围），另附三个车位框；
  · 车体包络 = 航段线段沿法向各扩 CAR_HALF(0.45)，即"车开过去要占的带"；
  · 判定用"带与矩形相交"，不用"中心线在不在矩形里" —— 中心线擦过角上也算压进；
  · 摆渡车（shuttle.fleets / shuttle 默认三段）天生就在场内，按模型名跳过，
    但会单独打印它场内航段与"别人的车位"的关系，防止两台摆渡车互相占位。
用法（WSL，仓库根目录）：
  python3 tools/route_lot_conflict.py                 # 全量报告
  python3 tools/route_lot_conflict.py --margin=0.15   # 换余量
退出码 0 = 场外车零冲突。
"""
import argparse
import os
import sys
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PK = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'parking_zones.yaml')
SD = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')
CAR_HALF = 0.40         # 车宽 0.80 的一半（parking_zones.yaml 的 vehicle.width）


def seg_rect(ax, ay, bx, by, x0, x1, y0, y1):
    """线段与轴对齐矩形是否相交（含端点在框内）。slab 法，不采样。"""
    dx, dy = bx - ax, by - ay
    t0, t1 = 0.0, 1.0
    for p, d, lo, hi in ((ax, dx, x0, x1), (ay, dy, y0, y1)):
        if abs(d) < 1e-12:
            if not (lo <= p <= hi):
                return False
            continue
        ta, tb = (lo - p) / d, (hi - p) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--margin', type=float, default=0.15)
    ap.add_argument('--parking', default=PK)
    ap.add_argument('--drive', default=SD)
    a = ap.parse_args()
    half = CAR_HALF + a.margin

    pk = yaml.safe_load(open(a.parking, encoding='utf-8'))['parking']
    lot, stalls = pk['lot'], pk['stalls']
    lx0, lx1, ly0, ly1 = lot['x0'], lot['x1'], lot['y0'], lot['y1']
    print('停车场占地 x %.2f..%.2f y %.2f..%.2f；车体带半宽 %.2f（%.2f+余量 %.2f）' % (
        lx0, lx1, ly0, ly1, half, CAR_HALF, a.margin))
    for s in stalls:
        cx, cy = s['center']
        w, d = s['size']
        print('  %-8s x %.3f..%.3f y %.2f..%.2f' % (s['id'], cx - w / 2, cx + w / 2,
                                                    cy - d / 2, cy + d / 2))
    cars = yaml.safe_load(open(a.drive, encoding='utf-8'))['signal_drive']['cars']
    shuttle = pk['shuttle']
    场内 = set([shuttle.get('model')] + list((shuttle.get('fleets') or {}).keys()))
    bad = 0
    print('\n== 车流航线（drive 为真且不属于摆渡分队的才判）==')
    for name in sorted(cars):
        cfg = cars[name]
        route = [tuple(p) for p in cfg['route']]
        pts = route + [route[0]]
        drive = cfg.get('drive', True)
        segs = []
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            # 带与矩形相交 = 把矩形按 half 外扩后判线段相交（等价、且免写法向）
            if seg_rect(ax, ay, bx, by, lx0 - half, lx1 + half, ly0 - half, ly1 + half):
                segs.append(((ax, ay), (bx, by)))
        hit_stall = []
        for s in stalls:
            cx, cy = s['center']
            w, d = s['size']
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                if seg_rect(ax, ay, bx, by, cx - w / 2 - half, cx + w / 2 + half,
                            cy - d / 2 - half, cy + d / 2 + half):
                    hit_stall.append(s['id'])
                    break
        tag = '摆渡分队' if name in 场内 else ('车流' if drive else '已退车流')
        state = '冲突' if segs else '不经过停车场'
        print('  %-7s [%-5s] %s  占地命中 %d 段  压车位 %s' % (
            name, tag, state, len(segs), ','.join(sorted(set(hit_stall))) or '无'))
        for (ax, ay), (bx, by) in segs:
            print('        段 (%.2f,%.2f)->(%.2f,%.2f)' % (ax, ay, bx, by))
        # 已退出车流的（drive:false）如果还留在场内也不判死，但要把它的航线画出来给人看
        if segs and (name in 场内 or not drive):
            continue
        if segs:
            bad += 1
    print('\n== 摆渡分队的应停位（谁停哪个车位，防两台互相占位）==')
    for m, f in (shuttle.get('fleets') or {}).items():
        tgt = [tuple(p) for p in f['entry_route']][-1]
        owner = next((s['id'] for s in stalls
                      if abs(tgt[0] - s['center'][0]) <= s['size'][0] / 2
                      and abs(tgt[1] - s['center'][1]) <= s['size'][1] / 2), '不在任何车位框内')
        print('  %-7s 入库终点 (%.2f,%.2f) -> %s' % (m, tgt[0], tgt[1], owner))
    print('\n结论：%d 条车流航线经过停车场占地' % bad)
    return 1 if bad else 0


sys.exit(main())
