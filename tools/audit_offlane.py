#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线全局穿障审计：拿 signal_drive_node 自己落的 20 Hz 轨迹 CSV 说话。

为什么用 CSV 而不是再跑一轮采样：signal_drive_node 从启动起就在往 path_csv 里逐拍写
`t,car,x,y,yaw,speed,limited_by,phase`，现场这一份已经攒了几个小时仿真时间，比任何
临时采样都全。而且它记的是**车真实被开到哪里**，不是配置里"应该"在哪里。

审计三件事：
  1) 横向偏移：把位姿投到当前航段，取横向绝对值。>lane_half 就是"已经不在自己车道里"。
  2) 无航段可投（车在两角点之间的街区里，投影落在所有航段之外）= 已经离线，
     这一类比横偏更严重，因为它意味着车在**斜穿街区**。
  3) 车体 OBB（1.6x0.8 的 8 个边界采样点）与带碰撞的静态件做带符号距离，负值=已侵入。
     静态件清单来自 world 的 SDF 生成源：5 个 bldg_* 碰撞盒 + 停车场的道闸机箱/杆、
     岗亭、龙门架立杆（杆按"放平/抬起"两种姿态都算一次）。

用法（WSL）：
  python3 tools/audit_offlane.py [csv路径]
默认 /tmp/signal_drive_path.csv。退出码恒 0，只看报告。
"""
import csv
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config')

CAR_HL, CAR_HW = 0.8, 0.4          # 车体碰撞盒半长/半宽（SDF 1.6 x 0.8）
LAT_TOL = 0.9                      # 横向容差：车道中心到路缘大约 1.0 m，超过就算压缘
OFFLINE_TOL = 1.0                  # 航段前后各放 1.0 m，仍投不上就是离线

# 建筑碰撞盒（与 tools/audit_drive.py 同一份，取自 world 里的 static box）
BUILDINGS = {
    'bldg_school': (-17.0, -3.0, -13.0, 1.0),
    'bldg_nw': (-18.0, -9.0, 8.0, 17.0),
    'bldg_tower_w': (-7.5, -0.5, 10.5, 17.5),
    'bldg_tower_e': (5.5, 12.5, 10.5, 17.5),
    'bldg_ne': (12.5, 21.5, 8.7, 17.7),
}


def signed_rect_dist(px, py, r):
    cx, cy = 0.5 * (r[0] + r[1]), 0.5 * (r[2] + r[3])
    hx, hy = 0.5 * (r[1] - r[0]), 0.5 * (r[3] - r[2])
    dx, dy = abs(px - cx) - hx, abs(py - cy) - hy
    return math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0)


def car_outline(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(-CAR_HL, -CAR_HW), (-CAR_HL, CAR_HW), (CAR_HL, CAR_HW), (CAR_HL, -CAR_HW),
           (-CAR_HL, 0.0), (CAR_HL, 0.0), (0.0, -CAR_HW), (0.0, CAR_HW)]
    return [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in pts]


def parking_obstacles(pk):
    """停车场里带碰撞的件，从 parking_zones.yaml 现算，不写死数字。"""
    out = {}
    bar = pk['barrier']
    sec_w, sec_h = [float(v) for v in bar['section']]
    hs = [float(v) for v in bar['housing_size']]
    for g in pk['gates']:
        px, py = [float(v) for v in g['boom']['pivot']]
        ln = float(g['boom']['length'])
        stub = float(bar['stub_back'])
        # 机箱：整体偏到车道外一侧（gen_parking_world.py 的偏移方向由 axis 决定），
        # 这里取"机箱中心距铰点 ≤ 半个机身"的保守方框
        hx = 0.5 * hs[0] + 0.05
        hy = 0.5 * hs[1] + 0.05
        out['%s_机箱' % g['id']] = (px - hx, px + hx, py - hy, py + hy)
        # 杆：放平沿横杆方向、抬起竖直向上，两种姿态都按扫掠盒算（覆盖中间所有角度）
        sx = 1.0 if g['boom']['axis'][1] < 0 else -1.0
        out['%s_杆放平' % g['id']] = (min(px, px + sx * ln) - 0.06,
                                      max(px, px + sx * ln) + 0.06,
                                      py - 0.06, py + 0.06)
        out['%s_杆抬起' % g['id']] = (px - 0.06, px + 0.06,
                                      min(py, py + sx * ln) - 0.06,
                                      max(py, py + sx * ln) + 0.06)
    med = pk['stripes']['median']
    bx, by = [float(v) for v in med['booth']]
    bw = 0.5 * float(med['booth_size'][0])
    out['岗亭'] = (bx - bw, bx + bw, by - bw, by + bw)
    st = pk['stripes']['gantry']
    for i, (mx, my) in enumerate(st['masts']):
        r = float(st['mast_radius'])
        out['龙门架立杆%d' % i] = (mx - r, mx + r, my - r, my + r)
    return out


def lat_proj(route, x, y):
    """返回 (最小横向绝对值, 航段号, 是否投得上)"""
    n = len(route)
    best = None
    for j in range(n):
        p, q = route[(j - 1) % n], route[j]
        dx, dy = q[0] - p[0], q[1] - p[1]
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        rx, ry = x - p[0], y - p[1]
        along = rx * ux + ry * uy
        if not -OFFLINE_TOL <= along <= L + OFFLINE_TOL:
            continue
        lat = -rx * uy + ry * ux
        if best is None or abs(lat) < abs(best[0]):
            best = (lat, j, along, L)
    if best is None:
        return None, None, False
    return best[0], best[1], True


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
    with open(os.path.join(CFG, 'signal_drive_params.yaml'), encoding='utf-8') as fh:
        cars_cfg = yaml.safe_load(fh)['signal_drive']['cars']
    with open(os.path.join(CFG, 'parking_zones.yaml'), encoding='utf-8') as fh:
        pk = yaml.safe_load(fh)['parking']
    obs = dict(BUILDINGS)
    obs.update(parking_obstacles(pk))
    lot = (float(pk['lot']['x0']), float(pk['lot']['x1']),
           float(pk['lot']['y0']), float(pk['lot']['y1']))

    if not os.path.exists(path):
        print('没有 %s（signal_drive_node 没在跑或 path_csv 改过路径）' % path)
        return 0
    rows = {}
    with open(path, encoding='utf-8-sig', newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                rows.setdefault(r['car'], []).append(
                    (float(r['t']), float(r['x']), float(r['y']), float(r['yaw']),
                     float(r['speed']), r['limited_by']))
            except (KeyError, ValueError):
                continue
    print('轨迹文件 %s' % path)
    print('静态件 %d 个（含停车场道闸/机箱/岗亭/龙门架立杆）' % len(obs))
    print('停车场占地 x %.1f..%.1f / y %.1f..%.1f\n' % lot)
    hdr = ('车', '样本', '时长s', '最大横偏m', '横偏>%.1f占比' % LAT_TOL, '离线样本',
           '进停车场样本', '最小车体间隙m')
    print('%-8s %7s %7s %9s %10s %7s %12s %10s' % hdr)
    events = []
    for car, rs in sorted(rows.items()):
        route = [tuple(p[:2]) for p in (cars_cfg.get(car) or {}).get('route') or []]
        if not rs:
            continue
        worst_lat = (0.0, None)
        over = offline = inlot = 0
        gap = (1e9, None, None, None, None)
        for t, x, y, yaw, v, who in rs:
            if route:
                lat, j, ok = lat_proj(route, x, y)
                if not ok:
                    offline += 1
                    events.append((t, car, '离线(不在任何航段内)', x, y, v, who))
                else:
                    if abs(lat) > abs(worst_lat[0]):
                        worst_lat = (lat, t)
                    if abs(lat) > LAT_TOL:
                        over += 1
            if lot[0] <= x <= lot[1] and lot[2] <= y <= lot[3]:
                inlot += 1
                if inlot == 1:
                    events.append((t, car, '进入停车场占地', x, y, v, who))
            for pt in car_outline(x, y, yaw):
                for name, r in obs.items():
                    d = signed_rect_dist(pt[0], pt[1], r)
                    if d < gap[0]:
                        gap = (d, t, name, x, y)
                    if d < 0.0:
                        events.append((t, car, '车体侵入 %s (%.2f m)' % (name, d),
                                       x, y, v, who))
        print('%-8s %7d %7.0f %9.2f %10.1f%% %7d %12d %10.2f' % (
            car, len(rs), rs[-1][0] - rs[0][0], worst_lat[0],
            100.0 * over / len(rs), offline, inlot, gap[0]))
        if gap[2]:
            print('         最近静态件 %s @ t=%.1f 车在 (%.1f, %.1f)' % (
                gap[2], gap[1], gap[3], gap[4]))
    print('\n事件明细（每类首次 + 间隔 >1 s 的侵入）')
    last = {}
    shown = 0
    for t, car, kind, x, y, v, who in sorted(events):
        key = (car, kind.split('(')[0])
        if key in last and t - last[key] < 5.0:
            continue
        last[key] = t
        shown += 1
        if shown > 40:
            print('  ...（其余 %d 条略）' % (len(events) - shown))
            break
        print('  t=%8.1f %-7s %-34s (%6.2f,%6.2f) v=%.2f %s' % (
            t, car, kind, x, y, v, who))
    if not shown:
        print('  无')


if __name__ == '__main__':
    main()
