#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把"车穿过障碍物"拆成可判定的两类，并给出每一段横偏的成因线索。

背景：截图里能看到车压过路缘、穿过标志牌/灯杆。这些现象有两个完全不同的根因：
  A 类（物理真碰）：static 世界里有实体碰撞盒（5 栋 bldg_*、灯杆、龙门架立杆、岗亭、闸机箱），
      车撞上去会被 ODE 侧向推开 → 表现为"突然横偏几米 + 一格内位置跳变 + 倒速异常"。
  B 类（视觉穿模）：沙盘 room_33 的碰撞只是一个顶面 z=6.7 的等效台，
      画在网格上的路缘石、路面标志牌、隔离墩、树木**根本没有 collision**，
      物理上是空气 → 车直接从画面上"穿过去"，任何控制器都拦不住。

本脚本用 tools/world_collision_inventory.py 产出的碰撞盒清单 + 20 Hz 轨迹 CSV 交叉，
对每段"离开车道中心线 >= 阈值"的窗口给出：持续时长、最大横偏、是否伴位置跳变
（=被弹开/瞬移）、窗口内出现过哪些实体碰撞盒、以及当时的限速来源。

用法（WSL）：
  python3 tools/world_collision_inventory.py           # 先出 /tmp/world_collisions.json
  python3 tools/analyze_deviation.py [横偏阈值m] [csv] [json]
"""
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')
CSV_IN = sys.argv[2] if len(sys.argv) > 2 else '/tmp/signal_drive_path.csv'
JFILE = sys.argv[3] if len(sys.argv) > 3 else '/tmp/world_collisions.json'
THRESH = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
VMAX = 1.0                    # m/s：signal_drive 指令上限（前进 ~0.7 / 倒车 -0.22）

import yaml

DT = 0.05                       # 20 Hz 落盘
CAR_L, CAR_W = 1.6, 0.8         # 车体 SDF 碰撞盒


def segs():
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    out = defaultdict(list)     # car -> [(p0,p1)]
    for name, cc in cars.items():
        r = [(float(p[0]), float(p[1])) for p in cc['route']]
        for j in range(len(r)):
            out[name].append((r[(j - 1) % len(r)], r[j]))
    return out


def lat_of(x, y, legs):
    """返回 (带符号横偏, 航段序号, 纵向剩余)。取绝对值最小的投影航段。"""
    best = None
    for j, (p, q) in enumerate(legs):
        dx, dy = q[0] - p[0], q[1] - p[1]
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        rx, ry = x - p[0], y - p[1]
        along = rx * ux + ry * uy
        if not -1.0 <= along <= L + 1.0:
            continue
        lat = -rx * uy + ry * ux
        if best is None or abs(lat) < abs(best[0]):
            best = (lat, j, L - along)
    return best


def static_boxes():
    """实体障碍盒：static 且离车道中心线 12 m 以内，排除 room_33 路面台和车自身。"""
    with open(JFILE, encoding='utf-8') as fh:
        data = json.load(fh)
    boxes = data['boxes'] if isinstance(data, dict) else data
    out = []
    for b in boxes:
        if not b.get('static'):
            continue
        if b['name'] in ('room_33', 'ground_plane'):
            continue
        a = b['aabb']
        if a[5] < 6.8 or a[4] > 8.2:        # 不与车体高度带相交的（如横梁在上头）单独标注
            pass
        out.append(b)
    return out


def overlaps(box, x, y, yaw):
    """车体 OBB 的外接 AABB 与盒是否相交（盒本身按 AABB 处理）。"""
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    h = 0.5 * (CAR_L * c + CAR_W * s)
    w = 0.5 * (CAR_L * s + CAR_W * c)
    a = box['aabb']
    return not (x + h < a[0] or x - h > a[1] or y + w < a[2] or y - w > a[3])


def z_hit(box):
    a = box['aabb']
    return a[4] < 7.45 and a[5] > 7.05      # 与车体 z 带有重叠才算真会撞上


def main():
    legs = segs()
    boxes = static_boxes()
    rows = defaultdict(list)
    with open(CSV_IN, newline='', encoding='utf-8-sig') as fh:
        for r in csv.DictReader(fh):
            try:
                rows[r['car']].append(
                    (float(r['t']), float(r['x']), float(r['y']), float(r['yaw']),
                     float(r['speed']), r['limited_by'] or ''))
            except (TypeError, ValueError):
                continue
    print('轨迹 %s：车 %d 辆，样本 %d，横偏阈值 %.1f m' % (
        CSV_IN, len(rows), sum(len(v) for v in rows.values()), THRESH))
    print('实体碰撞盒 %d 个（已排除 room_33 路面台/车体）\n' % len(boxes))

    win_all = []
    near = {}
    zbox = [b for b in boxes if z_hit(b)]
    print('与车体高度带（z 7.05~7.45）重叠的实体盒 %d 个：%s\n' % (
        len(zbox), ', '.join('%s/%s' % (b['name'], b['coll'] or '') for b in zbox)))
    nb = {}
    for car in sorted(rows):
        lst = []
        for b in zbox:
            a = b['aabb']
            key = (car, '%s/%s' % (b['name'], b['coll'] or ''))
            lst.append((0.5 * (a[0] + a[1]), 0.5 * (a[2] + a[3]),
                        0.5 * (a[1] - a[0]), 0.5 * (a[3] - a[2]), key))
        nb[car] = lst
    for car in sorted(rows):
        rs = rows[car]
        prev = None
        jumps = 0
        cur = None
        lat_max = 0.0
        for t, x, y, yaw, v, lim in rs:
            g = lat_of(x, y, legs[car])
            off = g is None
            lat = g[0] if g else 0.0
            if abs(lat) > abs(lat_max):
                lat_max = lat
            jump = prev is not None and math.hypot(x - prev[1], y - prev[2]) > 0.5
            if jump:
                jumps += 1
            prev = (t, x, y)
            cand = [b for b in boxes if z_hit(b)]
            hit = [b['name'] + '/' + (b['coll'] or '') for b in cand
                   if overlaps(b, x, y, yaw)]
            if off or abs(lat) >= THRESH or hit or jump:
                if cur is None:
                    cur = {'car': car, 't0': t, 't1': t, 'lat': lat, 'hit': Counter(),
                           'lim': Counter(), 'off': 0, 'jump': 0,
                           'neg': 0.0, 'xs': [], 'ys': []}
                cur['t1'] = t
                if abs(lat) > abs(cur['lat']):
                    cur['lat'] = lat
                for h in hit:
                    cur['hit'][h] += 1
                if lim:
                    cur['lim'][lim.split(':')[0]] += 1
                cur['off'] += 1 if off else 0
                cur['jump'] += 1 if jump else 0
                if v < -0.3:
                    cur['neg'] = min(cur['neg'], v)
                cur['xs'].append(x)
                cur['ys'].append(y)
            else:
                if cur is not None:
                    win_all.append(cur)
                cur = None
            # 全程逐格记录车体中心到每个实体盒的带符号最小距离：
            # 只在异常窗口里统计会漏掉"沿着盒面磨过去"的刮擦，而那正是截图里最常见的观感。
            for cx, cy, hx, hy, key in nb[car]:
                dx = abs(x - cx) - hx
                if dx > 8.0:
                    continue
                dy = abs(y - cy) - hy
                if dy > 8.0:
                    continue
                d = math.hypot(dx, dy) if dx > 0.0 and dy > 0.0 else max(dx, dy)
                if d < 8.0 and (key not in near or d < near[key][0]):
                    near[key] = (round(d, 2), round(t, 1), round(x, 1), round(y, 1))
        if cur is not None:
            win_all.append(cur)
        print('%s：样本 %d，最大横偏 %+.2f m，位置跳变格数 %d（>0.5 m/格＝被弹开或瞬移）'
              % (car, len(rs), lat_max, jumps))

    dur = lambda c: c['t1'] - c['t0']
    keep = [c for c in win_all if dur(c) >= 0.4 or c['hit'] or c['jump'] or c['off']]
    print('\n异常窗口 %d 段（持续>=0.4s 或伴实体盒相交/跳变/越界），合计 %.0f s' % (
        len(keep), sum(dur(c) for c in keep)))
    box_hits = Counter()
    for c in keep:
        box_hits.update(c['hit'])
    print('\n按实体碰撞盒汇总外接盒相交格数（20 格＝1 s）：')
    for k, n in box_hits.most_common(20):
        print('  %-30s %5d 格' % (k, n))
    if not box_hits:
        print('  （无：说明异常窗口里没有一件实体盒与车体重叠——全是 B 类视觉穿模）')
    print('\n最长的 20 段异常窗口：')
    for c in sorted(keep, key=dur, reverse=True)[:20]:
        print('  %s t %8.1f→%8.1f (%5.1fs) 横偏%+6.2f 越界%d 跳变%d 倒速%.2f x %.1f..%.1f y %.1f..%.1f 相交[%s] 限速%s' % (
            c['car'], c['t0'], c['t1'], dur(c), c['lat'], c['off'], c['jump'], c['neg'],
            min(c['xs']), max(c['xs']), min(c['ys']), max(c['ys']),
            ','.join(k for k, _ in c['hit'].most_common(3)) or '-',
            ','.join('%s:%d' % kv for kv in c['lim'].most_common(2)) or '-'))
    print('\n分车看贴/穿了哪些盒：')
    per = defaultdict(Counter)
    pen = {}
    for c in keep:
        per[c['car']].update(c['hit'])
    for car in sorted(per):
        print('  %s：%s' % (car, ', '.join('%s×%d' % kv for kv in per[car].most_common(6)) or '无'))
    print('\n异常窗口合计 %d 段' % len(win_all))
    print('\n车体中心到各实体盒的最小带符号距离（负＝中心真的进了障碍物体积内，物理上必然被推开）：')
    ins = [(k, v) for k, v in near.items() if v[0] < 0.0]
    print('  发生过侵入的组合：%d 个' % len(ins))
    for (car, b), (d, t, x, y) in sorted(ins, key=lambda kv: kv[1][0]):
        print('  %-6s %-30s %6.2f m @t=%.1f (%.1f, %.1f)   ← 侵入' % (car, b, d, t, x, y))
    print('  最近擦边（未侵入）前 10：')
    for (car, b), (d, t, x, y) in sorted([kv for kv in near.items() if kv[1][0] >= 0],
                                         key=lambda kv: kv[1][0])[:10]:
        print('  %-6s %-30s %6.2f m @t=%.1f (%.1f, %.1f)' % (car, b, d, t, x, y))
    print('\nA 类（有实体碰撞仍被穿过）判据：车体中心距盒 < -0.05 m 且当时仍在前进。')

    # ── 超物理速度样本归因 ──────────────────────────────────────────────────
    # signal_drive 的指令上限：前进 ~0.7（cruise）、倒车 -0.22（escape）。
    # 因此 |v|>1.0 只有三种来源：被别的车撞、被静态盒顶开、或 reset_car 瞬移
    # （瞬移时 CSV 里的速度取自 /model_states 的 twist，是上一格接触冲量的残留）。
    print('\n【超物理速度样本归因】|v| > %.2f m/s 的格数与最近邻' % VMAX)
    by_t = defaultdict(dict)
    for car, rs in rows.items():
        for r in rs:
            by_t[round(r[0], 2)][car] = r
    caused = Counter()
    shown = 0
    for car in sorted(rows):
        for t, x, y, yaw, v, lim in rows[car]:
            if abs(v) <= VMAX:
                continue
            nb_car, d_car = None, 9e9
            for oc, r in by_t.get(round(t, 2), {}).items():
                if oc == car:
                    continue
                d = math.hypot(x - r[1], y - r[2])
                if d < d_car:
                    nb_car, d_car = oc, d
            d_box, nb_box = 9e9, None
            for cx, cy, hx, hy, key in nb[car]:
                dx = abs(x - cx) - hx
                dy = abs(y - cy) - hy
                d = math.hypot(dx, dy) if dx > 0.0 and dy > 0.0 else max(dx, dy)
                if d < d_box:
                    d_box, nb_box = d, key[1]
            if d_car < 1.4:
                why = '车-车碰撞（%s 距 %.2f m）' % (nb_car, d_car)
            elif d_box < 0.9:
                why = '被静态盒顶开（%s 距 %.2f m）' % (nb_box, d_box)
            else:
                why = '无邻接物 → 复位瞬移/时线残留'
            caused[why.split('（')[0]] += 1
            if shown < 40:
                shown += 1
                print('  %-6s t=%8.1f v=%+5.2f (%6.2f,%6.2f) 最近车 %s 最近盒 %s 限速 %-14s → %s' % (
                    car, t, v, x, y,
                    '%.2f' % d_car if nb_car else '  - ',
                    '%.2f' % d_box, lim or '-', why))
    print('  归因合计：%s' % dict(caused))




if __name__ == '__main__':
    main()
