#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""清点 world 里所有模型的**实际碰撞占据**，输出成 JSON 给审计脚本用。

为什么需要这一份：沙盘 room_33 的网格只有一个"等效盒"碰撞（50x6.7x51.2 的平板，
顶面 z=6.7），画在网格上的路缘石、标志牌、信号灯杆、龙门架、隔离墩在物理上**全是空气**。
任务 #21 为了消掉"隐形碰撞盒侵入车道"，把若干模型的碰撞也删了。于是"车穿过障碍物"
分成了两类，处置完全不同，必须先分清：
  A 类：模型本来有碰撞，车照样过去了 → 控制/几何问题（车不该出现在那儿）。
  B 类：模型压根没有碰撞 → 视觉穿模，物理上无从阻挡，要么补碰撞要么改路线。

用法（WSL）：
  python3 tools/world_collision_inventory.py [world路径] [输出json]
默认输出 /tmp/world_collisions.json，并打印按"距最近车道中心线距离"排序的表。
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')

CAR_L = 0.85      # 车体行进向半长，与 route_corridor_scan.py 一致
CAR_W = 0.50      # 车体横向半宽（含余量），与 route_corridor_scan.py 一致
GROUND = 6.7      # 沙盘路面顶 z（room_33 <pose> 的 z）
CHASSIS = 0.35    # 底盘离地 m：盒底低于路面+0.35 时车能直接压过去，不算挡
OVERHEAD = 0.90   # 车顶 m：盒顶在 0.9 以上（且盒底也高于 0.9）的是龙门架横梁，能从下面过


def gap_to_lane(aabb, legs, center_dist=None):
    """轴对齐碰撞盒与"车体走廊矩形"（航段横向 ±CAR_W、纵向 ±CAR_L）的最短间隙。

    原来只看 lane_dist=盒中心到车道中心线的距离，会把一个 9 m 长的护栏判成
    "离车道 4 m 很安全"——它其实从车道旁边一路擦过去。间隙必须按盒的四个角算。
    gap=0 表示盒已经压在走廊里。
    """
    best = 1e9
    for p, q in legs:
        if abs(q[0] - p[0]) < 1e-9:            # 竖腿：横向是 x，纵向是 y
            rect = (p[0] - CAR_W, p[0] + CAR_W,
                    min(p[1], q[1]) - CAR_L, max(p[1], q[1]) + CAR_L)
        elif abs(q[1] - p[1]) < 1e-9:          # 横腿
            rect = (min(p[0], q[0]) - CAR_L, max(p[0], q[0]) + CAR_L,
                    p[1] - CAR_W, p[1] + CAR_W)
        else:                                   # 斜腿（当前路网全是正交航段）
            if center_dist is None:
                continue
            for cx, cy in ((aabb[0], aabb[2]), (aabb[1], aabb[2]),
                           (aabb[0], aabb[3]), (aabb[1], aabb[3])):
                best = min(best, max(0.0, center_dist(cx, cy) - CAR_W))
            continue
        dx = max(0.0, aabb[0] - rect[1], rect[0] - aabb[1])
        dy = max(0.0, aabb[2] - rect[3], rect[2] - aabb[3])
        best = min(best, math.hypot(dx, dy))
    return best


def parse_pose(e):
    if e is None or e.text is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    v = [float(x) for x in e.text.replace('\n', ' ').split()]
    while len(v) < 6:
        v.append(0.0)
    return tuple(v[:6])


def rot(pose):
    """Gazepose 固定轴 XYZ：pose=(x,y,z,roll,pitch,yaw)，R = Rz(yaw)*Ry(pitch)*Rx(roll)。

    之前把 roll 当成 yaw 传进去，room_33 的台体被算成"竖着的 51.2 m 高墙"，
    整张表的 z 范围全错。姿态角顺序错一位，结论就从"路面实心台"变成"隐形墙"。
    """
    z, y, x = pose[5], pose[4], pose[3]
    cz, sz = math.cos(z), math.sin(z)
    cy, sy = math.cos(y), math.sin(y)
    cx, sx = math.cos(x), math.sin(x)
    return ((cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx),
            (sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx),
            (-sy, cy * sx, cy * cx))


def apply(m, t, v):
    """m 是三行矩阵，先把 v 旋转再平移。"""
    x, y, z = v
    return (m[0][0] * x + m[0][1] * y + m[0][2] * z + t[0],
            m[1][0] * x + m[1][1] * y + m[1][2] * z + t[1],
            m[2][0] * x + m[2][1] * y + m[2][2] * z + t[2])


def mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def box_corners(M, t, half):
    out = []
    for i in (-1, 1):
        for j in (-1, 1):
            for k in (-1, 1):
                out.append(apply(M, t, (i * half[0], j * half[1], k * half[2])))
    return out


def geom_extent(g):
    """返回该几何在自身坐标系里的"半尺寸"（圆柱/球按外接盒）。"""
    b = g.find('box')
    if b is not None:
        return tuple(v * 0.5 for v in [float(x) for x in b.find('size').text.split()])
    c = g.find('cylinder')
    if c is not None:
        r = float(c.find('radius').text)
        h = float(c.find('length').text) * 0.5
        return (r, r, h)
    s = g.find('sphere')
    if s is not None:
        r = float(s.find('radius').text)
        return (r, r, r)
    return None


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else W
    out = sys.argv[2] if len(sys.argv) > 2 else '/tmp/world_collisions.json'
    tree = ET.parse(world)
    # 标准 world 是 <sdf><world><model>…，直接在根上 findall('model') 会一个都取不到。
    root = tree.getroot()
    worlds = root.findall('world') or ([root] if root.tag == 'world' else [])
    models = []
    for wd in worlds:
        models.extend(wd.findall('model'))
        for inc in wd.findall('include'):
            models.extend(inc.findall('model'))
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    legs = []
    for cc in cars.values():
        r = [tuple(p[:2]) for p in cc['route']]
        for j in range(len(r)):
            legs.append((r[(j - 1) % len(r)], r[j]))

    def dist_to_lane(x, y):
        best = 1e9
        for p, q in legs:
            dx, dy = q[0] - p[0], q[1] - p[1]
            L = math.hypot(dx, dy) or 1.0
            ux, uy = dx / L, dy / L
            along = ((x - p[0]) * ux + (y - p[1]) * uy)
            if not 0.0 <= along <= L:
                along = 0.0 if along < 0 else L
            best = min(best, math.hypot(x - p[0] - ux * along,
                                        y - p[1] - uy * along))
        return best

    inv = []      # 每个碰撞盒一条：模型 AABB 会把"立杆+横梁"糊成一大块，逐件才看得出谁挡路
    boxes_all = []
    for m in models:
        name = m.get('name')
        mp = parse_pose(m.find('pose'))
        Mm = rot(mp)
        tm = (mp[0], mp[1], mp[2])
        links = [m] + list(m.findall('link'))
        boxes = []
        mesh_cols = 0
        static = (m.find('static') is not None and
                  m.find('static').text.strip() in ('1', 'true'))
        has_visual = any(l.findall('visual') for l in links)
        for l in links:
            lp = parse_pose(l.find('pose'))
            Ml = mul(Mm, rot(lp))
            tl = apply(Mm, tm, (lp[0], lp[1], lp[2]))
            if l.find('static') is not None and \
                    l.find('static').text.strip() in ('1', 'true'):
                static = True
            for c in l.findall('collision'):
                g = c.find('geometry')
                if g is None:
                    continue
                half = geom_extent(g)
                if half is None:
                    mesh_cols += 1      # mesh 碰撞：有实体但无法逐件判定占据
                    continue
                gp = parse_pose(c.find('pose'))
                Mc = mul(Ml, rot(gp))
                tc = apply(Ml, tl, (gp[0], gp[1], gp[2]))
                cs = box_corners(Mc, tc, half)
                xs = [p[0] for p in cs]
                ys = [p[1] for p in cs]
                zs = [p[2] for p in cs]
                aabb = [round(min(xs), 3), round(max(xs), 3),
                        round(min(ys), 3), round(max(ys), 3),
                        round(min(zs), 3), round(max(zs), 3)]
                cx, cy = 0.5 * (aabb[0] + aabb[1]), 0.5 * (aabb[2] + aabb[3])
                row = {'name': name, 'link': l.get('name'),
                       'coll': c.get('name'), 'aabb': aabb, 'static': static,
                       'size': [round(aabb[1] - aabb[0], 2), round(aabb[3] - aabb[2], 2),
                                round(aabb[5] - aabb[4], 2)],
                       'gap': round(gap_to_lane(aabb, legs, dist_to_lane), 3),
                       'h_bot': round(aabb[4] - GROUND, 2),
                       'h_top': round(aabb[5] - GROUND, 2),
                       'lane_dist': round(dist_to_lane(cx, cy), 2)}
                boxes.append(row)
                boxes_all.append(row)
        if not boxes:
            inv.append({'name': name, 'collision': bool(mesh_cols),
                        'mesh_only': bool(mesh_cols), 'visual': has_visual,
                        'static': static, 'pose': list(mp)})
            continue
        x0 = min(b['aabb'][0] for b in boxes)
        x1 = max(b['aabb'][1] for b in boxes)
        y0 = min(b['aabb'][2] for b in boxes)
        y1 = max(b['aabb'][3] for b in boxes)
        z0 = min(b['aabb'][4] for b in boxes)
        z1 = max(b['aabb'][5] for b in boxes)
        inv.append({'name': name, 'collision': True, 'visual': has_visual,
                    'static': static, 'pose': list(mp),
                    'aabb': [x0, x1, y0, y1, z0, z1],
                    'gap': round(min(b['gap'] for b in boxes), 3),
                    'lane_dist': round(dist_to_lane(0.5 * (x0 + x1), 0.5 * (y0 + y1)), 2),
                    'n_box': len(boxes)})

    with open(out, 'w', encoding='utf-8') as fh:
        json.dump({'models': inv, 'boxes': boxes_all}, fh,
                  ensure_ascii=False, indent=1)

    col = [e for e in inv if e['collision'] and not e.get('mesh_only')]
    mesh = [e for e in inv if e.get('mesh_only')]
    noc = [e for e in inv if not e['collision']]
    print('world %s' % world)
    print('模型 %d 个：有盒碰撞 %d、只有 mesh 碰撞 %d、无碰撞 %d；碰撞盒共 %d 个' % (
        len(inv), len(col), len(mesh), len(noc), len(boxes_all)))

    # 只有落在车体竖直包络内（离地 0.35..0.9 m）的盒才可能挡车或刮底盘；
    # 路面平板（room_33 的 50×51.2×6.7 台体）从地下一直顶到 6.7，天然 gap=0，不参与判定。
    inenv = [e for e in boxes_all if e['static'] and
             e['h_top'] >= CHASSIS and e['h_bot'] <= OVERHEAD]
    blocked = sorted([e for e in inenv if e['gap'] <= 0.0], key=lambda e: e['gap'])
    tight = sorted([e for e in inenv if 0.0 < e['gap'] <= 0.5], key=lambda e: e['gap'])
    print('\n【车体走廊判定】gap = 盒到"航段 ±车体(%.2f×%.2f)"矩形的最短距离，'
          'gap<=0 压在行驶走廊内，0<gap<=0.5 属于贴边（一抖就穿）' % (2 * CAR_L, 2 * CAR_W))
    print('  竖直包络内（离地 %.2f..%.2f m）的盒 %d 个：挡车 %d、贴边 %d' % (
        CHASSIS, OVERHEAD, len(inenv), len(blocked), len(tight)))
    for tag, rows in (('挡车', blocked), ('贴边', tight)):
        for e in rows:
            a = e['aabb']
            print('  %s %-24s/%-12s gap=%6.2f 离地 %.2f..%.2f 尺寸 %.1f×%.1f×%.1f '
                  'x %.1f..%.1f y %.1f..%.1f' % (
                      tag, e['name'], (e['coll'] or '')[:12], e['gap'],
                      e['h_bot'], e['h_top'],
                      e['size'][0], e['size'][1], e['size'][2], a[0], a[1], a[2], a[3]))

    print('\n【碰撞盒】按离车道中心线的距离排序（越近越可能挡车/被车刮）')
    for e in sorted(boxes_all, key=lambda e: e['lane_dist']):
        a = e['aabb']
        print('  %-24s/%-10s lane_dist=%6.2f gap=%6.2f 离地=%5.2f static=%-5s '
              '尺寸 %.1f×%.1f×%.1f x %.1f..%.1f y %.1f..%.1f z %.2f..%.2f' % (
                  e['name'], (e['coll'] or '')[:10], e['lane_dist'], e['gap'],
                  e['h_bot'], e['static'],
                  e['size'][0], e['size'][1], e['size'][2],
                  a[0], a[1], a[2], a[3], a[4], a[5]))
    print('\n【整模型汇总】')
    for e in sorted(col, key=lambda e: e['lane_dist']):
        a = e['aabb']
        print('  %-22s lane_dist=%6.2f  static=%-5s  AABB x %.1f..%.1f y %.1f..%.1f '
              'z %.2f..%.2f  %d 盒' % (
                  e['name'], e['lane_dist'], e['static'], a[0], a[1], a[2], a[3],
                  a[4], a[5], e['n_box']))
    print('\n【只有 mesh 碰撞】占据由网格等效盒决定，逐件不可分：')
    for e in mesh:
        print('  %-22s pose=(%.1f, %.1f, %.1f)' % (
            e['name'], e['pose'][0], e['pose'][1], e['pose'][2]))
    print('\n【无碰撞】这些模型物理上是空气，车可以直接穿过去：')
    for e in noc:
        print('  %-22s pose=(%.1f, %.1f, %.1f)  有 visual=%s' % (
            e['name'], e['pose'][0], e['pose'][1], e['pose'][2], e['visual']))
    print('\nJSON 已写到 %s' % out)


if __name__ == '__main__':
    main()
