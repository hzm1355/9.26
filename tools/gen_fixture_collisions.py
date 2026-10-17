#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 room_33 沙盘网格里提取"画在路侧但完全没有 collision"的设施，生成等效盒碰撞。

为什么要它：沙盘只给了一个等效盒（顶面 z=6.7 的平板），画在网格上的路缘石、护栏、
标志牌、树木、建筑边线物理上全是空气 → 车在画面上直接穿过去（B 类穿模）。
OBJ 三角形太多，ODE 的三角网格碰撞不可靠（room_33 正是因为这个才改成等效盒的），
所以这里**只用轴对齐盒体**，绝不用 mesh 碰撞。

几何换算（与 world 里 room_33 的定义一致）：
  模型 pose = 0 0 6.7 1.5708 0 0，mesh scale = 10
  → wx = 10*x，wy = -10*z，高出路面 h = 10*y，wz = 6.7 + h

安全约束（宁可少补、不可补出隐形墙）：
  1. 只取投影与"车道中心线 ±band"相交的三角形，band 默认 3.5 m；
  2. 栅格 0.25 m，按三角形**真实投影**判定占用（矩形-三角形相交测试），
     不做 bbox 填充 → 等效盒不会比网格更宽；
  3. 贪心分解：从左上角起，只在"整行整列都被占用"时外扩，产出的盒严格盖在占用格上；
  4. 每个盒都要与所有车道段做"±clear 禁入带"裁剪（默认 2.0 m = lane_guard 1.5 + 0.5 余量），
     切不进带的碎片直接丢弃 ——比既有 bldg_* 隐形盒的 1.5 m 约定更保守，
     避免等效盒正好压在控制器的横向护栏上、每次擦边都被弹开；
  5. 盒高按实测取 [0, min(hmax, 1.0)]，底面贴路面 6.7：车体最低接触点在 6.70
     （轮半径 0.10、轮 link 高 0.10），所以 0.15 m 的路缘也能真正挡住车；
  6. 与既有碰撞盒面积重叠 >=60% 的盒丢弃（重复碰撞只会加倍接触求解负担）。

用法（WSL）：
  cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
  python3 tools/world_collision_inventory.py            # 先出 /tmp/world_collisions.json
  python3 tools/gen_fixture_collisions.py               # 只出报告，不改 world
  python3 tools/gen_fixture_collisions.py --apply       # 备份后注入 world

注意：**路线一变就必须重跑 --apply**。禁入带是按 config 里的 routes 算的，
新增路线（例如停车场摆渡车 car_6 进场的口子）若正好穿过某批等效盒，车会被自己
补出来的盒挡死。改 routes 后的顺序是：改 config → 重跑本脚本 --apply →
重跑 world_collision_inventory → 重启 gzserver。
"""
import json
import math
import os
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from collections import defaultdict

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.obj')
WORLD = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')
CFG = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'signal_drive_params.yaml')
EXIST = '/tmp/world_collisions.json'
BOXES = '/tmp/fixture_boxes.json'
DROP = '/tmp/fixture_dropped.json'
SDF = os.path.join(ROOT, 'tools', 'fixture_collisions.sdf')
MODEL = 'sandbox_fixtures_coll'
BEGIN = '<!-- BEGIN sandbox_fixtures_coll (tools/gen_fixture_collisions.py 自动生成，勿手改) -->'
END = '<!-- END sandbox_fixtures_coll -->'

S = 10.0
Z0 = 6.7
CELL = 0.25
BAND = 3.5
# 2.0 = lane_guard 1.5 + 0.5 余量。控制器允许车最远跑到横向 1.5 m，
# 等效盒若正好压在 1.5 m 上就成了"贴着护栏的墙"，每次擦边都会顶一下并被推开。
CLEAR = 2.0
HMIN = 0.15
HMAX_BOX = 1.0
X0, X1 = -25.0, 25.0
Y0, Y1 = -21.0, 21.0

# 手工排除矩形（x0, x1, y0, y1）：这一片上的等效盒一律丢弃。
# 为什么要有这一条：本脚本的禁入带只按 signal_drive_params.yaml 的**五车环线**车道算，
# 完全不知道停车场摆渡车的存在。于是它按沙盘"草地_0029 h_raw=1.07"补出的那根
# 1.0 m 高、2.75×0.25 m 的等效盒（world 里曾叫 c130，x 4.25..7.00 / y -14.25..-14.00、
# 无任何 visual）正好横在摆渡车出场后"南口复位原地转"（航点 6.80,-12.80、瞬时中心
# 在轮轴 6.80,-13.20）的扫掠扇形南侧 —— 车头转过目标朝向 0.25 rad 时车尾角压到
# y=-13.955，离它只差 4.5 cm，轮子被顶住、车楔死在 (6.31,-13.37)。2026-10-16 用户
# 明文授权删掉这面隐形墙，world 里已删；这里的矩形与那条盒严格重合（南边界取
# -14.24，只到 c130 与 c131 的分界内侧，保住 c131 与西侧 c127、北侧 c129）：
# 重跑 --apply 不会再把它生成回来。若日后沙盘网格换版、这一带的分格挪了，要重新核对
# 这一矩形覆盖的是不是同一根盒，别把它当成"停车场周围的墙都可以删"。
EXCLUDE = [(4.20, 7.05, -14.24, -14.01)]


def argv_opt(name, default, cast=str):
    for i, a in enumerate(sys.argv):
        if a.startswith(name + '='):
            return cast(a.split('=', 1)[1])
    return default


def lanes():
    with open(CFG, encoding='utf-8') as fh:
        cars = yaml.safe_load(fh)['signal_drive']['cars']
    out = []
    for cc in cars.values():
        r = [tuple(p[:2]) for p in cc['route']]
        for j in range(len(r)):
            p, q = r[(j - 1) % len(r)], r[j]
            out.append((min(p[0], q[0]), max(p[0], q[0]),
                        min(p[1], q[1]), max(p[1], q[1])))
    # 去重：同一边被两辆车各写一次
    return sorted(set(out))


def band_rects(ls, w):
    return [(a - w, b + w, c - w, d + w) for a, b, c, d in ls]


def in_any(px0, px1, py0, py1, rects):
    for a, b, c, d in rects:
        if px0 <= b and px1 >= a and py0 <= d and py1 >= c:
            return True
    return False


def seg_tri(ax, ay, bx, by, cx, cy, rx0, ry0, rx1, ry1):
    """三角形 xy 投影与矩形格是否相交（格很小，用"顶点在框/框角在形内/边相交"三段判）。"""
    for (px, py) in ((ax, ay), (bx, by), (cx, cy)):
        if rx0 <= px <= rx1 and ry0 <= py <= ry1:
            return True
    for (px, py) in ((rx0, ry0), (rx1, ry0), (rx1, ry1), (rx0, ry1)):
        d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
        d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
        d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
        if (d1 >= 0 and d2 >= 0 and d3 >= 0) or (d1 <= 0 and d2 <= 0 and d3 <= 0):
            return True
    return False


def occupancy(rects):
    """流式扫 OBJ，返回 {(ix,iy): (hmax, 材质名)}。"""
    vx, vy, vz = [], [], []
    NC = int((X1 - X0) / CELL)
    NR = int((Y1 - Y0) / CELL)
    cells = {}
    cur = '?'
    ntri = 0
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
                ntri += 1
                if (ntri & 0xFFFFF) == 0:
                    sys.stderr.write('  面 %d  格 %d  %.0fs\n' % (
                        ntri, len(cells), time.time() - t0))
                    sys.stderr.flush()
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
                    axw, bxw, cxw = S * vx[a], S * vx[b], S * vx[c]
                    ayw, byw, cyw = -S * vz[a], -S * vz[b], -S * vz[c]
                    x0 = min(axw, bxw, cxw)
                    x1 = max(axw, bxw, cxw)
                    y0 = min(ayw, byw, cyw)
                    y1 = max(ayw, byw, cyw)
                    if x1 < X0 or x0 > X1 or y1 < Y0 or y0 > Y1:
                        continue
                    if not in_any(x0, x1, y0, y1, rects):
                        continue
                    lo = S * min(vy[a], vy[b], vy[c])
                    ht = S * max(vy[a], vy[b], vy[c])
                    if ht <= 0.05 or lo >= HMAX_BOX:
                        # ht<=0.05：贴地涂装，补了没有物理意义
                        # lo>=HMAX_BOX：只在车顶上方（屋面/梁），挡不住车却会长出隐形墙
                        continue
                    i0 = max(0, int((x0 - X0) / CELL))
                    i1 = min(NC - 1, int((x1 - X0) / CELL))
                    j0 = max(0, int((y0 - Y0) / CELL))
                    j1 = min(NR - 1, int((y1 - Y0) / CELL))
                    if (i1 - i0 + 1) * (j1 - j0 + 1) > 400:
                        # 超大面（整片铺装/大屋顶）：只按顶点+重心+边中点落格，
                        # 避免一格一测试把时间吃掉；这类盒最终也会被面积阈值丢弃
                        pts = ((axw, ayw), (bxw, byw), (cxw, cyw),
                               ((axw + bxw + cxw) / 3.0, (ayw + byw + cyw) / 3.0),
                               ((axw + bxw) / 2.0, (ayw + byw) / 2.0),
                               ((bxw + cxw) / 2.0, (byw + cyw) / 2.0),
                               ((cxw + axw) / 2.0, (cyw + ayw) / 2.0))
                        for px, py in pts:
                            k = (int((px - X0) / CELL), int((py - Y0) / CELL))
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
                            if seg_tri(axw, ayw, bxw, byw, cxw, cyw,
                                       rx0, ry0, rx1, ry1):
                                k = (ix, iy)
                                old = cells.get(k)
                                if old is None or ht > old[0]:
                                    cells[k] = (ht, cur)
            elif c0 == 117 or c0 == 103:
                sp = raw.split(None, 1)
                if len(sp) > 1:
                    cur = sp[1].decode('utf-8', 'replace').strip()
    sys.stderr.write('  OBJ 扫描完：%d 面、%d 格、%.0fs\n' % (
        ntri, len(cells), time.time() - t0))
    sys.stderr.flush()
    return cells


def components(cells):
    occ = {k for k, (h, _m) in cells.items() if h >= HMIN}
    seen = set()
    comps = []
    for k in occ:
        if k in seen:
            continue
        stack = [k]
        seen.add(k)
        comp = []
        while stack:
            ix, iy = stack.pop()
            comp.append((ix, iy))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (ix + dx, iy + dy)
                    if n in occ and n not in seen:
                        seen.add(n)
                        stack.append(n)
        comps.append(comp)
    return comps


def decompose(comp, cells):
    """贪心：按 (iy,ix) 升序取第一个未分配的占用格，只在整行/整列全占用时外扩。"""
    free = set(comp)
    out = []
    while free:
        ix, iy = min(free)
        w = 1
        while (ix + w, iy) in free:
            w += 1
        hgt = 1
        while True:
            row = [(ix + t, iy + hgt) for t in range(w)]
            if all(r in free for r in row):
                hgt += 1
            else:
                break
        ks = [(ix + a, iy + b) for a in range(w) for b in range(hgt)]
        free.difference_update(ks)
        hm = max(cells[k][0] for k in ks)
        mats = defaultdict(int)
        for k in ks:
            mats[cells[k][1]] += 1
        out.append((X0 + ix * CELL, X0 + (ix + w) * CELL,
                    Y0 + iy * CELL, Y0 + (iy + hgt) * CELL, hm,
                    max(mats, key=mats.get), w * hgt))
    return out


def clip_clear(box, bands):
    """把盒子按 ±CLEAR 禁入带逐条切开，返回落在带外的碎片。"""
    x0, x1, y0, y1 = box[:4]
    pieces = [(x0, x1, y0, y1)]
    for a, b, c, d in bands:
        nxt = []
        for px0, px1, py0, py1 in pieces:
            if px1 <= a or px0 >= b or py1 <= c or py0 >= d:
                nxt.append((px0, px1, py0, py1))
                continue
            # 与禁入带相交：切成上/下/左/右四块（带内部分丢弃）
            for q in ((px0, px1, py0, min(py1, c)),
                      (px0, px1, max(py0, d), py1),
                      (px0, min(px1, a), max(py0, c), min(py1, d)),
                      (max(px0, b), px1, max(py0, c), min(py1, d))):
                if q[0] < q[1] - 1e-9 and q[2] < q[3] - 1e-9:
                    nxt.append(q)
        pieces = nxt
        if not pieces:
            break
    return pieces


def area_inter(p, q):
    return max(0.0, min(p[1], q[1]) - max(p[0], q[0])) * \
        max(0.0, min(p[3], q[3]) - max(p[2], q[2]))


def main():
    apply_ = '--apply' in sys.argv
    ls = lanes()
    rects = band_rects(ls, BAND)
    cbands = band_rects(ls, CLEAR)
    sys.stderr.write('车道段 %d 条，候选带 ±%.1f m，禁入带 ±%.1f m\n' % (
        len(ls), BAND, CLEAR))

    cells = occupancy(rects)
    comps = components(cells)
    sys.stderr.write('连通块 %d 个，开始分解\n' % len(comps))

    exist = []
    if os.path.exists(EXIST):
        with open(EXIST, encoding='utf-8') as fh:
            for b in json.load(fh)['boxes']:
                if b['name'] in ('room_33', 'ground_plane'):
                    continue
                a = b['aabb']
                exist.append(((a[0], a[1], a[2], a[3]), b['name']))
    else:
        sys.stderr.write('警告：%s 不存在，跳过与既有碰撞盒的去重\n' % EXIST)

    kept, dropped = [], []
    for comp in comps:
        for bx, bx1, by, by1, hm, mat, ncell in decompose(comp, cells):
            top = min(hm, HMAX_BOX)
            why = None
            if (bx1 - bx < 0.2 or by1 - by < 0.2) and top < 0.6:
                why = '碎片过小'
            elif hm < 0.2:
                why = '高度不足 0.2 m'
            elif ncell * CELL * CELL > 64.0:
                why = '占地 >8x8 m（整片铺装）'
            if why:
                dropped.append({'rect': [bx, by, bx1, by1], 'h': round(hm, 2),
                                'mat': mat, 'why': why})
                continue
            pieces = clip_clear((bx, bx1, by, by1), cbands)
            if not pieces:
                dropped.append({'rect': [bx, by, bx1, by1], 'h': round(hm, 2),
                                'mat': mat, 'why': '整个在 ±%.1f 禁入带内' % CLEAR})
                continue
            if len(pieces) > 1:
                a0 = (bx1 - bx) * (by1 - by)
                for p in pieces:
                    if area_inter(p, (bx, bx1, by, by1)) < 0.25 * a0:
                        dropped.append({'rect': list(p[:4]), 'h': round(hm, 2),
                                        'mat': mat, 'why': '裁禁入带后的窄条'})
            for p in pieces:
                if p[1] - p[0] < 0.2 or p[3] - p[2] < 0.2:
                    dropped.append({'rect': list(p[:4]), 'h': round(hm, 2),
                                    'mat': mat, 'why': '裁剪后过薄'})
                    continue
                ar = (p[1] - p[0]) * (p[3] - p[2])
                cov = 0.0
                for eb, en in exist:
                    ii = area_inter(p, eb)
                    if ii > cov:
                        cov = ii
                if cov >= 0.6 * ar:
                    dropped.append({'rect': list(p[:4]), 'h': round(hm, 2),
                                    'mat': mat, 'why': '与既有碰撞盒重叠 >=60%'})
                    continue
                if in_any(p[0], p[1], p[2], p[3], EXCLUDE):
                    dropped.append({'rect': list(p[:4]), 'h': round(hm, 2),
                                    'mat': mat, 'why': '手工排除 EXCLUDE（摆渡车南口'
                                                       '原地转扇形，见常量处注释）'})
                    continue
                kept.append({'rect': [round(p[0], 3), round(p[2], 3),
                                      round(p[1], 3), round(p[3], 3)],
                             'h': round(top, 2), 'h_raw': round(hm, 2),
                             'mat': mat, 'n_cell': ncell})

    with open(BOXES, 'w', encoding='utf-8') as fh:
        json.dump(kept, fh, ensure_ascii=False, indent=1)
    with open(DROP, 'w', encoding='utf-8') as fh:
        json.dump(dropped, fh, ensure_ascii=False, indent=1)

    def dl(x, y):
        best = 1e9
        for a, b, c, d in ls:
            if a == b:
                t = max(0.0, min(1.0, (y - c) / (d - c))) if d != c else 0.0
                best = min(best, math.hypot(x - a, y - (c + t * (d - c))))
            else:
                t = max(0.0, min(1.0, (x - a) / (b - a))) if b != a else 0.0
                best = min(best, math.hypot(x - (a + t * (b - a)), y - c))
        return best

    def face_dist(r):
        return min(dl(*c) for c in ((r[0], r[1]), (r[2], r[1]),
                                    (r[0], r[3]), (r[2], r[3])))

    print('等效盒候选 %d 个，丢弃 %d 个（栅格 %.2f m，盒高上限 %.1f m，禁入带 ±%.1f m）' % (
        len(kept), len(dropped), CELL, HMAX_BOX, CLEAR))
    print('\n【按材质汇总】n=盒数  h=最高  贴近车道=最小盒面距')
    bym = defaultdict(list)
    for e in kept:
        e['fd'] = face_dist(e['rect'])
        bym[e['mat']].append(e)
    for mat, es in sorted(bym.items(), key=lambda kv: -len(kv[1])):
        print('  %-40s n=%3d  h=%4.2f  最小盒面距 %4.2f  面积 %5.1f m²' % (
            mat[:40], len(es), max(x['h'] for x in es),
            min(x['fd'] for x in es),
            sum((x['rect'][2] - x['rect'][0]) * (x['rect'][3] - x['rect'][1])
                for x in es)))
    print('\n【离车道最近的 30 个盒】（这些是擦边风险最大的，必须 >1.5=lane_guard）')
    for e in sorted(kept, key=lambda e: e['fd'])[:30]:
        r = e['rect']
        print('  %-38s h=%4.2f  %.2f×%.2f @ (%6.2f,%6.2f)  距车道 %5.2f' % (
            e['mat'][:38], e['h'], r[2] - r[0], r[3] - r[1],
            0.5 * (r[0] + r[2]), 0.5 * (r[1] + r[3]), e['fd']))
    cnt = defaultdict(int)
    for e in dropped:
        cnt[e['why']] += 1
    print('\n【丢弃原因】')
    for k, v in sorted(cnt.items(), key=lambda kv: -kv[1]):
        print('  %-28s %d' % (k, v))
    tall = [e for e in kept if e['h'] >= 0.35]
    print('\n其中 h>=0.35 的 %d 个；这些盒一律不进 obstacles.yaml（见 '
          'gen_obstacle_config.py 的排除项）——控制器沿用 #75 已验证的行为，'
          '新盒只当物理兜底' % len(tall))

    # 生成 SDF 片段：一个 static 模型、一个 link 挂全部碰撞盒。
    # 单 link 比每盒一个 link 省掉 96 个刚体；world_collision_inventory 仍逐 collision 报告，
    # 序号与 /tmp/fixture_boxes.json 的次序一致，可按 cNN 回溯材质。
    lines = [BEGIN,
             '    <!-- B 类设施等效盒：从 room_33 沙盘网格投影生成，只挡不画（无 visual）。',
             '         生成脚本 tools/gen_fixture_collisions.py，判据：盒面距车道中心线 >=2.0 m。 -->',
             '    <model name="%s">' % MODEL,
             '      <static>true</static>',
             '      <link name="fixtures">']
    for n, e in enumerate(kept):
        r = e['rect']
        cx, cy = 0.5 * (r[0] + r[2]), 0.5 * (r[1] + r[3])
        sx, sy = r[2] - r[0], r[3] - r[1]
        sz = max(e['h'], 0.2)
        lines.append('        <!-- %s h_raw=%.2f -->' % (e['mat'][:60], e['h_raw']))
        lines.append('        <collision name="c%02d"><pose>%.3f %.3f %.3f 0 0 0</pose>'
                     '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry></collision>'
                     % (n, cx, cy, Z0 + sz / 2, sx, sy, sz))
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append(END)
    with open(SDF, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('\nSDF 片段已写到 %s（未注入 world）' % SDF)

    if not apply_:
        print('加 --apply 才会备份并注入 %s' % WORLD)
        return

    with open(WORLD, encoding='utf-8') as fh:
        text = fh.read()
    if BEGIN in text:
        text = re.sub(re.escape(BEGIN) + r'.*?' + re.escape(END), '', text,
                      flags=re.S)
    ts = time.strftime('%Y%m%d_%H%M%S')
    bak = WORLD + '.pre_fixture_' + ts + '.bak'
    shutil.copy2(WORLD, bak)
    print('已备份 world 到 %s' % bak)
    body = '\n'.join(lines)
    i = text.rindex('</world>')
    text = text[:i] + body + '\n\n    ' + text[i:]
    with open(WORLD, 'w', encoding='utf-8') as fh:
        fh.write(text)
    ET.parse(WORLD)
    print('已注入 %d 个等效盒并通过 XML 解析校验' % len(kept))


if __name__ == '__main__':
    main()
