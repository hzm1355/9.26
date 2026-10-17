#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_3 等待位南移方案（12.00,-12.80 -> -13.95）的净空预检 —— 离线只读 world 文件。

背景（本轮实测）：第二轮进场零抓拍并停驶挂起，成因不是转角撞墙，是等待位原地转
（reset#2）把**底盘原点**甩到离 gate_in focus 只有 0.79 m 的地方，切进 near_radius=1.4
的触发圈误拍一帧（P0019，yaw 164.94°，d=1.4000，识别 0.1671）；闩锁 armed=true 之后
进场车道内 d 最大只有 1.59 < far=2.5，永远清不掉 → 后续所有真实接近全被静音。
修法：等待位南移到 (12.00,-13.95)，静止点 d=2.74 > far（每轮回到等待位自动解锁），
整个转动机动底盘原点 d ≥ 1.94。

为什么另起一个脚本而不是复用 pk177：pk177 的 --spin 把轮轴放在"航点 + 0.4·û(起始
朝向)"，而 parking_shuttle_node 的 pivot_want 用的是"航点 + 0.4·û(该格目标朝向)"，
配对转向点两口径相差 (0.4,-0.4) m —— 拿 pk177 的数读这格会把瞬时中心核错位置
（memory「净空读数先点名位姿叠加口径」）。本脚本口径全部摘自节点源码：

  * pivot_want(x,y,th) = (x,y) + 0.4·û(th)；到达位姿的 pivot = 上一格航点 + 0.4·û(上一格朝向)
  * 原地转瞬时中心在轮轴；底盘原点 = 轮轴 − 0.4·û(当前朝向)
  * 触发判定看的是 chassis **link 原点**到 focus 的距离（v2x_gate_plugin 读
    GetLink(chassis)->WorldPose），不是车体外缘 —— "会不会误拍"与"会不会撞"是两个
    口径：前者用原点距离，后者用车体 OBB(半长0.80/半宽0.40) 与绕轮轴扫掠圆
    r=hypot(1.20,0.40)=1.2649。

轨迹包络（都在 print 里点名）：
  * 相位包络（唯一判据）：复刻控制器实相——①绕到达轮轴原地转（从到达朝向**单向**转到
    bearing−drive_tol，含过冲余量）②沿"到达轮轴→目标轮轴"走（朝向从停点插值到
    bearing，±0.15 扰动）③绕目标轮轴对齐到目标朝向（同样单向，含过冲余量）。
    朝向范围由 brg/err 符号推导，只覆盖控制器真的会经过的方位。
  * 不采用的包络：以轮轴为心的**整圆扫掠**（360°、半径 1.2649）与"轮轴×朝向全举"
    的叉积。两者都把物理不出现的方位算进来——本脚本早期版本对 reset#0 报过
    -0.215（c131 北缘 y=-14.25 vs 整圆南极点 y=-14.465），而 reset#0 在已通过的
    轮次里从没卡过（本轮日志也无顶住记录），是整圆口径的假报（memory「判据脚本
    先用已通过数据校准」）。全组合叉积仅存旁证行，不作判据。

校准（复现本轮已发生的事实；对不上 = 口径错，脚本退出码 2）：
  * 旧等待位静止：chassis 原点 d = 1.59（配置注释里的值）
  * 旧 reset#2 机动：chassis 原点最小 d = 0.79（< near 1.4，所以确实会误拍）

用法（WSL）：python3 tools/pk184_clearprechk308.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
W = os.path.join(ROOT, 'src', 'smart_car_sim', 'worlds', 'v2x_city_scene.world')

AXLE, HL, HW = 0.40, 0.80, 0.40
SWEEP = math.hypot(HL + AXLE, HW)          # 1.2649：绕轮轴原地转的车体扫掠半径
FOCUS_IN = (12.00, -11.21)                  # gate_in 相机 focus（world 配置）
NEAR, FAR = 1.40, 2.50
CRUISE, ACC, SLOW_K = 0.70, 0.60, 0.6       # 巡航 / 轮面加速度 / 降速律 min(cruise, dist/0.6)
DRIVE_TOL = 0.30                            # 节点 PATH_ALIGN_TOL：行车支允许的方位差

IN_OLD = (12.00, -12.80)                    # 旧等待位
IN_NEW = (12.00, -13.95)                    # 新等待位
VERIFY = (12.00, -12.21)                    # 核验位（两边都不动）
# 出口侧复位转点。2026-10-16 pk183 实测：取 -12.80 时相位余量只有 +0.016（c131），
# 小于 arrive_radius 0.12 的正常到位残差——第三轮停位偏南 0.03 就蹭上 c131，绕楔点
# 猎振把 6.3 rad 原地转预算烧尽挂起（前两轮能过纯属残差恰好偏北）。北移 0.25 后
# 扫掠余量约 +0.27，出口链路不受影响：车心 y=-12.55 仍穿出地感占用框南缘 -12.35
# （离场落杆照旧）、车体距 boom_col 南缘 0.65、核验位/抓拍几何不动。
RESET0 = (6.80, -12.55)


def pivot_want(x, y, th):
    return (x + AXLE * math.cos(th), y + AXLE * math.sin(th))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def d_pt_rect(px, py, bx):
    """点到旋转矩形的外距（盒内为 0）。bx = (name, x, y, th, hx, hy)。"""
    dx, dy = px - bx[1], py - bx[2]
    th = bx[3]
    uu = dx * math.cos(-th) - dy * math.sin(-th)
    vv = dx * math.sin(-th) + dy * math.cos(-th)
    return math.hypot(max(0.0, abs(uu) - bx[4]), max(0.0, abs(vv) - bx[5]))


def rect_perim(cx, cy, th, hx, hy, n=24):
    c, s = math.cos(th), math.sin(th)
    pts = []
    for k in range(n):
        t = 4.0 * k / n
        e, f = int(t), t - int(t)
        if e == 0:
            a, b = hx, -hy + 2.0 * hy * f
        elif e == 1:
            a, b = hx - 2.0 * hx * f, hy
        elif e == 2:
            a, b = -hx, hy - 2.0 * hy * f
        else:
            a, b = -hx + 2.0 * hx * f, -hy
        pts.append((cx + a * c - b * s, cy + a * s + b * c))
    return pts


def obb_gap(car, box):
    """车 OBB (cx,cy,th,HL,HW) 与盒 box 的最小间隙（m，负=重叠）。双向采样。"""
    cx, cy, th = car
    g = min(d_pt_rect(px, py, box) for px, py in rect_perim(cx, cy, th, HL, HW))
    c, s = math.cos(th), math.sin(th)
    for px, py in rect_perim(box[1], box[2], box[3], box[4], box[5]):
        dx, dy = px - cx, py - cy
        uu = dx * c + dy * s
        vv = -dx * s + dy * c
        g = min(g, math.hypot(max(0.0, abs(uu) - HL), max(0.0, abs(vv) - HW)))
    return g


def circ_gap(cx, cy, box, r=SWEEP):
    return d_pt_rect(cx, cy, box) - r


def car_at(chassis, th):
    return (chassis[0], chassis[1], th)


def car_at_axle(axle, th):
    return (axle[0] - AXLE * math.cos(th), axle[1] - AXLE * math.sin(th), th)


def step_range(a, b, step):
    n = max(1, int(abs(b - a) / step) + 1)
    return [a + (b - a) * i / float(n) for i in range(n + 1)]


def phased_cars(p0, p1, th_prev, th_target):
    """相位包络：复刻节点的转->走->对齐三段（见文件头）。p0/p1 为轮轴端点。"""
    cars = []
    brg = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    err0 = wrap(brg - th_prev)
    sgn = 1.0 if err0 >= 0.0 else -1.0
    th_stop = th_prev if abs(err0) <= DRIVE_TOL else brg - sgn * DRIVE_TOL
    # ① 绕 p0 原地转（含过冲 0.15）
    for th in step_range(th_prev, th_stop + sgn * 0.15, math.radians(5)):
        cars.append(car_at_axle(p0, th))
    # ② 沿 p0->p1 行车（朝向 th_stop -> brg，各带 ±0.15 扰动）
    n = max(1, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / 0.05) + 1)
    for i in range(n + 1):
        t = i / float(n)
        ax = p0[0] + (p1[0] - p0[0]) * t
        ay = p0[1] + (p1[1] - p0[1]) * t
        th_c = th_stop + (brg - th_stop) * t
        for dth in (-0.15, 0.0, 0.15):
            cars.append(car_at_axle((ax, ay), th_c + dth))
    # ③ 绕 p1 对齐（到 th_target，过冲 0.30 在到达侧）
    err1 = wrap(th_target - brg)
    s1 = 1.0 if err1 >= 0.0 else -1.0
    for th in step_range(brg, th_target + s1 * 0.30, math.radians(5)):
        cars.append(car_at_axle(p1, th))
    return cars


def band_cars(p0, p1, th_a, th_b, step=0.10, nth=9):
    """全组合包络（旁证用）：轮轴在 p0->p1 连线 × 朝向在 [th_a, th_b] 全举。"""
    cars = []
    n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step))
    for i in range(n + 1):
        t = i / float(n)
        ax = p0[0] + (p1[0] - p0[0]) * t
        ay = p0[1] + (p1[1] - p0[1]) * t
        for j in range(max(1, nth)):
            th = th_a if nth < 2 else th_a + (th_b - th_a) * j / float(nth - 1)
            cars.append(car_at_axle((ax, ay), th))
    return cars


def leg_cars(p0, p1, th, dth=0.0):
    """定朝向行驶段：底盘从 p0 到 p1、朝向 th（可给 ±dth 扰动）。"""
    a = (p0[0] + AXLE * math.cos(th), p0[1] + AXLE * math.sin(th))
    b = (p1[0] + AXLE * math.cos(th), p1[1] + AXLE * math.sin(th))
    cars = []
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.10))
    for i in range(n + 1):
        t = i / float(n)
        ax = a[0] + (b[0] - a[0]) * t
        ay = a[1] + (b[1] - a[1]) * t
        for d in ((-dth, 0.0, dth) if dth > 0.0 else (0.0,)):
            cars.append(car_at_axle((ax, ay), th + d))
    return cars


def maneuver_min_d(p0, p1, focus):
    """机动全过程中底盘原点到 focus 的最小距离下界（三圆/线段的最坏组合）。"""
    def cd(p):
        return max(0.0, math.hypot(p[0] - focus[0], p[1] - focus[1]) - AXLE)
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        seg = math.hypot(p0[0] - focus[0], p0[1] - focus[1])
    else:
        t = max(0.0, min(1.0, ((focus[0] - p0[0]) * dx +
                               (focus[1] - p0[1]) * dy) / L2))
        seg = math.hypot(p0[0] + t * dx - focus[0], p0[1] + t * dy - focus[1])
    return min(cd(p0), cd(p1), max(0.0, seg - AXLE))


def load_boxes():
    """与 pk177 相同的提取口径（model+link+collision 位姿叠加；架空件/贴地 slab
    按同一条 z 规则剔除），杆身 boom_col 单列：转动件，文件里是放平的静态投影，
    核验时它被抬起来，不能当拦路件。"""
    tree = ET.parse(W)
    boxes, booms = [], 0
    for model in tree.getroot().findall('world/model'):
        mname = model.get('name')
        if mname in ('room_33', 'room_20', 'room_12', 'ground_plane') \
                or mname.startswith('car_'):
            continue
        mv = [0.0] * 6
        if model.find('pose') is not None:
            mv = [float(v) for v in model.find('pose').text.split()]
        for link in model.findall('link'):
            lv = [0.0] * 6
            lh = list(link)
            if lh and lh[0].tag == 'pose':
                lv = [float(v) for v in lh[0].text.split()]
            for col in link.findall('collision'):
                cv = [0.0] * 6
                if col.find('pose') is not None:
                    cv = [float(v) for v in col.find('pose').text.split()]
                g = col.find('geometry')
                b = g.find('box') if g is not None else None
                cyl = g.find('cylinder') if g is not None else None
                if b is not None:
                    sx, sy, sz = [float(v) for v in b.find('size').text.split()]
                    hx, hy = sx / 2.0, sy / 2.0
                elif cyl is not None:
                    hx = hy = float(cyl.find('radius').text)
                    sz = float(cyl.find('length').text)
                else:
                    continue
                z = mv[2] + lv[2] + cv[2]
                if z - sz / 2.0 > 6.70 + 0.30:
                    continue
                if z - sz / 2.0 <= 6.70 - 0.05 and z + sz / 2.0 <= 6.70 + 0.06:
                    continue
                if 'boom_col' in (col.get('name') or ''):
                    booms += 1
                    continue
                name = '%s/%s/%s' % (mname, link.get('name'), col.get('name'))
                th = mv[5] + lv[5] + cv[5]
                wx = mv[0] + lv[0] + cv[0]
                wy = mv[1] + lv[1] + cv[1]
                boxes.append((name, wx, wy, th, hx, hy))
    return boxes, booms


def min_with_pose(boxes, cars, circs=()):
    """返回 (最小间隙, 盒子名, 达到它的车姿或圆心)。"""
    best = (1e9, '?', None)
    for b in boxes:
        g, pose = 1e9, None
        for c in cars:
            gg = obb_gap(c, b)
            if gg < g:
                g, pose = gg, c
        for cx, cy in circs:
            gg = circ_gap(cx, cy, b)
            if gg < g:
                g, pose = gg, ('circ', cx, cy)
        if g < best[0]:
            best = (g, b[0], pose)
    return best


def main():
    boxes, booms = load_boxes()
    print('world: %s' % W)
    print('参与判定的碰撞盒 %d 个（另剔除杆身 boom_col %d 个：转动件、核验时抬起）'
          % (len(boxes), booms))

    # ── 1) 口径校准：必须复现本轮已发生的事实 ─────────────────────────────
    rest_old = math.hypot(IN_OLD[0] - FOCUS_IN[0], IN_OLD[1] - FOCUS_IN[1])
    p0o = pivot_want(IN_OLD[0], IN_OLD[1], 0.0)
    p1o = pivot_want(IN_OLD[0], IN_OLD[1], math.pi / 2)
    man_old = maneuver_min_d(p0o, p1o, FOCUS_IN)
    ok = True
    print('\n── 校准（对已发生的实测事实）──')
    for label, got, want in (('旧等待位静止 chassis d', rest_old, 1.59),
                             ('旧 reset#2 机动 chassis 最小 d', man_old, 0.79)):
        flag = 'OK' if abs(got - want) < 0.02 else '!! 与实测不符'
        if abs(got - want) >= 0.02:
            ok = False
        print('  %-26s 算得 %.3f，实测 %.2f  %s' % (label, got, want, flag))
    print('  旧机动最小 d %.2f %s near %.2f → %s'
          % (man_old, '≤' if man_old <= NEAR else '>', NEAR,
             '会误拍（与 P0019 d=1.4000 的事件一致）' if man_old <= NEAR
             else '不该误拍（口径存疑）'))
    if not ok:
        print('!! 口径对不上，停止。')
        return 2

    # ── 2) 新几何的距离账 ────────────────────────────────────────────────
    rest_new = math.hypot(IN_NEW[0] - FOCUS_IN[0], IN_NEW[1] - FOCUS_IN[1])
    p0n = pivot_want(IN_NEW[0], IN_NEW[1], 0.0)
    p1n = pivot_want(IN_NEW[0], IN_NEW[1], math.pi / 2)
    man_new = maneuver_min_d(p0n, p1n, FOCUS_IN)
    y_fire = FOCUS_IN[1] - NEAR                     # 进场腿触发线（x=12 直线上）

    def cross_speed(spot):
        pv0 = pivot_want(spot[0], spot[1], math.pi / 2)
        run = abs(pv0[1] - (y_fire + AXLE))
        v_reach = math.sqrt(2.0 * ACC * run)
        v_law = min(CRUISE, abs(VERIFY[1] - y_fire) / SLOW_K)
        return min(v_law, v_reach), run

    v_old, run_old = cross_speed(IN_OLD)
    v_new, run_new = cross_speed(IN_NEW)
    print('\n── 触发几何（chassis 原点到 focus 的距离）──')
    print('  旧等待位: 静止 d=%.2f（在 near %.2f 内=%s；far %.2f 清锁=%s）'
          % (rest_old, NEAR, rest_old <= NEAR, FAR, rest_old > FAR))
    print('  新等待位: 静止 d=%.2f（在 near %.2f 内=%s；far %.2f 清锁=%s，余量 %.2f）'
          % (rest_new, NEAR, rest_new <= NEAR, FAR, rest_new > FAR, rest_new - FAR))
    print('  旧 reset#2 机动最小 d=%.2f（轮轴 p0=(%.2f,%.2f) -> p1=(%.2f,%.2f)）%s'
          % (man_old, p0o[0], p0o[1], p1o[0], p1o[1],
             '会误拍' if man_old <= NEAR else '不误拍'))
    print('  新 reset#2 机动最小 d=%.2f（轮轴 p0=(%.2f,%.2f) -> p1=(%.2f,%.2f)）'
          '，距 near %.2f 余量 %.2f m'
          % (man_new, p0n[0], p0n[1], p1n[0], p1n[1], NEAR, man_new - NEAR))
    print('  触发线 y=%.2f（两方案同一位置：触发几何与取景标定不动）' % y_fire)
    print('  触发线处车速: 旧 %.3f m/s（助跑 %.2f m）、新 %.3f m/s（助跑 %.2f m）'
          % (v_old, run_old, v_new, run_new))

    # ── 3) 世界碰撞盒核查 ────────────────────────────────────────────────
    scen = []

    scen.append(('新等待位静止 OBB (12.00,-13.95,90°)',
                 [car_at(IN_NEW, math.pi / 2)], []))
    scen.append(('旧等待位静止 OBB (12.00,-12.80,90°) 对照',
                 [car_at(IN_OLD, math.pi / 2)], []))
    scen.append(('新 reset#2 机动（相位包络）',
                 phased_cars(p0n, p1n, 0.0, math.pi / 2), []))
    scen.append(('旧 reset#2 机动（相位包络，对照）',
                 phased_cars(p0o, p1o, 0.0, math.pi / 2), []))
    # 停位偏差 ±0.08 一并叠进包络（教训见 RESET0 注释：余量必须盖住到位残差）
    cars0 = []
    for dx, dy in ((0.0, 0.0), (0.08, 0.0), (-0.08, 0.0),
                   (0.0, 0.08), (0.0, -0.08)):
        px, py = RESET0[0] + dx, RESET0[1] + dy
        cars0 += phased_cars(pivot_want(px, py, -math.pi / 2),
                             pivot_want(px, py, 0.0), -math.pi / 2, 0.0)
        cars0.append(car_at((px, py), -math.pi / 2))
    scen.append(('reset#0 机动（出口侧；相位包络含停位偏差 ±0.08）', cars0, []))
    b = math.atan2(IN_NEW[1] - RESET0[1], IN_NEW[0] - RESET0[0])
    scen.append(('reset#1 斜线腿 (6.80,%.2f)->(12.00,-13.95) 航向 %.1f°'
                 % (RESET0[1], math.degrees(b)),
                 leg_cars(RESET0, IN_NEW, b, dth=0.10), []))
    scen.append(('进场腿 (12.00,-13.95)->(12.00,-12.21) 含核验位',
                 leg_cars(IN_NEW, VERIFY, math.pi / 2, dth=0.10), []))

    fails = []
    print('\n── 世界碰撞盒核查（相位包络为主判据；负=吃进；<0 拦路 FAIL，<0.15 告警）──')
    for label, cars, circs in scen:
        g, name, pose = min_with_pose(boxes, cars, circs)
        tag = '!!' if g < 0.0 else ('~ ' if g < 0.15 else 'OK')
        if g < 0.0:
            fails.append(label)
        if isinstance(pose, tuple) and pose and pose[0] == 'circ':
            where = '圆心(%.2f,%.2f)' % (pose[1], pose[2])
        elif pose:
            where = '位姿(%.2f,%.2f,%+.0f°)' % (pose[0], pose[1], math.degrees(pose[2]))
        else:
            where = ''
        print('  [%s] %s：最小 %+.3f m  %s  %s' % (tag, label, g, name, where))

    # 全组合包络（旁证，不作判据；见文件头）
    p0r = pivot_want(RESET0[0], RESET0[1], -math.pi / 2)
    p1r = pivot_want(RESET0[0], RESET0[1], 0.0)
    print('\n── 旁证：全组合包络（叉积上界，不作判据）──')
    for label, cars, circs in (
            ('reset#0', band_cars(p0r, p1r, -math.pi / 2, math.radians(45)), []),
            ('旧 reset#2', band_cars(p0o, p1o, math.pi / 2, math.pi), []),
            ('新 reset#2', band_cars(p0n, p1n, math.pi / 2, math.pi), [])):
        g, name, pose = min_with_pose(boxes, cars, circs)
        print('  %-10s 最小 %+.3f m  %s' % (label, g, name))

    print('\n结论：%s' % ('全部通过，可以按新等待位改航线上线'
                        if not fails else '有 FAIL 项：%s' % '；'.join(fails)))
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
