#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行驶审计：车体-建筑间隙 + 停车线守灯合规取证。

  1) 车体按世界 SDF 的碰撞盒 1.6x0.8 m 建模，取 OBB 边界 8 个采样点，
     与 5 栋建筑（同样取自 SDF 的 static 碰撞盒）做带符号距离，负值＝已侵入。
  2) 横向偏移：把位姿投影到当前航段，取横向绝对值，判断是否压到路边。
  3) 守灯合规：对每条 obey 停车线跟踪"纵向剩余距离"，检测过线事件并回查过线瞬间灯色；
     同时统计红灯前的实际停车位置（是否停在 standoff 之外）与绿灯长时间不动。
  4) 全程用 sim 时钟，保证灯态边沿与位姿样本同一时间轴。

用法：python3 audit_drive.py <秒数> <输出json> [采样Hz]
"""
import bisect
import json
import math
import os
import re
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/audit_raw.json'
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']

# 建筑碰撞盒（v2x_city_scene.world 里的 static box，中心±半宽已换算成 AABB）
BUILDINGS = {
    'bldg_school': (-17.0, -3.0, -13.0, 1.0),
    'bldg_nw': (-18.0, -9.0, 8.0, 17.0),
    'bldg_tower_w': (-7.5, -0.5, 10.5, 17.5),
    'bldg_tower_e': (5.5, 12.5, 10.5, 17.5),
    'bldg_ne': (12.5, 21.5, 8.7, 17.7),
}
CAR_HL, CAR_HW = 0.8, 0.4          # 车体半长 / 半宽（SDF 1.6 x 0.8）
SAFE_MARGIN = 0.35                  # 车体与建筑的最小安全间隙


def signed_rect_dist(px, py, r):
    """点到 AABB 的带符号距离（标准式）：负值＝点在矩形内（已侵入）"""
    cx, cy = 0.5 * (r[0] + r[1]), 0.5 * (r[2] + r[3])
    hx, hy = 0.5 * (r[1] - r[0]), 0.5 * (r[3] - r[2])
    dx, dy = abs(px - cx) - hx, abs(py - cy) - hy
    return math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0)


def car_outline(x, y, yaw):
    """车体 OBB 边界采样：4 角点 + 4 边中点"""
    c, s = math.cos(yaw), math.sin(yaw)
    local = [(-CAR_HL, -CAR_HW), (-CAR_HL, CAR_HW), (CAR_HL, CAR_HW), (CAR_HL, -CAR_HW),
             (-CAR_HL, 0.0), (CAR_HL, 0.0), (0.0, -CAR_HW), (0.0, CAR_HW)]
    return [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in local]


class Audit(Node):

    def __init__(self, cfg_path):
        super().__init__('audit_drive', parameter_overrides=[])
        with open(cfg_path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.cars = cfg['signal_drive']['cars']
        # t0 必须在构造时就定好：灯态话题可能比 /model_states 先到，
        # 晚初始化会让第一次边沿用 None 做减法直接崩掉整个采样进程。
        self.t0 = self.get_clock().now().nanoseconds / 1e9
        self.poses = {c: [] for c in self.cars}      # [t, x, y, yaw, v]
        self.limits = {c: [] for c in self.cars}     # [t, 限速, who]
        self.phase_edges = {g: [] for g in GROUPS}   # [t, phase]
        self.last_phase = {g: None for g in GROUPS}
        self.clock_t = self.t0

        self.create_subscription(ModelStates, '/model_states', self.on_models, 10)
        for g in GROUPS:
            self.create_subscription(String, f'/traffic_light/{g}/state',
                                     lambda m, g=g: self.on_phase(g, m), 10)
        self.create_subscription(String, '/signal_drive/status', self.on_status, 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_phase(self, gid, m):
        p = m.data.strip().upper()
        if p == self.last_phase[gid]:
            return
        self.last_phase[gid] = p
        self.phase_edges[gid].append([round(self.now_s() - self.t0, 3), p])

    def on_status(self, m):
        for part in m.data.split(' | '):
            mt = re.match(r'(car_\d+)=\(([-\d.]+),([-\d.]+)\) v=([\d.]+) 限([\d.]+) (\S+)', part)
            if mt and self.t0 is not None:
                self.limits.setdefault(mt.group(1), []).append(
                    [round(self.now_s() - self.t0, 3), float(mt.group(5)), mt.group(6)])

    def on_models(self, msg):
        now = self.now_s()
        if now - self.clock_t < 1.0 / HZ:
            return
        self.clock_t = now
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if name not in self.cars:
                continue
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.poses[name].append([round(now - self.t0, 3),
                                     round(pose.position.x, 3), round(pose.position.y, 3),
                                     round(yaw, 4),
                                     round(math.hypot(twist.linear.x, twist.linear.y), 3)])

    def phase_at(self, gid, t):
        e = self.phase_edges[gid]
        i = bisect.bisect_left([x[0] for x in e], t)
        if i == 0:
            return e[0][1] if e else 'GREEN'
        return e[i - 1][1] if i <= len(e) else (e[-1][1] if e else 'GREEN')


def analyse(a):
    rep = {'采样秒数': round(a.clock_t - a.t0, 1) if a.t0 else 0}
    per_car = {}
    for c, rows in a.poses.items():
        if not rows:
            continue
        route = [tuple(p) for p in a.cars[c]['route']]
        n = len(route)
        worst_clr = (1e9, None, None, None, None)     # 间隙, 时刻, 建筑, x, y
        lat_max = (0.0, None, None, None, None)
        tight = []                        # 间隙低于安全值的样本
        for t, x, y, yaw, v in rows:
            for pt in car_outline(x, y, yaw):
                for bn, br in BUILDINGS.items():
                    d = signed_rect_dist(pt[0], pt[1], br)
                    if d < worst_clr[0]:
                        worst_clr = (d, t, bn, x, y)
                    if d < SAFE_MARGIN:
                        tight.append([round(t, 1), bn, round(d, 2), round(x, 1), round(y, 1)])
            best = None
            for j in range(n):
                p, q = route[(j - 1) % n], route[j]
                dx, dy = q[0] - p[0], q[1] - p[1]
                L = math.hypot(dx, dy) or 1.0
                ux, uy = dx / L, dy / L
                rx, ry = x - p[0], y - p[1]
                along = rx * ux + ry * uy
                lat = -rx * uy + ry * ux
                if not -1.0 <= along <= L + 1.0:
                    continue
                if best is None or abs(lat) < abs(best[0]):
                    best = (lat, j, along, L)
            if best and abs(best[0]) > abs(lat_max[0]):
                lat_max = (best[0], t, best[1], x, y)
        # 停车线合规
        lines = []
        for line in a.cars[c].get('obey') or []:
            gid, lx, ly = line['group'], float(line['x']), float(line['y'])
            head = float(line.get('heading', 0.0))
            ux, uy = math.cos(head), math.sin(head)
            stand = float(line.get('standoff', 2.2))
            prev = None
            cross, series = [], []
            stop_pos = None
            for t, x, y, yaw, v in rows:
                if abs(wrap(yaw - head)) > 1.0:
                    prev = None
                    continue
                along = (lx - x) * ux + (ly - y) * uy
                lat = abs(-(lx - x) * uy + (ly - y) * ux)
                if lat > float(line.get('lane_half', 2.4)) or along > 25.0:
                    prev = None
                    continue
                series.append((t, along))
                ph = a.phase_at(gid, t)
                if prev is not None and prev[0] > 0.0 >= along:
                    # 只有"红灯亮起时车还在可停距离外却仍然过线"才算闯红灯；
                    # 红灯前就已进入路口（黄灯清空）属于正常放行。
                    onset = [e for e in a.phase_edges[gid] if e[1] == ph and e[0] <= t]
                    a_on = along_at(series, onset[-1][0]) if onset else None
                    if ph == 'RED' and v > 0.15 and (a_on is None or a_on > 1.0):
                        kind = '闯灯'
                    elif ph in ('RED', 'YELLOW') and v > 0.15:
                        kind = '清空'
                    else:
                        kind = '越线'
                    cross.append([round(t, 1), ph, round(prev[0], 2), round(v, 2), kind,
                                  a_on])
                if ph in ('RED', 'YELLOW') and 0.0 < along < 6.0 and v < 0.05:
                    if stop_pos is None or along < stop_pos[1]:
                        stop_pos = [round(t, 1), round(along, 2), ph]
                prev = (along, ph)
            red_run = [x for x in cross if x[1] == 'RED' and x[4] == '闯灯']
            yel_run = [x for x in cross if x[1] == 'YELLOW' and x[4] in ('闯灯', '清空')]
            over = [x for x in cross if x[4] == '越线']
            lines.append({'group': gid, '过线次数': len(cross),
                          '闯红灯': len(red_run), '黄灯过线': len(yel_run),
                          '低速越线': len(over),
                          '闯灯明细': red_run[:6],
                          '红灯前最近停车距离_m': (stop_pos[1] if stop_pos else None),
                          '车头最近距离_m': (round(stop_pos[1] - CAR_HL, 2) if stop_pos else None),
                          'standoff_m': stand})
        per_car[c] = {
            '样本数': len(rows),
            '最小车体-建筑间隙_m': round(worst_clr[0], 3),
            '最近建筑': worst_clr[2],
            '间隙告警位置': [round(worst_clr[1], 1), worst_clr[3], worst_clr[4]],
            '低于安全间隙样本数': len(tight),
            '间隙告警样本(前8)': tight[:8],
            '最大横向偏移_m': round(lat_max[0], 2),
            '横偏发生位置': [round(lat_max[1], 1), '航段→%s' % lat_max[2], lat_max[3], lat_max[4]],
            '停车线合规': lines}
    rep['车辆'] = per_car
    rep['灯态边沿数'] = {g: len(a.phase_edges[g]) for g in GROUPS}
    return rep


def wrap(x):
    while x > math.pi:
        x -= 2 * math.pi
    while x < -math.pi:
        x += 2 * math.pi
    return x


def along_at(series, t):
    """红灯亮起那一瞬间车心到停车线的纵向距离：判断"当时还有没有地方刹停"。"""
    if not series:
        return None
    return round(min(series, key=lambda s: abs(s[0] - t))[1], 2)


def main():
    pkg = get_package_share_directory('v2x_demo')
    cfg_path = os.path.join(pkg, 'config', 'signal_drive_params.yaml')
    rclpy.init()
    a = Audit(cfg_path)
    t_end = time.time() + DUR
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(a, timeout_sec=0.1)
    rep = analyse(a)
    rep['原始位姿'] = {c: a.poses[c] for c in a.poses}
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump({'报告': rep, '灯态': a.phase_edges, '限速': a.limits},
                  f, ensure_ascii=False)
    print(json.dumps(rep, ensure_ascii=False, indent=1))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
