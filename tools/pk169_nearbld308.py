#!/usr/bin/env python3
"""169：列出停车场四周的实体与位姿，用来定"前面那几栋"到底是哪些。

口径要点：只取**顶层 model 的 world 位姿**，再按各自 collision 盒的半尺寸把 footprint
画出来 —— 净空判断要的是"占地"，不是"中心离得多远"（memory「接触几何而非中心距」）。
inertial/visual 的 pose 是相对父 link 的，这里只统计带 collision 的 link，并把 collision
的局部 pose 叠加上去（memory「净空读数先点名位姿叠加口径」）。

用法：python3 pk169_nearbld308.py [world 路径]
"""
import os
import sys
import xml.etree.ElementTree as ET

W = sys.argv[1] if len(sys.argv) > 1 else (
    '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world')

# 停车场占地（gen_parking_world.py 的判定框）：x 6.50..16.50、y -12.80..-7.00
LOT = (6.50, 16.50, -12.80, -7.00)
PAD = 6.0   # 往四周各看 6 m


def nums(el, n):
    if el is None or not (el.text or '').strip():
        return None
    v = [float(x) for x in el.text.replace(',', ' ').split()]
    return v[:n] if len(v) >= n else None


def box_of(model):
    """把该 model 的所有 collision 盒叠成 world 帧的轴对齐外接矩形。"""
    mp = nums(model.find('pose'), 6) or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    out = None
    for l in model.findall('link'):
        for c in l.findall('collision'):
            size = None
            for tag in ('box', 'cylinder'):
                e = c.find('geometry/%s' % tag)
                if e is not None:
                    if tag == 'box':
                        size = nums(e.find('size'), 2)
                    else:
                        r = nums(e.find('radius'), 1)
                        size = [r[0], r[0]] if r else None
                    break
            if size is None:
                continue
            cp = nums(c.find('pose'), 6) or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            lp = nums(l.find('pose'), 6) or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # 只处理不转动的摆放（world 里的建筑全是轴对齐的），转动的另说
            hx, hy = size[0] / 2.0, size[1] / 2.0
            x = mp[0] + lp[0] + cp[0]
            y = mp[1] + lp[1] + cp[1]
            r = (x - hx, x + hx, y - hy, y + hy)
            out = r if out is None else (min(out[0], r[0]), max(out[1], r[1]),
                                         min(out[2], r[2]), max(out[3], r[3]))
    return out


def main():
    root = ET.parse(W).getroot()
    print('world=%s' % W)
    print('停车场占地 x %.2f..%.2f  y %.2f..%.2f，往外看 %.1f m' % (
        LOT[0], LOT[1], LOT[2], LOT[3], PAD))
    rows = []
    for m in root.iter('model'):
        nm = m.get('name') or ''
        b = box_of(m)
        if b is None:
            continue
        if (b[0] < LOT[1] + PAD and b[1] > LOT[0] - PAD and
                b[2] < LOT[3] + PAD and b[3] > LOT[2] - PAD):
            rows.append((nm, b))
    for nm, b in sorted(rows, key=lambda r: -((r[1][1] - r[1][0]) * (r[1][3] - r[1][2]))):
        gx = max(0.0, LOT[0] - b[1], b[0] - LOT[1])
        gy = max(0.0, LOT[2] - b[3], b[2] - LOT[3])
        print('  %-22s x %+.2f..%+.2f  y %+.2f..%+.2f  占地 %.2fx%.2f  距场 %+.2f/%+.2f' % (
            nm, b[0], b[1], b[2], b[3], b[1] - b[0], b[3] - b[2], gx, gy))
    print('共 %d 个带碰撞盒的实体落在这个窗口里' % len(rows))
    print('静态 world=%s  存在=%s' % (os.path.basename(W), os.path.exists(W)))


if __name__ == '__main__':
    main()
