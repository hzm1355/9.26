#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_offset 离线自测：合成数据 + 真实 obstacles.yaml 两条路都走。

为什么单独测这个纯函数：它决定车要不要横移，错了就要么白绕要么真刮杆；而它在 rclpy
节点里，现场跑一轮要几十分钟，合成用例几毫秒就能判对错。

用法（WSL，仓库根，需先 source ROS 与本包）：
  python3 tools/test_obstacle_offset.py
"""
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'v2x_demo'))
from v2x_demo.signal_drive_node import obstacle_offset  # noqa: E402

CLR, LOOK, HALF = 0.65, 6.0, 2.4
# 航段：沿 +x 方向，起点 (0,0)，单位向量 (1,0) → 横向 lat 就是世界 y
EAST = (0.0, 0.0, 1.0, 0.0)


def off(rects, along=5.0, lat=0.0):
    return obstacle_offset([(r[0], r[1], r[2], r[3], 't') for r in rects],
                           CLR, LOOK, HALF, EAST[0], EAST[1], EAST[2], EAST[3],
                           along, lat)


def near(a, b):
    return abs(a - b) < 1e-6


def main():
    fails = []

    def check(tag, cond, got):
        print('  %-34s %s  实得 %s' % (tag, 'OK' if cond else 'FAIL', got))
        if not cond:
            fails.append(tag)

    print('【合成用例】航段沿 +x，clr=%.2f  lookahead=%.0f  lane_half=%.1f' % (
        CLR, LOOK, HALF))
    # 1 无障碍：不偏移
    t, b, ib = off([])
    check('1 无障碍→不偏移', near(t, 0.0) and not b and not ib, (t, b, ib))
    # 2 灯杆在走廊外（中心 lat=0，杆占 y 1.27..1.53，禁行带 y 0.62..2.18）
    t, b, ib = off([(4.87, 5.13, 1.27, 1.53)])
    check('2 杆在边线外→不偏移', near(t, 0.0) and not b and not ib, (t, b, ib))
    # 3 车自己漂进禁行带（lat=1.0 落在 0.62..2.18）→ in_band
    t, b, ib = off([(4.87, 5.13, 1.27, 1.53)], lat=1.0)
    check('3 车压进禁行带→in_band', ib and near(t, 0.0), (t, b, ib))
    # 4 杆压在中心线上（y 0.4..0.6 → 禁行带 -0.25..1.25）→ 从更空的负侧过
    t, b, ib = off([(4.8, 5.2, 0.4, 0.6)])
    check('4 杆压中心线→外让 -0.25', near(t, -0.25) and not b, (t, b, ib))
    # 5 两根杆左右夹路（内面 ±0.4，缝隙 0.8＝正好一个车宽，没有留白）→ 判挤不下
    t, b, ib = off([(4.8, 5.2, 0.4, 0.6), (4.8, 5.2, -0.6, -0.4)])
    check('5 左右夹死→blocked', b, (t, b, ib))
    # 6 前方 20 m 的杆不在视窗内 → 不偏移
    t, b, ib = off([(24.8, 25.2, 0.4, 0.6)], along=5.0)
    check('6 超出前视窗→不偏移', near(t, 0.0) and not b, (t, b, ib))
    # 7 刚过杆（车在杆后方 3 m）→ 不偏移
    t, b, ib = off([(1.8, 2.2, 0.4, 0.6)], along=5.0)
    check('7 已驶过→不偏移', near(t, 0.0) and not b, (t, b, ib))
    # 8 需要外让超过走廊半宽（杆中心 lat=3.0，只能往 -0.25 走）→ 不 blocked
    t, b, ib = off([(4.8, 5.2, 2.6, 3.4)])
    check('8 让向空旷侧→不越界', near(t, 0.0) and not b, (t, b, ib))
    # 9 南北向航段（沿 +y）同样的杆也要能算出来
    t, b, ib = obstacle_offset([(1.27, 1.53, 4.87, 5.13, 't')], CLR, LOOK, HALF,
                               0.0, 0.0, 0.0, 1.0, 5.0, 0.0)
    check('9 换航向等价', near(t, 0.0) and not b, (t, b, ib))

    print('\n【真实清单】src/v2x_demo/config/obstacles.yaml')
    path = os.path.join(ROOT, 'src', 'v2x_demo', 'config', 'obstacles.yaml')
    if not os.path.exists(path):
        check('清单存在', False, path)
    else:
        with open(path, encoding='utf-8') as fh:
            obs = [(o['rect'][0], o['rect'][2], o['rect'][1], o['rect'][3], o['name'])
                   for o in (yaml.safe_load(fh) or {}).get('obstacles') or []]
        print('  共 %d 个静态碰撞盒' % len(obs))
        with open(os.path.join(ROOT, 'src', 'v2x_demo', 'config',
                               'signal_drive_params.yaml'), encoding='utf-8') as fh:
            cars = yaml.safe_load(fh)['signal_drive']['cars']
        worst = None
        hit = []
        for cname, cc in cars.items():
            r = [tuple(p[:2]) for p in cc['route']]
            for j in range(len(r)):
                p, q = r[(j - 1) % len(r)], r[j]
                dx, dy = q[0] - p[0], q[1] - p[1]
                L = math.hypot(dx, dy) or 1.0
                ux, uy = dx / L, dy / L
                # 沿整条航段扫一遍：车严格按中心线走时，哪些盒会要求外让/判死
                for k in range(0, int(L * 4) + 1):
                    along = min(L, k * 0.25)
                    t, b, _ib = obstacle_offset(obs, CLR, LOOK, HALF,
                                                p[0], p[1], ux, uy, along, 0.0)
                    if b:
                        hit.append('%s leg%d @%.1f' % (cname, j, along))
                    if abs(t) > 0.0 and (worst is None or abs(t) > abs(worst[0])):
                        worst = (round(t, 2), '%s leg%d' % (cname, j), round(along, 1))
        print('  中心线严格巡线时被判"挤不下"的采样点：%d %s' % (len(hit), hit[:6]))
        print('  需要外让的最大偏移：%s' % (worst,))
        check('中心线不被判死', not hit, len(hit))

    print('\n%s' % ('全部通过' if not fails else '失败用例：%s' % fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
