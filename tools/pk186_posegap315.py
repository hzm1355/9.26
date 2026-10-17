#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pk185 挂起位姿的接触体检 —— 离线只读 world，口径全部复用 pk184。

要回答的问题很具体：car_3 停在 (11.972,-11.821) yaw 1.748，shuttle 连发 4 次
0.70 m/s 都"走不动"。三种可能（见记忆「不动的三态定性」）：指令没送到 / 插件零出力 /
被几何顶住。本脚本只管第三种：按 OBB 分离轴算车体与世界碰撞盒的间隙，并沿**车头方向**
逐格前探 —— 如果前探第一格就把某个盒吃成负间隙，那就是顶住，且能点名是哪个盒。

用法（WSL）：python3 tools/pk186_posegap315.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pk184_clearprechk308 as pk


def worst(boxes, car, top=3):
    out = sorted(((pk.obb_gap(car, b), b[0]) for b in boxes), key=lambda t: t[0])
    return out[:top]


def main():
    boxes, _booms = pk.load_boxes()
    poses = [
        ('挂起终态 16:29', 11.9723, -11.821, 1.748),
        ('卡住那一刻 16:16:30', 11.9758, -11.8166, 1.75),
        ('抬杆后刚到这格 16:15:55', 11.9373, -11.8773, 1.6857),
        ('核验位（标定值）', 12.00, -12.21, 1.5708),
        ('退回前停格 16:14:44', 11.5231, -12.0888, 1.5573),
    ]
    print('参与判定的碰撞盒 %d 个' % len(boxes))
    for name, x, y, th in poses:
        print('\n[%s] 位姿 (%.3f,%.3f) yaw %.3f rad（%.1f°）' % (
            name, x, y, th, math.degrees(th)))
        for g, nm in worst(boxes, (x, y, th)):
            print('    %+0.3f m  %s' % (g, nm))

    print('\n── 沿车头方向前探（每格 0.05 m，看是不是"往前就吃进"）──')
    x, y, th = 11.9723, -11.821, 1.748
    for i in range(1, 9):
        px, py = x + 0.05 * i * math.cos(th), y + 0.05 * i * math.sin(th)
        g, nm = min(((pk.obb_gap((px, py, th), b), b[0]) for b in boxes))
        print('  +%0.2f m -> (%.3f,%.3f) 最小 %+0.3f m  %s' % (
            0.05 * i, px, py, g, nm))

    print('\n── 改按正北（yaw=1.5708）前探：如果这一支全程为正，卡点是朝向不是位置 ──')
    for i in range(1, 9):
        px, py = x + 0.05 * i * math.cos(1.5708), y + 0.05 * i * math.sin(1.5708)
        g, nm = min(((pk.obb_gap((px, py, 1.5708), b), b[0]) for b in boxes))
        print('  +%0.2f m -> (%.3f,%.3f) 最小 %+0.3f m  %s' % (
            0.05 * i, px, py, g, nm))
    return 0


if __name__ == '__main__':
    sys.exit(main())
