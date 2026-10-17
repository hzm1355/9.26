#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#121 —— 一次性看清六车的完整姿态（roll/pitch 不再丢）与是否在沙盘 footprint 内。

为什么要这个：
  gz_read（brake_probe120 / wheel_theta117）为了打印 yaw 只取了四元数的 z、w，
  把 x、y 丢了 —— 这正是 whats_under106 栽过的同一类坑（丢 roll 把地板读成竖砖）。
  #120 实测 car_6 在 (30.876,-18.515) z=0.4000 完全不动，而右轮被 ω=+1.0 指令
  驱到 +3.285 rad/s（指令 3.5 的 94%）车体 ω=0.000 ⇒ 轮子根本没有接地。
  本脚本把 roll/pitch 一起算出来，直接判"翻车/侧躺/掉出沙盘"。

沙盘 footprint（room_33，world 里 model pose 0 0 6.7 1.5708 0 0，box 50 6.7 51.2）：
  世界系 x ∈ [-25, +25]，y ∈ [-25.1, +26.1]，顶面 z = 6.700。
"""
import math
import re
import subprocess
import sys

NUM = r'[-+0-9.eE]+'
CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']
X_LO, X_HI = -25.0, 25.0
Y_LO, Y_HI = -25.1, 26.1
FLOOR_TOP = 6.700


def rpy_of(qx, qy, qz, qw):
    """Gazebo 用 ZYX 内旋；返回 (roll, pitch, yaw) 弧度。"""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qz * qz + qx * qx))
    return roll, pitch, yaw


def gz_read(car):
    p = subprocess.run(['gz', 'model', '-m', car, '-i'],
                       capture_output=True, text=True, timeout=25)
    txt = p.stdout
    m = re.search(r'^pose \{\s*position \{\s*x: (' + NUM + r')\s+y: (' + NUM +
                  r')\s+z: (' + NUM + r')\s*\}\s*orientation \{\s*x: (' + NUM +
                  r')\s+y: (' + NUM + r')\s+z: (' + NUM + r')\s+w: (' + NUM + ')',
                  txt, re.M)
    if not m:
        return None

    def ang(j):
        mm = re.search(r'name: "' + car + '::' + j + r'".{0,400}?angle: (' + NUM + ')',
                       txt, re.S)
        return float(mm.group(1)) if mm else float('nan')
    vals = [float(g) for g in m.groups()]
    r, pch, yaw = rpy_of(*vals[3:7])
    return (vals[0], vals[1], vals[2], r, pch, yaw,
            math.degrees(r), math.degrees(pch), ang('left_wheel_joint'),
            ang('right_wheel_joint'))


def main():
    cars = sys.argv[1:] or CARS
    print('沙盘 footprint: x[%.1f,%.1f] y[%.1f,%.1f] 顶面 z=%.3f' % (
        X_LO, X_HI, Y_LO, Y_HI, FLOOR_TOP))
    print('%-7s %8s %8s %7s  %7s %7s %7s  %8s %8s  %s' % (
        '车', 'x', 'y', 'z', 'roll°', 'pitch°', 'yaw', '左轮角', '右轮角', '状态'))
    for car in cars:
        try:
            v = gz_read(car)
        except Exception as e:
            print('%-7s 读取失败 %s' % (car, e))
            continue
        if v is None:
            print('%-7s gz 没有 pose' % car)
            continue
        x, y, z, r, pch, yaw, rd, pd = v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]
        flags = []
        if not (X_LO <= x <= X_HI and Y_LO <= y <= Y_HI):
            flags.append('出界')
        if abs(rd) > 15.0 or abs(pd) > 15.0:
            flags.append('倾斜')
        if z < FLOOR_TOP - 0.5:
            flags.append('低于地面')
        print('%-7s %8.3f %8.3f %7.4f  %7.1f %7.1f %7.3f  %8.3f %8.3f  %s' % (
            car, x, y, z, rd, pd, yaw, v[8], v[9], ' '.join(flags) or '正常'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
