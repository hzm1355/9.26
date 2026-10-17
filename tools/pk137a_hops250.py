#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""250 的跳点生成器：把"一步摆 20 m"拆成一串 ≤2 m 的小跳。

拆开是因为 SetModelPose 只改模型根链接，轮子/脚轮还挂在原来的世界位姿上；一步挪太远
就是把全部关节误差攒成一次释放（实测弹到 5~8.5 m/s，memory「摆位弹射与复位风暴」）。
用法：pk137a_hops250.py "起始六分量" 目标x 目标y 目标yaw
"""
import math
import sys


def main():
    parts = sys.argv[1].split()
    if len(parts) < 6:
        print('起始位姿读不到六分量：%r' % sys.argv[1], file=sys.stderr)
        return 2
    x, y = float(parts[0]), float(parts[1])
    yaw = float(parts[5])
    tx, ty, tyaw = (float(v) for v in sys.argv[2:5])
    d = math.hypot(tx - x, ty - y)
    n = max(2, int(math.ceil(d / 2.0)))
    dyaw = ((tyaw - yaw + math.pi) % (2.0 * math.pi)) - math.pi
    for k in range(1, n + 1):
        f = k / n
        print('%.3f %.3f %.4f' % (x + (tx - x) * f, y + (ty - y) * f, yaw + dyaw * f))
    print('# 共 %d 跳，直线距离 %.2f m' % (n, d), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
