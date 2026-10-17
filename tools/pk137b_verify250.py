#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""250 的落位体检：摆完必须"两次采样差 ~0 且离目标 <0.30 m"才算摆住了。

为什么不用摆位命令的回显：SetModelPose 的返回只说明命令发出去了，不说明物理接受了；
这一轮之前踩过"回显是旧 latch"的坑（memory「生效要用效果复核」「摆位钉不住的机制」）。
自走阈值取 0.05 m/4 s：车真在弹的时候这个数量级一下就越过去。
用法：pk137b_verify250.py "第一次六分量" "第二次六分量" 目标x 目标y
"""
import math
import sys


def six(text):
    parts = text.split()
    if len(parts) < 6:
        return None
    try:
        return [float(t) for t in parts[:6]]
    except ValueError:
        return None


def main():
    a, b = six(sys.argv[1]), six(sys.argv[2])
    if a is None or b is None:
        print('  !! 位姿读不到六分量（摆位后模型可能已被客户端丢掉）')
        return 1
    tx, ty = float(sys.argv[3]), float(sys.argv[4])
    drift = math.hypot(b[0] - a[0], b[1] - a[1])
    off = math.hypot(b[0] - tx, b[1] - ty)
    tilt = math.hypot(b[3], b[4])
    print('  4 s 内自走 %.3f m（应 ~0，大了说明还在弹）｜离目标 %.3f m｜侧倾 %.3f rad' % (
        drift, off, tilt))
    if tilt > 0.30:
        print('  !! 车翻了/嵌地（roll+pitch=%.3f rad），这一步不能当"摆好了"' % tilt)
        return 1
    return 0 if (drift < 0.05 and off < 0.30) else 1


if __name__ == '__main__':
    sys.exit(main())
