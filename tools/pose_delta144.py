#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""比较两行 `gz model -m car_6 -p` 的输出，判摆位之后车有没有被弹走 / 还在自转。

为什么要单独一个文件：这段判据要嵌在 shell 里，用 heredoc 内联 python 会被执行链吃掉
（memory「脚本写成文件勿内联 heredoc」）。
用法：pose_delta144.py "<摆后 x y z R P Y>" "<4s 后 x y z R P Y>"
"""
import math
import sys


def parse(s):
    v = [float(t) for t in s.split()]
    return v[0], v[1], v[2], v[5]


def main():
    if len(sys.argv) < 3:
        sys.stderr.write('要两行位姿\n')
        return 2
    b, c = parse(sys.argv[1]), parse(sys.argv[2])
    d = math.hypot(c[0] - b[0], c[1] - b[1])
    dyaw = c[3] - b[3]
    while dyaw > math.pi:
        dyaw -= 2 * math.pi
    while dyaw < -math.pi:
        dyaw += 2 * math.pi
    print('  4 s 内位移 %.2f m｜z=%.2f｜yaw=%+.3f（Δyaw=%+.3f）' % (d, c[2], c[3], dyaw))
    bad = []
    if d > 0.50:
        bad.append('位移 >0.5 m：轮位误差在摆位瞬间释放了，别继续')
    if c[2] < 6.50:
        bad.append('z <6.5：车掉出台面')
    if abs(dyaw) > 0.30:
        bad.append('yaw 4 s 内变了 %.2f rad：还在自转' % dyaw)
    print('  判读：' + ('通过，可以起摆渡车' if not bad else '不通过 —— ' + '；'.join(bad)))
    return 0 if not bad else 2


if __name__ == '__main__':
    sys.exit(main())
