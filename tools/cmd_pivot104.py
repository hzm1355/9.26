#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#104 判死 angular.z 通道的"到底是什么"：同一水平点上跑符号与幅值矩阵。

#102 已证明 (0, +0.35) 让 car_6 朝车头方向前冲而不是原地转（_cp102b/_cp102d 两发干净
复现，且模型本体与插件参数和 car_1~car_5 逐字相同 —— 所以这不是 car_6 独有）。
但"前冲"有两种截然不同的成因，修法完全相反：
  (A) ω 被当成标量前进速度用（带符号）  → ω 变负应当倒车；
  (B) ω 泄漏成一个与符号无关的共模前进项 → ω 变负照样前进。
再加小信号段测共模增益是否随 |ω| 线性，以及"v 与 ω 同号/异号"的弧线组合到底转不转得动
——这决定 #82/#83 是把 shuttle 的 align 改成弧线转向，还是这条路根本走不通。

段间只在离位 > 1.2 m 时重摆回同一落点，保证六段可比。全程带 #102 的离位/超速护栏。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cmd_pivot102 import Pivot, RADIUS, SEP, SWEEP_R  # noqa: E402
import rclpy  # noqa: E402

IDLE_SECS = 3.0
REST_SECS = 2.0


def seg_report(rows):
    print('  %-22s 走 %5.2f m 转 %+0.3f rad 峰值|v|=%.2f 峰值|ω|=%.2f  '
          '均速 %+.2f m/s（指令 v=%.2f）' % (
              rows['tag'], rows['d'], rows['dyaw'], rows['peak'], rows['peak_w'],
              rows['d'] / rows['secs'], rows['v']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=1.5708)
    ap.add_argument('--pivot-x', type=float, default=10.50, dest='pivot_x')
    ap.add_argument('--pivot-y', type=float, default=-9.55, dest='pivot_y')
    ap.add_argument('--secs', type=float, default=2.5)
    ap.add_argument('--cool', type=float, default=14.0)
    ap.add_argument('--lim-dist', type=float, default=4.0, dest='lim_dist')
    ap.add_argument('--lim-speed', type=float, default=1.5, dest='lim_speed')
    ap.add_argument('--mode', default='pivot', choices=('pivot', 'arcs'),
                    help='pivot=纯 ω 符号幅值矩阵；arcs=车队同款弧线组合（两轮同向）')
    a = ap.parse_args()

    rclpy.init()
    n = Pivot(a.model, a.lim_dist, a.lim_speed)
    if n.wait_pose() is None:
        print('!! 10 s 没收到 /model_states')
        raise SystemExit(2)

    px, py = a.pivot_x, a.pivot_y
    c = n.clearance(px, py)
    print('落点 (%.3f,%.3f)：中心净空 %.2f m（最近 %s），扫掠半径 %.2f m' % (
        px, py, c[1], c[0], SWEEP_R))
    if not n.settle_place(px, py, a.z, a.yaw, a.cool):
        print('!! 摆位静不下来，本发不判 ω 通道')
        raise SystemExit(3)

    print('=== 0) 零速静置 %.1f s 复确认该点不滚坡 ===' % IDLE_SECS)
    d0, dy0, p0 = n.step(0.0, 0.0, IDLE_SECS, '0) 静置', every=3, max_print=6)
    print('  静置：走了 %.2f m，转了 %+0.3f rad，峰值|v|=%.2f' % (d0, dy0, p0))

    # (v, w, tag)：符号对 + 幅值对 + 弧线对
    arcs = [
        (0.0, -0.35, '1) 纯 ω=-0.35 对照'),
        (0.25, -0.35, '2) 弧线 v.25 ω-.35'),
        (0.25, +0.35, '3) 弧线 v.25 ω+.35'),
        (0.31, -0.45, '4) 车队同款 v.31 ω-.45'),
        (0.31, +0.45, '5) 车队同款 v.31 ω+.45'),
    ]
    piv = [
        (0.0, +0.35, '1) 纯 ω=+0.35'),
        (0.0, -0.35, '2) 纯 ω=-0.35'),
        (0.0, +0.10, '3) 纯 ω=+0.10'),
        (0.0, -0.10, '4) 纯 ω=-0.10'),
        (+0.15, +0.35, '5) 弧线 v+.15 ω+.35'),
        (+0.15, -0.35, '6) 弧线 v+.15 ω-.35'),
    ]
    plan = piv if a.mode == 'pivot' else arcs
    if a.mode == 'arcs':
        a.secs = max(a.secs, 4.0)
    got = []
    for v, w, tag in plan:
        # 段间把车请回同一落点，否则后一段是在前一段的冲量残局上测的
        if n.origin is None or math.hypot(n.pose[1] - n.origin[0],
                                          n.pose[2] - n.origin[1]) > 1.2:
            n.publish(0.0, 0.0)
            time.sleep(0.5)
            if not n.settle_place(px, py, a.z, a.yaw, 6.0, tries=3):
                print('  %s：前置重摆失败，跳过' % tag)
                continue
        tl = (v - w * SEP / 2.0) / RADIUS
        tr = (v + w * SEP / 2.0) / RADIUS
        print('=== %s，%.1f s（目标轮速 左 %+.2f 右 %+.2f rad/s）===' % (
            tag, a.secs, tl, tr))
        d, dyaw, peak = n.step(v, w, a.secs, tag, every=1, max_print=30)
        n.publish(0.0, 0.0)
        # 峰值 ω 从段内打印行看不到返回值，这里用"转过的角/时长"近似即可
        got.append({'tag': tag, 'v': v, 'w': w, 'd': d, 'dyaw': dyaw,
                    'peak': peak, 'peak_w': abs(dyaw) / a.secs, 'secs': a.secs})
        seg_report(got[-1])
        n.rest(REST_SECS)

    print('=== 判读 ===')
    for g in got:
        if abs(g['w']) < 1e-9:
            continue
        # 纯 ω 段的共模增益：观测均速 / 指令 ω；带符号，正=与 ω 同号无关的"绝对值泄漏"
        k = (g['d'] / g['secs']) / abs(g['w'])
        turn_frac = abs(g['dyaw']) / (abs(g['w']) * g['secs'])
        verdict = []
        if g['v'] == 0.0:
            if k > 0.5:
                verdict.append('共模前冲增益 k=%.2f（ω 被拿去当前进速度）' % k)
            else:
                verdict.append('无共模前冲 k=%.2f' % k)
            verdict.append('实际转角仅达指令 %.0f%%' % (100 * turn_frac))
        else:
            verdict.append('弧线段：前冲 %+.2f m/s，转角 %.0f%%，'
                           'ω 若有效应看到 yaw 明显跟 %+.2f' % (
                               g['d'] / g['secs'], 100 * turn_frac, g['w']))
        print('  %-22s %s' % (g['tag'], '；'.join(verdict)))

    print('=== 收车：摆回等待位并验静 ===')
    n.settle_place(a.x, a.y, a.z, a.yaw, a.cool)
    p = n.pose
    print('收尾：位姿 (%.3f,%.3f) z=%.4f yaw=%+.3f' % (p[1], p[2], p[4], p[3]))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
