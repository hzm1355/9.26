#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在给定时间窗内用 /model_states 的朝向差分实测 car_6 的角速度（纯观察，不发指令）。

用法：python3 spin_probe_window135.py [窗口秒数=10]
只信 gzserver 回报的位姿，不信插件自报的 odom —— 现在怀疑的就是插件那一路。
"""
import math
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates

MODEL = 'car_6'


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def main():
    win = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = rclpy.create_node('spin_window135')
    pts = []

    def on_ms(m):
        if MODEL not in m.name:
            return
        p = m.pose[m.name.index(MODEL)]
        pts.append((time.monotonic(), yaw_of(p.orientation),
                    p.position.x, p.position.y))

    node.create_subscription(
        ModelStates, '/model_states', on_ms,
        QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                   reliability=ReliabilityPolicy.BEST_EFFORT))
    t0 = time.monotonic()
    while time.monotonic() - t0 < win:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if len(pts) < 5:
        print('  窗口 %.1f s：只采到 %d 帧，判不了' % (win, len(pts)))
        return 1
    tot = 0.0
    for a, b in zip(pts, pts[1:]):
        d = b[1] - a[1]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        tot += d
    span = pts[-1][0] - pts[0][0]
    vx = (pts[-1][2] - pts[0][2]) / span
    vy = (pts[-1][3] - pts[0][3]) / span
    w = tot / span
    print('  窗口 %.1f s：%d 帧，实测 ω=%+.3f rad/s（%.1f deg/s）、车心线速 %.2f m/s、'
          '回转半径 %.2f m' % (span, len(pts), w, math.degrees(w),
                              math.hypot(vx, vy),
                              math.hypot(vx, vy) / abs(w) if abs(w) > 1e-3 else -1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
