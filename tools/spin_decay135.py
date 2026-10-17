#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 原地自转的衰减观测（纯观察，不发任何指令、不摆位）。

为什么值得单独看：cmd_authority135 那轮四个相位（基线/反向 +3.0/零速/停发）实测角速度
全是 -2.3 rad/s，说明"发指令"改写不了它；但同一个量的绝对值从 21:47 的 2.717 掉到了
21:57 的 2.254。会衰减 = 收敛中的伺服（插件在追一个被 SetEntityState 灌出来的巨型轮位
目标），不衰减 = 死锁的残值。两者对"要不要重启 gzserver"的结论完全不同：
  · 若在收敛 → 等它自己收敛到零就能重新用 cmd_vel 正常驾驶，不打断任何在跑的东西；
  · 若不收敛（或收敛到某个非零平台）→ 只能重启 gzserver 复位插件，需要用户同意。

按 30 s 一窗报告窗口内实测角速度（用 /model_states 的朝向差分，±π 绕圈已处理），
并给出每个窗口的线性拟合，看趋势是不是单调朝 0。
"""
import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates

MODEL = 'car_6'
BIN_S = 30.0
TOTAL = 420.0


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def main():
    rclpy.init()
    node = rclpy.create_node('spin_decay135')
    pts = []

    def on_ms(msg):
        if MODEL not in msg.name:
            return
        i = msg.name.index(MODEL)
        pts.append((time.monotonic(), yaw_of(msg.pose[i].orientation),
                    msg.pose[i].position.x, msg.pose[i].position.y))

    q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                   reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(ModelStates, '/model_states', on_ms, q)

    t0 = time.monotonic()
    last_bin = -1
    print('  %8s %8s %8s %10s %s' % ('窗口', '帧', 'ω rad/s', '转速 deg/s',
                                     '车心 (x,y) 半径 m'))
    while time.monotonic() - t0 < TOTAL:
        rclpy.spin_once(node, timeout_sec=0.05)
        b = int((time.monotonic() - t0) // BIN_S)
        if b != last_bin and b > 0 and len(pts) > 20:
            a0, a1 = (b - 1) * BIN_S, b * BIN_S
            win = [p for p in pts if a0 <= p[0] - t0 < a1]
            if len(win) >= 20:
                tot = 0.0
                for x, y in zip(win, win[1:]):
                    d = y[1] - x[1]
                    while d > math.pi:
                        d -= 2.0 * math.pi
                    while d < -math.pi:
                        d += 2.0 * math.pi
                    tot += d
                w = tot / (win[-1][0] - win[0][0])
                # 车心到"旋转中心"的半径用速度/角速度反推，直接证明它是绕差速轴转
                vx = (win[-1][2] - win[0][2]) / (win[-1][0] - win[0][0])
                vy = (win[-1][3] - win[0][3]) / (win[-1][0] - win[0][0])
                r = math.hypot(vx, vy) / abs(w) if abs(w) > 1e-3 else float('nan')
                print('  %8.0f %8d %8.3f %10.1f %10.2f' % (
                    a0, len(win), w, math.degrees(w), r))
            last_bin = b
    node.destroy_node()
    rclpy.shutdown()
    print('  共采 %d 帧（%0.0f s）' % (len(pts), TOTAL))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
