#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 car_6 从 gate_out 的近圈里挪出去，让抓拍插件重新武装，并顺手送到车位旁。

为什么必须挪：v2x_gate 的解除条件是"离 focus 超过 far_radius(2.5 m)"
（v2x_gate_plugin.cc:184），而 gate_out-6 那次抓拍之后车一直停在 (5.94,-9.00)，
离焦点 (6.80,-10.95) 只有 2.06 m —— 武装标志还压着。这时直接重启 shuttle，它会
从 exit#3 接着往下跑、以正确朝向穿近圈，但插件因为 armed=true 不再触发第二次抓拍，
manager 拿不到识别结果就不抬杆，30 s 超时 → 第 3 次作废 → 停驶挂起，白烧一轮。

走的这段路全程在两条杆轴（y=-11.05）以北，不越杆；速度只用 0.15 m/s，因为这台
diff_drive 有"驱动节点接手那一下弹射"的前科（#135/#136），每段结束都复核 z，
z 一旦低于 6.5 就说明车被顶出台面，立刻停手而不是继续开。

指令通道自测复用 rearm_by_driving136 的 Driver（turn_to/straight_to/inch 都是
本轮在环验过的），不再抄一份，免得两份实现各自漂移。
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rearm_by_driving136 import Driver, GATE_IN_FOCUS, GATE_OUT_FOCUS, FAR

ROAD_Z = 6.70


class DriverZ(Driver):
    """父类的 pose 只有 (x,y,yaw)，而这段路唯一要防的是"被顶出台面"，所以多存一个 z。"""

    def on_ms(self, m):
        if 'car_6' not in m.name:
            return
        p = m.pose[m.name.index('car_6')]
        from rearm_by_driving136 import yaw_of
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation), p.position.z)


def dist(p, f):
    return math.hypot(p[0] - f[0], p[1] - f[1])


def main():
    rclpy.init()
    d = DriverZ()
    if not d.wait_pose():
        print('  !! 拿不到 car_6 位姿（gzserver 或模型名不对）')
        return 1
    d.hold_zero(2.0)
    n_sub = d.wait_match()
    x0, y0, yaw0, z0 = d.pose
    print('== 起点 (%.2f,%.2f) yaw=%+.2f z=%.3f ｜cmd_vel 订阅端 %d 个 ｜离 gate_out %.2f m =='
          % (x0, y0, yaw0, z0, n_sub, dist(d.pose, GATE_OUT_FOCUS)))
    if n_sub == 0:
        print('  !! 没有订阅端配对，发车也是空发，停手')
        return 3
    moved = d.inch(1.5, 0.15)
    print('  蠕动 1.5 s 位移 %.3f m（应 ≈0.22）→ %s'
          % (moved, '通道有效' if moved > 0.08 else '没动，停手'))
    if moved <= 0.08:
        d.hold_zero(2.0)
        d.destroy_node()
        rclpy.shutdown()
        return 3

    print('== 1) 车头转北，直行到 y>=-8.00（这一步就把 gate_out 解到 2.5 m 外）==')
    d.turn_to(1.5708)
    ok1 = d.straight_to(lambda p: p[1] >= -8.00, 0.15, 1.5708, limit=60.0)
    print('  现在 (%.2f,%.2f) ｜离 gate_out %.2f m → %s'
          % (d.pose[0], d.pose[1], dist(d.pose, GATE_OUT_FOCUS),
             '已解除武装' if dist(d.pose, GATE_OUT_FOCUS) > FAR else '仍在近圈'))
    print('== 2) 转东行，送到车位 P1 西侧 (7.60,-8.00)，让 shuttle 从 entry#6 接手 ==')
    d.turn_to(0.0)
    ok2 = d.straight_to(lambda p: p[0] >= 7.60, 0.15, 0.0, limit=60.0)
    d.hold_zero(2.0)
    x1, y1, yaw1, z = d.pose
    din = dist(d.pose, GATE_IN_FOCUS)
    dout = dist(d.pose, GATE_OUT_FOCUS)
    print('== 终点 (%.2f,%.2f) yaw=%+.2f ｜离 gate_in %.2f m ｜离 gate_out %.2f m（>%s 才算解除：%s）=='
          % (x1, y1, yaw1, din, dout, FAR, '已解除' if dout > FAR else '仍在近圈'))
    print('  z=%.3f（路面 6.70；低于 6.5 说明掉出台面）' % z)
    d.destroy_node()
    rclpy.shutdown()
    return 0 if (ok1 and ok2 and dout > FAR) else 2


if __name__ == '__main__':
    sys.exit(main())
