#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 的 cmd_vel 通路到底还活着没有 —— 一次相位化实验给结论。

为什么要重做一遍：前面用 `ros2 topic pub` 连发 45 s 零速、又发了 20 s 反向 ω=+3.0，
odom 里的 (vx, vy, ω) 三个数纹丝不动（-2.719 → -2.712），看着像"插件聋了"。
但 ros2cli 是一个短命节点，DDS 发现与真实常驻发布者的行为不保证一致，所以这次用
一个正经的 rclpy 发布者，并且**用 /model_states 的朝向自己算角速度**（不采信插件
自报的 odom，插件正在可疑）：

  基线 8 s → 反向 ω=+3.0 持续 14 s → 零速持续 14 s → 停发观察 8 s

判据（都按实测角速度，不看任何回显）：
  ① 基线 ω 与插件 odom 自报 ω 同号同量级 → 说明它确实在执行那条残值；
  ② 反向段 ω 变正 → cmd_vel 通路活着，之前只是 ros2cli 没送进去；
  ③ 反向段 ω 不变 → 插件不再收任何 cmd_vel，这台 gzserver 里的 diff_drive 已失聪，
     在环复验必须先重启 gzserver（这需要用户明确同意）。
"""
import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

MODEL = 'car_6'
BASE, REV, ZERO, POST = 8.0, 14.0, 14.0, 8.0


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class Rec:
    def __init__(self):
        self.samples = []      # (t, yaw, x, y, odom_vx, odom_vy, odom_w)

    def pose(self, yaw, x, y):
        self.samples.append([time.monotonic(), yaw, x, y, None, None, None])

    def odom(self, vx, vy, w):
        if self.samples:
            self.samples[-1][4:7] = (vx, vy, w)


def rate(samples, t0, t1):
    """给定时间段，用朝向增量算平均角速度（自动处理 ±π 绕圈）。"""
    pts = [s for s in samples if t0 <= s[0] <= t1]
    if len(pts) < 3:
        return None, len(pts)
    tot = 0.0
    for a, b in zip(pts, pts[1:]):
        d = b[1] - a[1]
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        tot += d
    span = pts[-1][0] - pts[0][0]
    return (tot / span if span > 0.5 else None), len(pts)


def main():
    rclpy.init()
    node = rclpy.create_node('cmd_authority135')
    rec = Rec()

    def on_ms(msg):
        if MODEL not in msg.name:
            return
        i = msg.name.index(MODEL)
        p = msg.pose[i]
        rec.pose(yaw_of(p.orientation), p.position.x, p.position.y)

    def on_odom(msg):
        t = msg.twist.twist
        rec.odom(t.linear.x, t.linear.y, t.angular.z)

    q_ms = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(ModelStates, '/model_states', on_ms, q_ms)
    node.create_subscription(Odometry, '/car_6/odom', on_odom, 10)
    pub = node.create_publisher(Twist, '/car_6/cmd_vel', 10)

    def drive(v, w, secs, tag):
        t0 = time.monotonic()
        n = 0
        while time.monotonic() - t0 < secs:
            m = Twist()
            m.linear.x, m.angular.z = v, w
            pub.publish(m)
            n += 1
            for _ in range(3):
                rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        return t0, time.monotonic(), n

    print('  等待 /model_states 与 odom 首次到达…')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15 and not (rec.samples and rec.samples[-1][4]):
        rclpy.spin_once(node, timeout_sec=0.1)
    print('  首帧 t=%.1f、yaw=%.2f' % (rec.samples[0][0], rec.samples[0][1])
          if rec.samples else '  !! 15 s 内没采到位姿')

    tb0 = time.monotonic()
    while time.monotonic() - tb0 < BASE:
        rclpy.spin_once(node, timeout_sec=0.1)
    te0 = time.monotonic()

    r0, r1, n1 = drive(0.0, 3.0, REV, 'rev')
    z0, z1, n2 = drive(0.0, 0.0, ZERO, 'zero')
    tp0 = time.monotonic()
    while time.monotonic() - tp0 < POST:
        rclpy.spin_once(node, timeout_sec=0.1)
    tp1 = time.monotonic()

    print('\n=== 实测角速度（由 /model_states 朝向算，不采信插件自报）===')
    for tag, a, b in (('基线（无发布者）', tb0, te0), ('反向 ω=+3.0 段', r0, r1),
                      ('零速段', z0, z1), ('停发后', tp0, tp1)):
        w, n = rate(rec.samples, a, b)
        print('  %-16s %5.1f s：%s 帧，ω=%s' % (
            tag, b - a, n, '%+.3f rad/s' % w if w is not None else '样本不足'))
    print('  发了 %d 条反向、%d 条零速（约 20 Hz，每 50 ms 一条）' % (n1, n2))

    last = rec.samples[-1]
    print('\n  末帧 odom 自报：vx=%.3f vy=%.3f ω=%.3f；位姿 x=%.2f y=%.2f yaw=%+.2f' % (
        last[4] or float('nan'), last[5] or float('nan'), last[6] or float('nan'),
        last[2], last[3], last[1]))
    print('  判据②（通路活着）= 反向段 ω 变正；判据③（插件失聪）= 四段 ω 全都一样')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
