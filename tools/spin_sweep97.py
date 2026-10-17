#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 指令→轮速映射扫描：纯角速度为什么会开出一个 5 m/s 的前进。

#95 的结论（_sa95.txt A2 段）：从 yaw=0 的干净摆位出发、全场只有本探针发指令，
只发 angular.z=+0.35（linear.x=0）时，**真值与 /car_6/odom 同时**报
  v=+2.13 → +5.13 m/s，ω=+0.03 → +0.38 rad/s
odom 是由左右轮关节速度反推的，它和真值一起说"车在往前冲" ⇒ 两个驱动轮真的在同向前转，
不是接触推挤、也不是位姿差分口径。这一发要把映射关系量出来：

  · G 段 零速对照 —— 负样本，实测必须 |v|≈0 |ω|≈0，否则后面数字全不可用
  · L 段 linear.x=0.25 / ω=0 —— 线速度通道本身对不对（对照）
  · R 段 ω=+0.05/+0.10/+0.35/-0.35 —— 前冲量级随 ω 怎么走、符号跟不跟
      v_forward ∝ ω → 插件把 angular.z 当成了线速度（换算/参数错）
      只有大 ω 才前冲、小 ω 不动 → 死区/摩擦 + 单向饱和
      负 ω 也前冲 → 取了绝对值
  · 每一段单独统计**收到的** /car_6/cmd_vel 最大值：同进程的订阅会收到自己发布的消息，
      所以判据是"收到的 |linear.x| 是否超过本段发送值"，超了才说明另有发布者灌指令

每段之后先刹轮 1.5 s 再摆回 (12.00,-12.80,6.70) yaw=0，避免上一段的残骸污染下一段
（#95 的 B 段就是从"停在楔住处"直接摆位，被 ODE 顶出去 7.4 m）。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState

BEST = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  history=HistoryPolicy.KEEP_LAST)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Sweep(Node):
    def __init__(self, model):
        super().__init__('spin_sweep')
        self.model = model
        self.pose = None            # (sim_t, x, y, yaw, z)
        self.odom = (0.0, 0.0)
        self.clock = 0.0
        # 本段收到的 cmd_vel 统计（帧数, 最大 |linear.x|, 最大 |angular.z|）。
        # ROS 2 同进程的订阅**会**收到自己发布的消息，所以"非零即外来"是错的判据，
        # 真正的判据是"收到的最大值是否超过本段发送值"，每段开头复位一次。
        self.received = (0, 0.0, 0.0)
        self.create_subscription(ModelStates, '/model_states', self.on_states, BEST)
        self.create_subscription(Odometry, '/%s/odom' % model, self.on_odom, 10)
        self.create_subscription(Clock, '/clock', self.on_clock, BEST)
        self.create_subscription(Twist, '/%s/cmd_vel' % model, self.on_cmd, BEST)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            pass
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % model, 10)

    def on_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(self, m):
        self.odom = (float(m.twist.twist.linear.x), float(m.twist.twist.angular.z))

    def on_cmd(self, m):
        # 同进程的订阅也会收到自己发的消息，所以这里不能按"非零即外来"判。
        # 判据改成：本段收到的最大值若超过本段发送值，就是别人在灌指令。
        self.received = (self.received[0] + 1,
                         max(self.received[1], abs(float(m.linear.x))),
                         max(self.received[2], abs(float(m.angular.z))))

    def on_states(self, m):
        try:
            i = m.name.index(self.model)
        except ValueError:
            return
        p = m.pose[i]
        self.pose = (self.clock, p.position.x, p.position.y,
                     yaw_of(p.orientation), p.position.z)

    def wait_pose(self, secs=8.0):
        end = time.time() + secs
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pose

    def place(self, x, y, z, yaw):
        r = SetEntityState.Request()
        r.state = EntityState()
        r.state.name = self.model
        r.state.pose.position.x = x
        r.state.pose.position.y = y
        r.state.pose.position.z = z
        r.state.pose.orientation.z = math.sin(yaw / 2.0)
        r.state.pose.orientation.w = math.cos(yaw / 2.0)
        r.state.reference_frame = 'world'
        f = self.cli.call_async(r)
        rclpy.spin_until_future_complete(self, f, timeout_sec=8.0)
        return bool(f.result() and f.result().success)

    def send(self, v, w, secs, tag):
        end = time.time() + secs
        last = self.pose
        peak_v = peak_w = 0.0
        ov = ow = 0.0
        start = self.pose
        self.received = (0, 0.0, 0.0)
        while time.time() < end:
            t = Twist()
            t.linear.x = v
            t.angular.z = w
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.01)
            self.pub.publish(t)
            if self.pose is None or last is None:
                continue
            dt = self.pose[0] - last[0]
            if dt > 1e-9:
                peak_v = max(peak_v, math.hypot(self.pose[1] - last[1],
                                                self.pose[2] - last[2]) / dt)
                peak_w = max(peak_w, abs(wrap(self.pose[3] - last[3]) / dt))
                ov, ow = self.odom
                last = self.pose
        d = math.hypot(self.pose[1] - start[1], self.pose[2] - start[2]) if start else 0
        dyaw = wrap(self.pose[3] - start[3]) if start else 0
        # 本段发出去的就是 (v, w)，收到的最大 |linear.x| 只要不超过它就没有外来指令；
        # 超了说明还有别的发布者在灌大线速度，那前冲就不是插件问题。
        over = self.received[1] > abs(v) + 1e-6 or self.received[2] > abs(w) + 1e-6
        print('  %-14s v*=%+.2f ω*=%+.2f  峰值 真值|v|=%.2f |ω|=%.2f  '
              '末帧 odom v=%+.2f ω=%+.2f  段内走了 %.2f m 转了 %+0.2f rad  '
              '收到 %d 条 cmd，最大 |v|=%.2f |ω|=%.2f%s' % (
                  tag, v, w, peak_v, peak_w, ov, ow, d, dyaw,
                  self.received[0], self.received[1], self.received[2],
                  '  !! 有超出本段发送值的外来指令' if over else ''))
        return self.pose, peak_v, peak_w, over

    def brake(self, secs, x, y, z, yaw):
        """刹轮 → 摆回原点 → 再压零速，把上一段的残骸清掉再进下一段。"""
        end = time.time() + secs
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        ok = self.place(x, y, z, yaw)
        end = time.time() + 1.5
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--secs', type=float, default=2.0)
    a = ap.parse_args()
    rclpy.init()
    n = Sweep(a.model)
    p = n.wait_pose()
    if p is None:
        print('!! 8 s 没收到 /model_states')
        raise SystemExit(2)
    print('开场：仿真钟 %.3f（0 表示 /clock 没连上，真值口径会失真），位姿 (%.3f,%.3f) '
          'yaw=%+.3f z=%.4f' % (p[0], p[1], p[2], p[3], p[4]))

    rows = []
    # G 段是负样本：零速必须实测 |v|≈0、|ω|≈0。它不过就说明差分口径或环境本身在动，
    # 后面所有 R 段的数字都不可用（按"判据先用已通过的对照校准"那条来）。
    segs = [('G 零速对照', 0.0, 0.0),
            ('L 线速度对照', 0.25, 0.0),
            ('R 微角速度', 0.0, 0.05),
            ('R 小角速度', 0.0, 0.10),
            ('R 指令角速度', 0.0, 0.35),
            ('R 反向角速度', 0.0, -0.35)]
    for tag, v, w in segs:
        ok = n.brake(1.5, a.x, a.y, a.z, 0.0)
        print('=== %s（摆位 %s，刹轮后 yaw=0）===' % (tag, 'ok' if ok else '失败'))
        rows.append((tag, v, w) + n.send(v, w, a.secs, tag)[1:3] + (n.received,))
    n.brake(2.0, a.x, a.y, a.z, 0.0)
    print('--- 换算表（前冲/指令角速度）---')
    for tag, v, w, pv, pw, rcv in rows:
        ratio = (pv / w) if w else float('nan')
        print('  %-14s 指令 v=%+.2f ω=%+.2f → 峰值真值 |v|=%.2f |ω|=%.2f  '
              'v/|ω|=%s  收到 %d 条 cmd 最大 |v|=%.2f' % (
                  tag, v, w, pv, pw,
                  '%.2f' % ratio if w else '—', rcv[0], rcv[1]))
    p = n.pose
    print('收尾位姿 (%.3f,%.3f) yaw=%+.3f z=%.4f' % (p[1], p[2], p[3], p[4]))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
