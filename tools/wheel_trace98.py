#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 轮级追踪：把 odom 拆成左右轮等效地速，和指令算出的目标轮速同桌对比。

#97（_ss97.txt）否掉了"插件把 angular.z 当线速度"这一条：
  · L 段 指令 v=+0.25 ω=0.00 → 峰值真值 |v|=1.48 |ω|=2.56，段内转了 −1.67 rad
    （零角速度指令却转出 1.67 rad，不是换算能解释的）
  · R 段 ω=+0.05 与 ω=+0.35 的转向符号都不自洽（+1.67、+2.25、−2.93），像是被打断
  · G 段零速对照 0.00 m → 摆位本身干净，测量链没有偏置
  · 每段收到的 cmd 最大 |v| 从不超过本段发送值 → 全场只有本探针在发指令

所以剩下的两种可能是"轮速没稳态跟上（瞬态尖峰后撞停）"还是"稳态就比指令大一个倍数"，
前者是接触/物理，后者是插件换算。这一发把两者分开：
  · 指令下发降到 20 Hz（#97 那个循环被自己的 cmd 回调喂成 ~3500 Hz，先把这个变量钉死）
  · 段长拉到 4 s，每 0.2 s 打一行：仿真钟、位姿(x,y,yaw,z,roll,pitch)、真值 v/ω、
    odom v/ω，以及**按 gz 自己的换算**反解出的左右轮等效地速
      v_l = odom_v − odom_ω·sep/2 ， v_r = odom_v + odom_ω·sep/2
    目标是  t_l = cmd_v − cmd_ω·sep/2 ， t_r = cmd_v + cmd_ω·sep/2
  · 段末给"后半段均值/前 1 s 峰值"，均值≈目标 ⇒ 稳态没问题、尖峰是暂态；
    均值≈目标的数倍 ⇒ 换算/参数错

sep 用 world 里的 0.70，轮半径 0.10，与 diff_drive 插件参数一致。
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
SEP = 0.70
RATE_HZ = 20.0


def euler_of(q):
    """返回 (roll, pitch, yaw)，与 SDF 的 rpy 同序。"""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Trace(Node):
    def __init__(self, model):
        super().__init__('wheel_trace')
        self.model = model
        self.pose = None        # (sim_t, x, y, roll, pitch, yaw, z)
        self.odom = (0.0, 0.0)
        self.clock = 0.0
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
        self.received = (self.received[0] + 1,
                         max(self.received[1], abs(float(m.linear.x))),
                         max(self.received[2], abs(float(m.angular.z))))

    def on_states(self, m):
        try:
            i = m.name.index(self.model)
        except ValueError:
            return
        p = m.pose[i]
        r, q, y = euler_of(p.orientation)
        self.pose = (self.clock, p.position.x, p.position.y, r, q, y, p.position.z)

    def wait_pose(self, secs=10.0):
        end = time.time() + secs
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pose

    def publish(self, v, w):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)

    def spin_for(self, secs, v, w, cb=None):
        """定频发指令；cb(现在, 上一帧位姿) 用来打表。"""
        end = time.time() + secs
        prev = self.pose
        while time.time() < end:
            self.publish(v, w)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / RATE_HZ)
            rclpy.spin_once(self, timeout_sec=0.0)
            if cb and self.pose is not None and prev is not None:
                if self.pose[0] - prev[0] > 1e-9:
                    cb(self.pose, prev)
                    prev = self.pose

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

    def brake_and_place(self, secs, x, y, z, yaw):
        self.spin_for(secs, 0.0, 0.0)
        ok = self.place(x, y, z, yaw)
        self.spin_for(1.5, 0.0, 0.0)
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--secs', type=float, default=4.0)
    a = ap.parse_args()
    rclpy.init()
    n = Trace(a.model)
    p = n.wait_pose()
    if p is None:
        print('!! 10 s 没收到 /model_states')
        raise SystemExit(2)
    print('开场：仿真钟 %.3f，位姿 (%.3f,%.3f) z=%.4f' % (p[0], p[1], p[2], p[6]))

    for tag, v, w in [('L 只给线速度 v=0.25', 0.25, 0.0),
                      ('R 只给角速度 w=+0.35', 0.0, 0.35)]:
        ok = n.brake_and_place(1.5, a.x, a.y, a.z, 0.0)
        n.received = (0, 0.0, 0.0)
        tl, tr = v - w * SEP / 2.0, v + w * SEP / 2.0
        print('=== %s（摆位 %s）目标轮速 左=%+.3f 右=%+.3f m/s ===' % (tag, ok, tl, tr))
        rows = []

        def cb(cur, prev):
            dt = cur[0] - prev[0]
            tv = math.hypot(cur[1] - prev[1], cur[2] - prev[2]) / dt
            to = wrap(cur[5] - prev[5]) / dt
            ov, ow = n.odom
            rows.append((cur[0], tv, to, ov, ow))
            print('    仿真t=%8.3f (%7.3f,%7.3f) z=%6.3f r=%+.3f p=%+.3f yaw=%+.3f  '
                  '真值 v=%+.2f ω=%+.2f  odom v=%+.2f ω=%+.2f  '
                  '轮速 左=%+.2f 右=%+.2f' % (
                      cur[0], cur[1], cur[2], cur[6], cur[3], cur[4], cur[5],
                      tv, to, ov, ow, ov - ow * SEP / 2.0, ov + ow * SEP / 2.0))

        n.spin_for(a.secs, v, w, cb)
        if rows:
            half = rows[len(rows) // 2:]
            av = sum(r[3] for r in half) / len(half)
            ao = sum(r[4] for r in half) / len(half)
            peak = max(r[1] for r in rows)
            print('  段末统计：峰值真值|v|=%.2f  后半段 odom 均值 v=%+.2f ω=%+.2f  '
                  '→ 后半段轮速 左=%+.2f 右=%+.2f（目标 %+.2f/%+.2f）' % (
                      peak, av, ao, av - ao * SEP / 2.0, av + ao * SEP / 2.0, tl, tr))
        print('    本段收到 cmd %d 条，最大 |v|=%.2f |ω|=%.2f' % n.received)
    n.brake_and_place(2.0, a.x, a.y, a.z, 0.0)
    p = n.pose
    print('收尾位姿 (%.3f,%.3f) z=%.4f yaw=%+.3f' % (p[1], p[2], p[6], p[5]))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
