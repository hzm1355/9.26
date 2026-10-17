#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 "发零速也自走"的定性探针（#85）。

上一发 pivot_trace 的现场：把车精确摆到 (12.00,-12.80,6.70) yaw=0 后
**持续 publish 全零 cmd_vel**，车却以 ~0.5 m/s、~1.2 rad/s 走半径 ~0.4 m 的弧；
而 spin_probe 里完全不发指令的同一辆车，24 s 纹丝不动。两者的共同点是
cmd_vel_timeout=0.3 s —— 不发也要超时刹轮，所以"刹轮"不是差别，差别只在
**有没有消息进插件**。

这一发要劈开的三种解释：
  A) 插件真把轮速设成了非零（那它自己的 /car_6/odom 会跟着报同样的 v/w，
     因为 odom 就是从轮子关节速度算出来的）→ 查 cmd 解析/超时分支；
  B) 轮子其实没转（odom v≈0），但整车被物理求解器推着走 → 接触/坡面/穿模；
  C) 是 /model_states 的读数假象（例如读到的是 odom 系或被 TF 污染）→
     拿 `gz model -p` 的真值复核。

做法：摆位后按 A/B/A/B 交替四段（不发 / 连发零速 / 不发 / 连发零速），
每段 5 s，每帧同时打印 ①/model_states 真位姿与其差分成 v、ω
②插件自己 /car_6/odom 的 v、ω ③插件 odom 积分出的位置。
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Probe(Node):
    def __init__(self, model):
        super().__init__('selfdrive_probe')
        self.model = model
        self.true_last = None      # (t_wall, x, y, yaw)
        self.true_now = None       # (x, y, yaw)
        self.true_inst = None      # (v, w) 由相邻两帧真值差分
        self.od = None             # (v_lin, v_ang, x, y, yaw)
        self.publish = False
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % model, 10)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)
        self.create_subscription(Odometry, '/%s/odom' % model, self.on_odom, 10)
        self.create_timer(0.05, self.tick)
        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            pass

    def tick(self):
        if self.publish:
            self.pub.publish(Twist())      # 全零

    def on_states(self, m):
        try:
            i = m.name.index(self.model)
        except ValueError:
            return
        p = m.pose[i]
        now = (p.position.x, p.position.y, yaw_of(p.orientation))
        t = time.monotonic()
        if self.true_last:
            dt = t - self.true_last[0]
            if dt > 1e-3:
                self.true_inst = (math.hypot(now[0] - self.true_last[1],
                                            now[1] - self.true_last[2]) / dt,
                                  wrap(now[2] - self.true_last[3]) / dt)
        self.true_last = (t, now[0], now[1], now[2])
        self.true_now = now

    def on_odom(self, m):
        t = m.twist.twist
        p = m.pose.pose
        self.od = (t.linear.x, t.angular.z, p.position.x, p.position.y,
                   yaw_of(p.orientation))

    def place(self, x, y, z, yaw):
        r = SetEntityState.Request()
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

    def phase(self, name, secs, publish, t0):
        self.publish = publish
        # 换相位先让上一段的瞬态过掉，再开始按帧打印
        end_warm = time.time() + 0.6
        while time.time() < end_warm:
            rclpy.spin_once(self, timeout_sec=0.02)
        end = time.time() + secs
        n_move = 0
        while time.time() < end:
            before = self.true_now
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.true_now is None or before == self.true_now:
                continue
            x, y, h = self.true_now
            v, w = self.true_inst or (0.0, 0.0)
            od = self.od or (float('nan'),) * 5
            if v > 0.05 or abs(w) > 0.05:
                n_move += 1
            print('  t=%5.2f 真(%7.3f,%7.3f) yaw=%+.3f | 真值 v=%5.2f w=%+5.2f '
                  'r=%s | 插件 odom v=%5.2f w=%+5.2f 位置(%6.2f,%6.2f)' % (
                      time.time() - t0, x, y, h, v, w,
                      '%5.2f' % (v / abs(w)) if abs(w) > 0.05 else ' --  ',
                      od[0], od[1], od[2], od[3]))
        self.publish = False
        print('  —— %s 段（publish=%s）：显著运动帧 %d' % (name, publish, n_move))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='car_6')
    ap.add_argument('--x', type=float, default=12.00)
    ap.add_argument('--y', type=float, default=-12.80)
    ap.add_argument('--z', type=float, default=6.70)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--secs', type=float, default=5.0)
    a = ap.parse_args()
    rclpy.init()
    n = Probe(a.model)
    # 摆位前刹轮 1.5 s（历史经验：不刹就在落点一次性释放攒下的轮位误差）
    n.publish = True
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    n.publish = False
    print('摆位 %s → (%.2f,%.2f,%.2f) yaw=%+.4f' % (
        'ok' if n.place(a.x, a.y, a.z, a.yaw) else '失败', a.x, a.y, a.z, a.yaw))
    # 落地后压 1.0 s 零速，再开始 A/B/A/B
    n.publish = True
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)
    n.publish = False
    p = n.true_now
    print('刚摆完读到 (%.3f,%.3f) yaw=%+.3f' % p)
    t0 = time.time()
    n.phase('1-不发', a.secs, False, t0)
    n.phase('2-连发零速', a.secs, True, t0)
    n.phase('3-不发', a.secs, False, t0)
    n.phase('4-连发零速', a.secs, True, t0)
    p = n.true_now
    print('收尾读到 (%.3f,%.3f) yaw=%+.3f，离落点 %.2f m' % (
        p[0], p[1], p[2], math.hypot(p[0] - a.x, p[1] - a.y)))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
