#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_6 在无发布者、持续零速流下仍匀速转圈 —— 一次取干净的联合采样判成因。

四路证据同时采（分开采会互相错位，本项目的教训是"改前→改后"判据必须同源同时钟）：
  A. gz 真值位姿（~gz::Msg Response，唯一权威的位置来源，本环境没有 /joint_states）
  B. gz 轮角绝对值（累计量，只看增量）
  C. /car_6/odom 的 twist（插件自己的轮积分 —— 轮子真在转时它才非零）
  D. /model_states 里 car_6 的 twist（Gazebo 报的车体真实速度）

判据：
  ① 轮角 Δ 与"车心弧长/轮半径"一致 → 是轮子在驱动它（插件层面还在给轮速）；
  ② 轮角 Δ≈0 而车心在动 → 外力/接触挤出，与轮驱无关；
  ③ odom twist 非零 → 插件确实认为自己在跑；
  ④ 全程 /car_6/cmd_vel 发布者数=0，所以任何"指令"都只能是插件自己保持的残值。
"""
import math
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

MODEL = 'car_6'
N = 15
DT = 2.0


def gz(*args):
    out = subprocess.run(['gz', 'model', '-m', MODEL] + list(args),
                         capture_output=True, timeout=25)
    return out.stdout.decode('utf-8', 'replace')


def gz_pose():
    txt = gz('-p').strip().splitlines()[0].split()
    return [float(v) for v in txt[:6]]


def gz_wheels():
    a = {}
    for side in ('left', 'right'):
        txt = gz('-i')
        for line in txt.splitlines():
            if 'car_6::%s_wheel_joint' % side in line:
                a[side] = None
            if line.strip().startswith('angle:') and a.get(side, True) is None:
                a[side] = float(line.split()[1])
                break
    # -i 一次输出全部 joint，重新按段解析更稳妥
    txt = gz('-i')
    cur = None
    for line in txt.splitlines():
        s = line.strip()
        if s.startswith('name:'):
            cur = s.split(':', 1)[1].strip().strip('"')
        elif s.startswith('angle:') and cur and '_wheel_joint' in cur:
            a[cur.split('::')[1].split('_')[0]] = float(s.split()[1])
    return a


class Sink:
    def __init__(self, node):
        self.node = node
        self.odom = None
        self.ms = None
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Odometry, '/car_6/odom', self.on_odom, 10)
        node.create_subscription(ModelStates, '/model_states', self.on_ms, q)

    def on_odom(self, m):
        self.odom = (m.twist.twist.linear.x, m.twist.twist.angular.z,
                     m.pose.pose.position.x, m.pose.pose.position.y)

    def on_ms(self, m):
        if MODEL in m.name:
            i = m.name.index(MODEL)
            p = m.pose[i]
            t = m.twist[i]
            self.ms = (p.position.x, p.position.y, t.linear.x, t.angular.z,
                       p.orientation.z, p.orientation.w)


def main():
    rclpy.init()
    node = rclpy.create_node('why_spin135')
    sink = Sink(node)
    print('  等待首轮数据…')
    for _ in range(40):
        rclpy.spin_once(node, timeout_sec=0.25)
        if sink.odom and sink.ms:
            break
    rows = []
    for k in range(N):
        rclpy.spin_once(node, timeout_sec=0.0)
        pose = gz_pose()
        wh = gz_wheels()
        rows.append((time.time(), pose, wh, sink.odom, sink.ms))
        yaw = pose[5]
        o = sink.odom
        m = sink.ms
        print('  %2d  x=%7.3f y=%7.3f yaw=%+6.2f | 轮 L=%+9.2f R=%+9.2f | '
              'odom v=%+.3f w=%+.3f | ms v=%+.3f w=%+.3f' % (
                  k, pose[0], pose[1], yaw, wh.get('left', float('nan')),
                  wh.get('right', float('nan')),
                  o[0] if o else float('nan'), o[1] if o else float('nan'),
                  m[2] if m else float('nan'), m[3] if m else float('nan')))
        time.sleep(DT)
    print('\n=== 增量对账 ===')
    t0, p0, w0 = rows[0][0], rows[0][1], rows[0][2]
    t1, p1, w1 = rows[-1][0], rows[-1][1], rows[-1][2]
    dyaw = math.atan2(math.sin(p1[5] - p0[5]), math.cos(p1[5] - p0[5]))
    dl, dr = w1['left'] - w0['left'], w1['right'] - w0['right']
    R, SEP = 0.10, 0.70
    print('  时段 %.1f s：yaw 变化 %+.2f rad（%+.3f rad/s）' % (
        t1 - t0, dyaw, dyaw / (t1 - t0)))
    print('  轮角增量 左 %+.2f rad、右 %+.2f rad（均值 %+.2f）' % (dl, dr, (dl + dr) / 2))
    print('  由轮角换算：车心弧长 %.2f m、转角 %+.2f rad' % (
        R * (dl + dr) / 2.0, R * (dr - dl) / SEP))
    print('  判据①：轮子真在转 = %s' % ('是' if max(abs(dl), abs(dr)) > 0.5 else '否'))
    print('  判据②：轮角不动而车动 = %s' % (
        '是（外力/挤出）' if max(abs(dl), abs(dr)) <= 0.5 else '否'))
    print('  判据③：odom 自报非零 = %s' % (
        '是' if any(abs(r[3][0]) > 0.01 or abs(r[3][1]) > 0.01
                    for r in rows if r[3]) else '否'))
    print('  判据④：Gazebo 车体速度非零 = %s' % (
        '是' if any(abs(r[4][2]) > 0.01 or abs(r[4][3]) > 0.01
                    for r in rows if r[4]) else '否'))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
