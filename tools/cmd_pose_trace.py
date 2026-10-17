#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把"节点下了什么指令"和"车实际怎么动"录到同一张表里，用来分清两种跑偏：

  · 控制器自激：cmd_v/cmd_w 本身就在振荡，且 |实测| ≤ |指令|（指令顶到上限 0.45 就停住）。
  · 物理被推：cmd 平稳甚至为零，实测线速度/角速度却越过指令上限
    （cruise 0.7 / max_angular 0.45）—— 只能是 ODE 在给车灌能量。

#77 的事故车不固定（car_3 在 371~405 s、car_2 在 1588~1651 s 各来了一次），
一次只录一辆车就要赌运气，所以默认五车同录，每车一张表。

行数按 /clock 的 10 Hz 收（520 s→5200 行），但单帧位移越过 JUMP_FORCE 的那一帧
无视时刻重复也要落表 —— 复位瞬移常常就落在同一个 100 ms 窗口里，按 sim_t 去重
会把它整帧抹掉，读表时看着像"车没动"。

用法（WSL）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 tools/cmd_pose_trace.py --seconds=540
"""
import argparse
import csv
import math
import os

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

COLS = ['t', 'x', 'y', 'yaw', 'cmd_v', 'cmd_w', 'mv', 'mw', 'jump']

# 位姿话题是 /model_states（signal_drive_node.py:323 用的就是它）。
# /gazebo/model_states 只在 gz-transport 里，ROS 侧 echo 得空表，别拿它当证据。
TOPIC = '/model_states'
ALL_CARS = ['car_1', 'car_2', 'car_3', 'car_4', 'car_5']

# 正常巡航 0.7 m/s 配 ~97 Hz 位姿流，单帧位移只有 ~7 mm，所以 0.3 m 这一档
# 只可能被瞬移越过；它又低于 trace_reset_scan 用的 0.5/0.6 门限，
# 保证扫描要看的每一帧都已经落进表里。
JUMP_FORCE = 0.3


def dump(path, rows):
    with open(path, 'w', newline='', encoding='utf-8') as fp:
        w = csv.writer(fp)
        w.writerow(COLS)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cars', default=','.join(ALL_CARS))
    ap.add_argument('--seconds', type=float, default=300.0)
    ap.add_argument('--outdir', default='/tmp')
    a = ap.parse_args()
    cars = [c.strip() for c in a.cars.split(',') if c.strip()]
    os.makedirs(a.outdir, exist_ok=True)

    rclpy.init()
    nd = rclpy.create_node('cmd_pose_trace')
    st = {'t': 0.0}
    per = {c: {'cmd_v': 0.0, 'cmd_w': 0.0, 'prev': None, 'rows': []} for c in cars}
    paths = {c: os.path.join(a.outdir, 'cmd_pose_%s.csv' % c) for c in cars}

    nd.create_subscription(
        Clock, '/clock',
        lambda m: st.__setitem__('t', m.clock.sec + m.clock.nanosec * 1e-9),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def mk_cmd(name):
        def on_cmd(m):
            d = per[name]
            d['cmd_v'] = m.linear.x
            d['cmd_w'] = m.angular.z
        return on_cmd

    for c in cars:
        nd.create_subscription(Twist, '/%s/cmd_vel' % c, mk_cmd(c), 10)

    def on_states(m):
        t = st['t']
        for c in cars:
            if c not in m.name:
                continue
            d = per[c]
            i = m.name.index(c)
            p = m.pose[i].position
            q = m.pose[i].orientation
            tw = m.twist[i]
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            prev = d['prev']
            d['prev'] = (t, p.x, p.y, yaw)
            if prev is None:
                continue
            # 位姿流 ~97 Hz、/clock 只有 10 Hz，所以按 sim_t 去重成 10 Hz 的行。
            # 但同一刻里后到的那一帧可能就是复位瞬移本身 —— 整帧丢掉的话，
            # 事后读表就成了"车没动"。所以只要单帧位移越过 JUMP_FORCE，
            # 哪怕和上一行同一个 t 也照录（代价只多出瞬移那几个格）。
            jump = math.hypot(p.x - prev[1], p.y - prev[2])
            if t - prev[0] <= 1e-6 and jump < JUMP_FORCE:
                continue
            # 实测速度直接取 twist（模型根的世界系速度），不做位姿差分：
            # 差分要吃 /clock 的量化，20 Hz 位姿配 100 Hz 时钟会把速度缩一个量级。
            # 位姿差只留作 jump 列，用来把「复位瞬移」从「被推着走」里摘出去。
            mv = math.hypot(tw.linear.x, tw.linear.y)
            mw = abs(tw.angular.z)
            d['rows'].append([round(t, 2), round(p.x, 3), round(p.y, 3), round(yaw, 3),
                              round(d['cmd_v'], 3), round(d['cmd_w'], 3),
                              round(mv, 3), round(mw, 3), round(jump, 3)])
        if len(next(iter(per.values()))['rows']) % 200 == 0:
            for c in cars:
                if per[c]['rows']:
                    dump(paths[c], per[c]['rows'])

    nd.create_subscription(ModelStates, TOPIC, on_states, 10)

    # /clock 没来之前 t 恒为 0，会把超时判据卡死；先等时钟起走。
    waited = 0.0
    while rclpy.ok() and st['t'] == 0.0 and waited < 30.0:
        waited += 0.1
        rclpy.spin_once(nd, timeout_sec=0.1)
    if st['t'] == 0.0:
        print('没收到 /clock，退出（不要拿空表当结论）')
        return 1

    t_end = st['t'] + a.seconds
    while rclpy.ok() and st['t'] < t_end:
        rclpy.spin_once(nd, timeout_sec=0.1)
    rc = 0
    for c in cars:
        rows = per[c]['rows']
        dump(paths[c], rows)
        if not rows:
            print('%s：零样本 —— %s 上没有这条车，别把空表当结论' % (c, TOPIC))
            rc = 1
            continue
        print('%s：样本 %d，峰值 v=%.2f w=%.2f，瞬移>0.5 m %d 格 -> %s'
              % (c, len(rows), max(abs(r[6]) for r in rows), max(abs(r[7]) for r in rows),
                 sum(1 for r in rows if abs(r[8]) > 0.5), paths[c]))
    return rc


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    finally:
        if rclpy.ok():
            rclpy.shutdown()
