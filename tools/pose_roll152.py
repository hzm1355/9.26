#!/usr/bin/env python3
"""car_6 卡住判死：把"指令在不在"和"车到底动没动"放在同一个窗口里对。

为什么必须同窗口：分开采样会造出假证据。上一轮我先读 gz 位姿（两次隔 5 s，yaw 只差
0.0008 rad）再单独 `ros2 topic echo --once /car_6/cmd_vel`，后者撞上 DDS 发现竞态
回了空包，读起来像"没人在发指令"，而 shuttle 日志同一时刻明明在写 w=+0.35。所以这
里用一个订阅窗口同时收 cmd_vel 帧和 model_states 帧，输出：
  - cmd_vel 收到多少帧、其中非零 ω 多少帧、最后一次是多少；
  - 同一窗口内 yaw 的最大变化、位置的最大位移；
  - 完整姿态（z / roll / pitch）—— memory「car_6 掉出沙盘翻车 roll=-90°」那一类
    形态在这里必须排除，因为翻车后轮子悬空，表现就是"发指令但车不动"，和顶住实体、
    和 diff_drive 插件卡死三者外观完全一样。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

NAME = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
WINDOW_S = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


def euler(q):
    # 与 gz 的惯性系一致：z-y-x 内旋（yaw, pitch, roll）
    w, x, y, z = q.w, q.x, q.y, q.z
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    rclpy.init()
    nd = Node('pose_roll152')
    q = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST)

    st = {'frames': 0, 'first': None, 'last': None,
          'yaw_min': None, 'yaw_max': None,
          'x_min': None, 'x_max': None, 'y_min': None, 'y_max': None,
          'roll': 0.0, 'pitch': 0.0, 'z': 0.0, 'yaw': 0.0}

    def on_states(m):
        for i, n in enumerate(m.name):
            if n != NAME:
                continue
            p = m.pose[i].position
            r, pt, y = euler(m.pose[i].orientation)
            st['frames'] += 1
            st['z'], st['roll'], st['pitch'], st['yaw'] = p.z, r, pt, y
            for k, v in (('x', p.x), ('y', p.y), ('yaw', y)):
                if st[k + '_min'] is None or v < st[k + '_min']:
                    st[k + '_min'] = v
                if st[k + '_max'] is None or v > st[k + '_max']:
                    st[k + '_max'] = v

    cmd = {'n': 0, 'nz': 0, 'last_lin': 0.0, 'last_ang': 0.0}

    def on_tw(m):
        cmd['n'] += 1
        if abs(m.angular.z) > 1e-6 or abs(m.linear.x) > 1e-6:
            cmd['nz'] += 1
        cmd['last_lin'], cmd['last_ang'] = m.linear.x, m.angular.z

    nd.create_subscription(ModelStates, '/model_states', on_states, q)
    nd.create_subscription(Twist, '/' + NAME + '/cmd_vel', on_tw, q)

    t0 = time.time()
    while time.time() - t0 < WINDOW_S:
        rclpy.spin_once(nd, timeout_sec=0.1)
    rclpy.shutdown()

    if st['frames'] == 0:
        print('FAIL %s s 内没收到 %s 位姿（gzserver 位姿流断了）' % (WINDOW_S, NAME))
        return 1
    print('  位姿帧 %d / %.0f s  cmd_vel 帧 %d（非零 %d）'
          % (st['frames'], WINDOW_S, cmd['n'], cmd['nz']))
    print('  最后一条 cmd_vel: lin=%.3f ang=%+.3f' % (cmd['last_lin'], cmd['last_ang']))
    print('  窗口内 yaw 变化 %.5f rad   位移 dx=%.4f dy=%.4f m'
          % (st['yaw_max'] - st['yaw_min'],
             st['x_max'] - st['x_min'], st['y_max'] - st['y_min']))
    print('  当前 中心=(%.3f, %.3f) z=%.3f  roll=%+.1f° pitch=%+.1f° yaw=%+.3f rad'
          % ((st['x_min'] + st['x_max']) / 2.0, (st['y_min'] + st['y_max']) / 2.0,
             st['z'], math.degrees(st['roll']), math.degrees(st['pitch']), st['yaw']))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
