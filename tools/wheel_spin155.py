#!/usr/bin/env python3
"""判"轮子到底转没转"：给油期间采 car_6 的轮关节角，看关节角变不变。

为什么要单独这一步：/car_6/odom 是 diff_drive 插件按**轮子**积分出来的，而插件算 odom
用的是它自己缓存的轮位，不是物理真值 —— 用它判"轮子在不在转"等于拿执行者的自述当证据
（memory「command-executor-wedge-triage」）。物理真值只有两处：/joint_states 里轮关节的
实际角度，和 /link_states 里轮 link 的实际姿态（轮子自转时 link 位置不变、姿态变）。

两种成因在这里彻底分开：
  被实体顶住   —— 轮关节角在变（或顶到堵转时 gz 仍给非零 effort），底盘位姿不变；
  插件实例卡死 —— 轮关节角一动不动，底盘也不动，而 cmd_vel 订阅者数还是 1。

用法: wheel_spin155.py [模型] [v] [秒]
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates

NAME = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0


def tail(n):
    """'car_6/left_wheel_joint' -> 'left_wheel_joint'；没有分隔符就原样返回。

    /link_states 与 /joint_states 里混着不带 '/' 的条目（world 固定件、数值 id 命名的
    关节），直接 split[1] 会 IndexError 把回调异常抛出去、掀掉采样进程。
    """
    return n.split('/', 1)[1] if '/' in n else n


class Probe:
    def __init__(self):
        rclpy.init()
        self.nd = Node('wheel_spin155')
        self.q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.joint = {}
        self.link = {}
        self.pose = None
        self.njoint = 0
        self.nd.create_subscription(JointState, '/joint_states', self.on_joint, self.q)
        self.nd.create_subscription(LinkStates, '/link_states', self.on_link, self.q)
        self.nd.create_subscription(Twist, '/' + NAME + '/odom', self.on_odom, self.q)
        # 发布端必须默认 RELIABLE：给成 BEST_EFFORT 会被 gz 插件判 QoS 不兼容，
        # DDS 层零投递且只留一句警告，看起来和"实例卡死"一模一样（drive_probe153 同款坑）
        self.pub = self.nd.create_publisher(Twist, '/' + NAME + '/cmd_vel', 10)

    def on_joint(self, m):
        self.njoint = len(m.name)
        for i, n in enumerate(m.name):
            if n.startswith(NAME):
                pos = m.position[i] if i < len(m.position) else 0.0
                vel = m.velocity[i] if i < len(m.velocity) else 0.0
                self.joint[n] = (pos, vel)

    def on_link(self, m):
        for i, n in enumerate(m.name):
            if not n.startswith(NAME):
                continue
            p = m.pose[i].position
            o = m.pose[i].orientation
            roll = math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x ** 2 + o.y ** 2))
            self.link[tail(n)] = (p.x, p.y, p.z, roll)

    def on_odom(self, m):
        self.pose = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.nd, timeout_sec=0.05)

    def send(self, v, secs):
        t0 = time.time()
        n = 0
        while time.time() - t0 < secs:
            m = Twist()
            m.linear.x = v
            self.pub.publish(m)
            n += 1
            rclpy.spin_once(self.nd, timeout_sec=0.02)
        return n


def main():
    p = Probe()
    p.spin(3.0)
    print('  /joint_states 收到过 %d 个关节名，其中属于 %s 的 %d 个；/link_states 里 %s 的 link %d 个'
          % (p.njoint, NAME, len(p.joint), NAME, len(p.link)))
    if not p.joint and not p.link:
        print('FAIL 两个真值话题都没帧，判不了（gzserver 的 state publisher 没起）'
              ' —— 注意这不能反过来说明物理停了，得单独看 /clock')
        rclpy.shutdown()
        return 1
    j0 = dict(p.joint)
    l0 = dict(p.link)
    n = p.send(V, SECS)
    p.spin(1.0)
    p.send(0.0, 2.0)
    print('  给油 v=%+.2f 发 %d 帧 / %.1f s，收尾已连发 2 s 零速' % (V, n, SECS))
    print('  --- 轮关节：位置/速度的变化 ---')
    moved = 0
    for k in sorted(p.joint):
        d = p.joint[k][0] - j0.get(k, (0.0, 0.0))[0]
        vj = p.joint[k][1]
        if abs(d) > 1e-4 or abs(vj) > 1e-3:
            moved += 1
        print('    %-34s Δ角 %+.4f rad  当前角速度 %+.4f rad/s' % (tail(k), d, vj))
    print('  --- 轮 link 姿态（自转时位置不变、roll 变）---')
    for k in sorted(p.link):
        d = p.link[k][3] - l0.get(k, (0, 0, 0, 0.0))[3]
        dd = math.hypot(p.link[k][0] - l0.get(k, (0, 0, 0, 0))[0],
                        p.link[k][1] - l0.get(k, (0, 0, 0, 0))[1])
        print('    %-22s Δ位置 %.4f m  Δroll %+.4f rad' % (k, dd, d))
    if p.pose is not None:
        print('  odom 自述（只作对照，别当轮子真值）：线速 %+.3f 角速 %+.3f'
              % p.pose)
    dj = sum(abs(p.joint[k][0] - j0.get(k, (0.0, 0.0))[0]) for k in p.joint)
    print('  判定：轮关节总变化 %.4f rad、有动的关节 %d 个 —— %s'
          % (dj, moved,
             '轮子在转，底盘不动 => 被实体顶住（A）' if (dj > 0.05 or moved)
             else '轮子纹丝不动 => diff_drive 实例没出力（B）'))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
