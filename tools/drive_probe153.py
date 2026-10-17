#!/usr/bin/env python3
"""car_6 转不动的判别实验：分别给"纯 ω"和"纯 v"，看哪一路有反应。

三种成因在被动观测下长得一模一样（152 轮实测：146 帧 ang=+0.350 有效指令，yaw 变化
0.00000 rad）：
  A 顶到实体 / 被夹在两层地面之间 —— 平动可能还有，转动被死挡；
  B diff_drive 插件实例卡死（memory「gz-diffdrive-instance-wedge」）—— 两路都不理；
  C 力矩/上限不够 —— 加大 ω 就动。
所以这里按 A/B/C 各自能区分的顺序打三段指令，每段都用 /model_states 的真实位姿算
进展，不看 odom（这一轮 /car_6/odom 读不到）。

退出前无条件连发 2 s 零速：diff_drive 的 cmd_vel_timeout 实测不生效（#120），
留着最后一条非零指令车会继续跑，轻则顶杆攒轮位误差、重则开出台面翻车（#85）。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

NAME = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
MODEL = '/' + NAME


def yaw_of(o):
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class Probe:
    def __init__(self):
        rclpy.init()
        self.nd = Node('drive_probe153')
        self.q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.pose = None
        self.odom = None
        self.nd.create_subscription(ModelStates, '/model_states', self.on_states, self.q)
        self.nd.create_subscription(Odometry, MODEL + '/odom', self.on_odom, self.q)
        # 发布端必须是 RELIABLE：把订阅用的 BEST_EFFORT 顺手递给 publisher，gz 的
        # diff_drive 插件（按默认 reliable 订阅）会直接判不兼容，DDS 层一条都不投，
        # 实测只留一句 "New subscription discovered ... requesting incompatible QoS"
        # 警告，而车一动不动 —— 和"驱动实例卡死"这个待判的成因长得一模一样。
        # shuttle 那边就是 create_publisher(Twist, ..., 10) 默认可靠，车才会走。
        self.pub = self.nd.create_publisher(
            Twist, MODEL + '/cmd_vel', QoSProfile(depth=10))

    def on_states(self, m):
        for i, n in enumerate(m.name):
            if n == NAME:
                p = m.pose[i].position
                self.pose = (p.x, p.y, p.z, yaw_of(m.pose[i].orientation))

    def on_odom(self, m):
        p = m.pose.pose.position
        o = m.pose.pose.orientation
        self.odom = (p.x, p.y, p.z, yaw_of(o))

    def settle(self, secs=1.0):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.nd, timeout_sec=0.05)

    def run(self, lin, ang, secs, label):
        a0 = self.pose
        if a0 is None:
            print('  %s: 没有位姿，跳过' % label)
            return
        o0 = self.odom
        t0 = time.time()
        n = 0
        while time.time() - t0 < secs:
            m = Twist()
            m.linear.x = lin
            m.angular.z = ang
            self.pub.publish(m)
            n += 1
            rclpy.spin_once(self.nd, timeout_sec=0.02)
        a1 = self.pose
        dth = math.atan2(math.sin(a1[3] - a0[3]), math.cos(a1[3] - a0[3]))
        dd = math.hypot(a1[0] - a0[0], a1[1] - a0[1])
        # odom 是插件按轮子积分的，model 位姿是底盘被物理算出来的。两者一起看才能把
        # "执行端没吃到指令"和"轮子在转但底盘被物理卡住"分开。
        o1 = self.odom
        if o0 and o1:
            oth = math.atan2(math.sin(o1[3] - o0[3]), math.cos(o1[3] - o0[3]))
            odd = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
            extra = '  odom 转向 %+.4f 平动 %.4f' % (oth, odd)
        else:
            extra = '  odom 无帧'
        print('  %-22s 发 %3d 帧  实测转向 %+.4f rad 实测平动 %.4f m%s'
              % (label, n, dth, dd, extra))
        print('  %-22s   底盘 (%.3f,%.3f)→(%.3f,%.3f)' % ('', a0[0], a0[1],
                                                          a1[0], a1[1]))

    def zero(self, secs=2.0):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.pub.publish(Twist())
            rclpy.spin_once(self.nd, timeout_sec=0.02)


def main():
    p = Probe()
    p.settle(3.0)
    if p.pose is None:
        print('FAIL 3 s 内没收到 %s 位姿' % NAME)
        return 1
    print('  起始 (%.3f, %.3f) z=%.3f yaw=%+.3f' % p.pose)
    # 发布通道自检：车不动有两种可能，"指令没送到"和"送到了但被顶住"。不先看订阅数
    # 就直接下结论，会把 QoS 不兼容这种自己的脚本 bug 读成仿真侧的故障（本项目已经
    # 在 /gazebo/model_states 上错过一次同型的）。
    try:
        nsub = p.pub.get_subscription_count()
    except Exception as e:
        nsub = -1
        print('  (拿不到订阅数：%s)' % e)
    print('  /%s/cmd_vel 的订阅者数 = %d（0 就是指令根本没送到，别往下判）'
          % (NAME, nsub))
    if nsub == 0:
        p.zero(1.0)
        rclpy.shutdown()
        return 1
    try:
        p.run(0.0, 0.35, 5.0, 'A 纯ω=+0.35(复现)')
        p.run(0.0, 1.20, 5.0, 'B 纯ω=+1.20(加力矩)')
        p.run(0.12, 0.0, 5.0, 'C 纯v=+0.12(平动)')
        p.run(-0.12, 0.0, 5.0, 'D 纯v=-0.12(倒退)')
    finally:
        p.zero(2.0)
        print('  已连发 2 s 零速钉住轮位目标')
        p.settle(1.5)
        print('  收尾 (%.3f, %.3f) z=%.3f yaw=%+.3f' % p.pose)
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
