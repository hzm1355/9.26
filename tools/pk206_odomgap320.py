#!/usr/bin/env python3
# 每辆车的 odom 位姿 vs model_states 位姿，给一个横向差值。
#
# 为什么单独量这个：signal_drive_node 的"出界→复位"用的是车的位置读数，而 Gazebo 的
# diff_drive 插件 odom 是**自己积分出来的**（cmd_vel 积分），/set_entity_state 摆位不会
# 把它一起搬走 —— 于是摆完位，模型在 A、插件以为自己在 B，两者差多远就决定驱动节点
# 接手后的第一脚会不会把车拽走或判成出界（memory「摆位弹射与复位风暴」/任务 #87
# "轮位目标 vs odom 位姿 vs 接触挤出"就是这条机制）。差值持续不变=两套读数各说各话，
# 差值在收敛=插件正被 cmd_vel 拉回来，车会顺走。
# 用法：python3 tools/pk206_odomgap320.py [采样秒数] [模型名...]
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
MODELS = sys.argv[2:] or ['car_1', 'car_2', 'car_3', 'car_4', 'car_5']


class Collector(Node):

    def __init__(self):
        super().__init__('pk206_odomgap')
        self.model = {}
        self.odom = {}
        self.vel = {}
        self.create_subscription(ModelStates, '/model_states', self.on_ms, 10)
        for m in MODELS:
            self.create_subscription(Odometry, '/%s/odom' % m,
                                    lambda msg, k=m: self.on_odom(k, msg), 10)
            self.create_subscription(Twist, '/%s/cmd_vel' % m,
                                     lambda msg, k=m: self.on_vel(k, msg), 10)

    def on_ms(self, msg):
        for name in MODELS:
            if name in msg.name:
                i = msg.name.index(name)
                p = msg.pose[i].position
                t = msg.twist[i].linear
                self.model[name] = (p.x, p.y, math.hypot(t.x, t.y))

    def on_odom(self, key, msg):
        p = msg.pose.pose.position
        self.odom[key] = (p.x, p.y)

    def on_vel(self, key, msg):
        self.vel[key] = (msg.linear.x, msg.angular.z)


def main():
    rclpy.init()
    node = Collector()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(DUR)
    ex.shutdown()

    print('  %-7s %-16s %-16s %-9s %-9s %s' % (
        '车', '模型位姿(x,y)', 'odom(x,y)', 'gap', '实测速', '末条 cmd_vel'))
    for m in MODELS:
        mm = node.model.get(m)
        oo = node.odom.get(m)
        if mm is None or oo is None:
            print('  %-7s 模型%s odom%s —— 有一路读不到，没法比' % (
                m, '有' if mm else '无', '有' if oo else '无'))
            continue
        gap = math.hypot(mm[0] - oo[0], mm[1] - oo[1])
        cv = node.vel.get(m)
        print('  %-7s (%7.2f,%7.2f)   (%7.2f,%7.2f)   %7.2f m  %5.2f m/s  %s' % (
            m, mm[0], mm[1], oo[0], oo[1], gap, mm[2],
            'v=%+.2f w=%+.2f' % cv if cv else '从没发过'))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
