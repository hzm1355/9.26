#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 car_6 往南挪到指定 y 停下 —— 用 cmd_vel 开过去，不用 /delete_entity + spawn。

为什么用"开"而不是"摆"：
 * 摆位（gz model -x -y …）会把 diff_drive 攒下的轮位误差在落位那一瞬释放，实测能甩到
   5~8.5 m/s 把车顶出沙盘（memory「摆位弹射与复位风暴」）。这里只是要往南 1 m 让入口闸
   的 d 越过 far_radius=2.5 完成再武装，犯不上冒这个险。
 * respawn 更不行：#143 实测 /spawn_entity 既不认传入 XML 的 model 级 <pose> 也不认
   initial_pose，只会按 world 里那份出生位姿生成；而且 delete+spawn 会让 gzclient 把这个
   模型从场景里丢掉（要重启客户端才回来）。
 * 倒车还有一个附带好处：顺手证明驱动通道是活的。
用法：nudge_south151.py [模型] [目标 y] [速度 m/s]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

MODEL = sys.argv[1] if len(sys.argv) > 1 else 'car_6'
TARGET = float(sys.argv[2]) if len(sys.argv) > 2 else -14.05
SPEED = float(sys.argv[3]) if len(sys.argv) > 3 else 0.20


class Nudge(Node):
    def __init__(self):
        super().__init__('nudge_south151')
        self.pose = None
        self.pub = self.create_publisher(Twist, '/%s/cmd_vel' % MODEL, 10)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        # 话题名是 /model_states，不是 /gazebo/model_states。world 行 3046 那句
        # <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/> 没有
        # 任何 remap，节点自报的发布话题就是 /model_states；写成带 /gazebo/ 前缀的
        # 那一版订阅到的是一个**从来不存在**的话题，15 s 超时、零帧，看起来像
        # "gzserver 挂了"，其实是我照抄了别处的话题名。同一条错误还让一个
        # `ros2 topic echo /gazebo/model_states` 在机器上挂了 5 天不出数。
        self.create_subscription(ModelStates, '/model_states', self.cb, q)

    def cb(self, m):
        if MODEL in m.name:
            p = m.pose[m.name.index(MODEL)].position
            self.pose = (p.x, p.y, p.z)


def zero(n, secs):
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        n.pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.02)
        time.sleep(0.03)


rclpy.init()
n = Nudge()
t0 = time.monotonic()
while rclpy.ok() and n.pose is None and time.monotonic() - t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
if n.pose is None:
    print('!! 15 s 内没拿到 %s 的位姿（gzserver 没起或模型名不对），不动' % MODEL)
    rclpy.shutdown()
    sys.exit(2)
x0, y0, z0 = n.pose
print('起点 (%.2f,%.2f,%.2f)｜目标 y=%.2f｜倒车 %.2f m/s' % (x0, y0, z0, TARGET, SPEED))
if y0 <= TARGET:
    print('已经在目标以南，不用挪')
else:
    last = y0
    while rclpy.ok() and n.pose[1] > TARGET and time.monotonic() - t0 < 60:
        tw = Twist()
        tw.linear.x = -SPEED
        n.pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.05)
        x, y, z = n.pose
        if abs(y - last) > 0.45:
            print('!! 单拍跳了 %.2f m（%.2f→%.2f），立刻停，别把车甩出台面' % (
                abs(y - last), last, y))
            break
        last = y
    else:
        print('  到点或超时：y=%.2f' % n.pose[1])
zero(n, 2.0)
x1, y1, z1 = n.pose
print('终点 (%.2f,%.2f,%.2f)｜z 未掉=%.2f（<6.5 就是掉出台面）' % (x1, y1, z1, z1))
d = ((x1 - 12.00) ** 2 + (y1 + 11.21) ** 2) ** 0.5
print('到 gate_in focus (12.00,-11.21) 的水平距离 d=%.2f m｜far_radius=2.5 → %s' % (
    d, '已越过，入口闸会重新武装' if d > 2.5 else '还没越过，闸机仍在解除武装之外'))
rclpy.shutdown()
