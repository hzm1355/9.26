#!/usr/bin/env python3
"""第 151 轮重启前的现场探针：只读，不发任何非零速度，不起节点。

要回答的三个问题：
1) car_6 现在到底在哪儿，到 gate_in focus 的 d 是否已经越过 far=2.5；
2) 这 8 s 里有没有别的进程还在往 cmd_vel 上灌非零值（上一轮的 nudge 没死透
   就会把本轮的判据污染成"车自己会动"）；
3) gzserver 的位姿流是不是活的（没这条，后面所有轮询都是空转）。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

FOCUS = (12.00, -11.21)   # gate_in 轮轴，world 行 2534-2536
FAR = 2.5
NEAR = 1.4


def main():
    rclpy.init()
    nd = Node('probe151b')
    q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                   history=HistoryPolicy.KEEP_LAST)
    pose = {}
    live = {'n': 0}

    def on_states(m):
        live['n'] += 1
        for i, n in enumerate(m.name):
            if n == 'car_6':
                p = m.pose[i].position
                yaw = math.atan2(2.0 * (m.pose[i].orientation.w * m.pose[i].orientation.z),
                                 1.0 - 2.0 * (m.pose[i].orientation.z ** 2 +
                                              m.pose[i].orientation.w ** 2))
                pose['v'] = (p.x, p.y, p.z, yaw)

    tw = {}

    def on_tw(topic):
        def cb(m):
            live['cmd'] = live.get('cmd', 0) + 1
            if abs(m.linear.x) + abs(m.angular.z) > 1e-6:
                tw[topic] = (round(m.linear.x, 3), round(m.angular.z, 3))
        return cb

    # rclpy 的签名是 create_subscription(TopicType, topic, callback, qos)，
    # 参数顺序写反会直接砸进 resolve_topic_name 拿到类型对象，报 rclpy_resolve_name
    # 的 pybind 类型错 —— 报错信息看着像 topic 名有问题，其实是我把顺序记颠了。
    nd.create_subscription(ModelStates, '/model_states', on_states, q)
    nd.create_subscription(Twist, '/car_6/cmd_vel', on_tw('/car_6/cmd_vel'), q)

    t0 = time.time()
    while time.time() - t0 < 8.0:
        rclpy.spin_once(nd, timeout_sec=0.2)

    print('  /gazebo/model_states 收到 %d 帧（8 s，@1 kHz 理论应上千）' % live['n'])
    print('  cmd_vel 收到 %d 帧' % live.get('cmd', 0))
    if tw:
        for k, v in tw.items():
            print('  !! 非零 cmd_vel 来自 %s: lin=%.3f ang=%.3f' % (k, v[0], v[1]))
    else:
        print('  这 8 s 内没有非零 cmd_vel（车处于静止，驱动通道干净）')

    p = pose.get('v')
    if p is None:
        print('FAIL car_6 位姿没拿到 —— gzserver 没起或模型名不对')
        rclpy.shutdown()
        return 1
    d = math.hypot(p[0] - FOCUS[0], p[1] - FOCUS[1])
    print('  car_6 底盘中心: (%.2f, %.2f, z=%.2f) 朝向 %.2f rad' % p)
    # 闸机判的是轮轴到 focus 的距离，不是底盘中心；axle_ahead=0.40 车头方向前方
    px = p[0] + 0.40 * math.cos(p[3])
    py = p[1] + 0.40 * math.sin(p[3])
    dp = math.hypot(px - FOCUS[0], py - FOCUS[1])
    print('  d(底盘中心)=%.2f m  d(轮轴)=%.2f m  far=%.1f near=%.1f' % (d, dp, FAR, NEAR))
    print('  硬判据 y<=-13.75：%s' % ('OK' if p[1] <= -13.75 else '未达标，还需要南退'))
    print('  入口闸可再武装（轮轴 d>far）：%s' % ('OK' if dp > FAR else 'NO'))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
