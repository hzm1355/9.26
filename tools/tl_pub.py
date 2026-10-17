#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""向信号灯指令话题可靠下发一条 std_msgs/String 指令。

   为什么不用 `ros2 topic pub --once`：它 create_publisher 之后立刻发一条就退出，
   此刻管理器侧还没完成对该新发布者的 DDS 发现，消息被静默丢弃且无任何报错。
   本轮 preset=peak 就是这么丢的（管理器日志里没有 'preset -> peak'）。
   这里先自旋等到 get_subscription_count()>0，再连发 N 条。"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC = sys.argv[1]
if len(sys.argv) > 3:                     # 末位是次数，中间全部算指令内容
    REPS = int(sys.argv[-1])
    DATA = ' '.join(sys.argv[2:-1])
else:
    REPS = 3
    DATA = sys.argv[2]
WAIT = 8.0


def main():
    rclpy.init()
    n = Node('tl_pub_once')
    pub = n.create_publisher(String, TOPIC, 10)
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < WAIT:
        rclpy.spin_once(n, timeout_sec=0.1)
    subs = pub.get_subscription_count()
    for _ in range(REPS):
        pub.publish(String(data=DATA))
        rclpy.spin_once(n, timeout_sec=0.25)
    print(f'  已下发 {TOPIC} <- "{DATA}"（订阅者 {subs} 个，连发 {REPS} 次，等订阅者耗时 {time.time()-t0:.1f}s）')
    n.destroy_node()
    rclpy.shutdown()


main()
