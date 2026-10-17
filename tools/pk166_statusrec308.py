#!/usr/bin/env python3
"""166：把 /parking/shuttle_status 的每一拍原值落盘，用来判"朝向误差翻转"的机理。

为什么订这个而不是自己再算一遍：这一行是**整形之后**的指令 + 实测位姿的同一拍快照
（目标车头、实测航向、发出去的 v/w、本航点已累计转向），翻转、过冲、刹不住三件事
都在这几个数之间，不需要再拿 /model_states 差分二次加工 —— 二次加工会把口径改掉。

用法：python3 pk166_statusrec308.py <秒> <模型名> <输出文件>
"""
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class Rec(Node):
    def __init__(self, want, fp):
        super().__init__('pk166_statusrec')
        self.want = want
        self.fp = fp
        self.n = 0
        self.create_subscription(String, '/parking/shuttle_status', self.on_s, 50)

    def on_s(self, m):
        if not m.data.startswith(self.want):
            return
        self.n += 1
        self.fp.write('%.3f %s\n' % (time.time(), m.data))
        self.fp.flush()


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    want = sys.argv[2] if len(sys.argv) > 2 else 'car_3'
    out = sys.argv[3] if len(sys.argv) > 3 else '/tmp/pk166_status308.log'
    rclpy.init()
    fp = open(out, 'a', encoding='utf-8')
    node = Rec(want, fp)
    # executor 必须在 init 之后建，且 spin_once 只收 timeout_sec（不带 node 参数）。
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t0 = time.time()
    while time.time() - t0 < secs:
        ex.spin_once(timeout_sec=0.2)
    print('收到 %d 拍 → %s' % (node.n, out))
    fp.close()
    ex.remove_node(node)
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
