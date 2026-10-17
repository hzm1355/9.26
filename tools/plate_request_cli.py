#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""向 /plate/request 发一条换牌指令，并等 /plate/set 回音确认链路通。

车牌号里含中文，命令行传参会被 wsl.exe 的引号层破坏，默认号码写死在 DEFAULT。
用法：python3 plate_request_cli.py ['car_1|rear|沪A12345']
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT = 'car_1|rear|沪A12345'


class PlateRequest(Node):
    def __init__(self, payload):
        super().__init__('plate_request_cli')
        self.pub = self.create_publisher(String, '/plate/request', 10)
        self.repl = []
        self.create_subscription(String, '/plate/set', lambda m: self.repl.append(m.data), 10)

        # 等 plate_gen_node 订阅上来再发，否则首条消息在发现完成前就发出去了会丢
        t0 = time.time()
        while rclpy.ok() and self.pub.get_subscription_count() == 0 and time.time() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            self.get_logger().error('/plate/request 没有订阅者：plate_gen_node 未运行？')
            return

        self.pub.publish(String(data=payload))
        self.get_logger().info('已发送: %s' % payload)

        t0 = time.time()
        while rclpy.ok() and not self.repl and time.time() - t0 < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.repl:
            self.get_logger().info('/plate/set 回读成功: %s' % self.repl[0])
        else:
            self.get_logger().error('20 s 内没有 /plate/set —— plate_gen_node 没处理这条请求')


def main():
    rclpy.init()
    node = PlateRequest(sys.argv[1] if len(sys.argv) > 1 else DEFAULT)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
