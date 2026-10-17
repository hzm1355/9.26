#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一帧相机话题存成 PNG，用肉眼确认"停车场前面"在相机里到底是什么。

为什么需要：桌面窗口枚举里没有 Gazebo 窗口（gzclient pid 9635 活着，但画面在
VcXsrv 的无标题窗口里），拿不到视口截图；而闸机相机是停车场这块唯一能直接看
现场的传感器，它拍到什么就说明识别为什么不出结果。

用法：python3 tools/pk171_camgrab308.py <话题> <秒> <输出.png>
"""
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self, topic, out):
        super().__init__('pk171_camgrab308')
        self.br = CvBridge()
        self.out = out
        self.n = 0
        q = QoSProfile(depth=5,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, topic, self.cb, q)
        self.get_logger().info('订阅 %s' % topic)

    def cb(self, m):
        img = self.br.imgmsg_to_cv2(m, desired_encoding='bgr8')
        cv2.imwrite(self.out, img)
        self.n += 1
        self.get_logger().info('已存第 %d 帧 %s  形状=%s' % (self.n, self.out, str(img.shape)))


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    topic, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
    rclpy.init()
    node = Grab(topic, out)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    end = node.get_clock().now().nanoseconds / 1e9 + secs
    while node.get_clock().now().nanoseconds / 1e9 < end and rclpy.ok():
        ex.spin_once(timeout_sec=0.2)
    n = node.n
    ex.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()
    print('FRAMES=%d OUT=%s' % (n, out))
    return 0 if n > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
