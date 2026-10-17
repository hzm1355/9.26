#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓一路 Gazebo 相机的一帧存成 PNG。用法：snap_gate140.py <话题> <输出.png> [等待秒]

为什么要单独一份：闸机的抓拍只在"进触发圈上升沿"那一拍出，想单独看
"车停在核验位时相机里是什么样"就必须能随时取一帧，否则只能等下一轮、
或者改 world 里的 near_radius（那要整栈重启）。
QoS 必须 best_effort：gz 的相机出帧是 sensor_data，用默认 reliable 订会一帧都收不到。
"""
import sys
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

TOPIC = sys.argv[1]
OUT = sys.argv[2]
WAIT = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0


class Snap(Node):
    def __init__(self):
        super().__init__('snap_gate140')
        self.got = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, TOPIC, self.cb, q)

    def cb(self, m):
        if self.got is not None:
            return
        h, w = m.height, m.width
        buf = np.frombuffer(m.data, dtype=np.uint8)
        step = m.step if m.step else w * 3
        arr = buf.reshape(h, step // 3, 3)[:, :w, :]
        self.got = np.ascontiguousarray(arr)
        self.get_logger().info('拿到一帧 %dx%d stride=%d' % (w, h, step))


def main():
    rclpy.init()
    n = Snap()
    t0 = time.time()
    while rclpy.ok() and n.got is None and time.time() - t0 < WAIT:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.got is None:
        print('%s 在 %.0f s 内没有出帧' % (TOPIC, WAIT))
        rclpy.shutdown()
        return 1
    cv2.imwrite(OUT, n.got)
    print('已存 %s：%dx%d 全图均值 %.1f' % (OUT, n.got.shape[1], n.got.shape[0],
                                          float(n.got.mean())))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
