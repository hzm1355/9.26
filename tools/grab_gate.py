#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓拍相机话题并存 PNG。环境里没有 cv_bridge，所以手工把 Image 消息还原成数组。
车辆只在过卡口的一瞬间进入画面，所以按时间间隔抽样而不是连拍。
用法：python3 grab_gate.py <image_topic> <输出目录> <张数> [抽样间隔秒，默认 6]"""
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from PIL import Image as PILImage
from sensor_msgs.msg import Image


class Grab(Node):
    def __init__(self, topic, outdir, count, interval):
        super().__init__('grab_gate')
        os.makedirs(outdir, exist_ok=True)
        self.outdir, self.n, self.count = outdir, 0, count
        self.interval, self.last = interval, 0.0
        self.create_subscription(Image, topic, self.cb, 10)

    def cb(self, msg):
        now = time.time()
        if self.n >= self.count or now - self.last < self.interval:
            return
        self.last = now
        # R8G8B8：每行按 msg.step 排布（可能有行末填充），不能直接 reshape(h,w,3)
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        PILImage.fromarray(img).save('%s/f%02d.png' % (self.outdir, self.n))
        self.n += 1
        self.get_logger().info('已存第 %d 帧 %dx%d' % (self.n, msg.width, msg.height))


def main():
    rclpy.init()
    node = Grab(sys.argv[1], sys.argv[2], int(sys.argv[3]),
                float(sys.argv[4]) if len(sys.argv) > 4 else 6.0)
    limit = 60 + node.count * node.interval + 30
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < limit and node.n < node.count:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.n == 0:
        print('!! 没有任何帧：相机话题未发布')


if __name__ == '__main__':
    main()
