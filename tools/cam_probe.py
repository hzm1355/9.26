#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给每路 Gazebo 相机抓一帧存盘，用来判断录视频该取哪个画面。
用法：python3 cam_probe.py [每话题等待秒=12] [输出目录]
"""
import json
import os
import sys
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

WAIT = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/mnt/d/desktop1/9.26.1/_camprobe'
os.makedirs(OUT, exist_ok=True)

TOPICS = [
    '/car_1/camera/image_raw',
    '/intersection_cam/intersection_cam/image_raw',
    '/monitor_cam/monitor_cam/image_raw',
    '/speed_cam/speed_cam/image_raw',
    '/gate_j3_north/gate_cam/image_raw',
    '/gate_j2_south/gate_cam/image_raw',
]


class Probe(Node):
    def __init__(self):
        super().__init__('cam_probe')
        self.got = {}
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        for t in TOPICS:
            self.create_subscription(Image, t, lambda m, t=t: self.cb(t, m), q)

    def cb(self, topic, msg):
        if topic in self.got:
            return
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            self.got[topic] = ('encoding', msg.encoding)
            return
        a = np.frombuffer(msg.data, dtype=np.uint8)
        h, w = msg.height, msg.width
        if msg.encoding == 'mono8':
            img = a.reshape(h, w)
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img = a.reshape(h, w, 3)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        name = topic.strip('/').replace('/', '_')
        path = os.path.join(OUT, name + '.jpg')
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.got[topic] = ('ok', w, h, float(gray.mean()),
                           float(gray.std()), float(gray.max()), path)


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < WAIT:
        rclpy.spin_once(n, timeout_sec=0.1)
    print(json.dumps(n.got, ensure_ascii=False, indent=1, default=str))
    for t in TOPICS:
        if t not in n.got:
            print('MISSING  %s   （该相机无帧输出）' % t)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
