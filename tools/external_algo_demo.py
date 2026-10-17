#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""条目 2 在线验证用的"外部算法"桩节点：证明对接边界只有两条消息。

外部算法要做的全部事情 = 订阅 v2x_msgs/PlateCapture（必要时自己读 image_path 或
订阅 /gate_*/gate_cam/image_raw）+ 发布 v2x_msgs/PlateResult。本桩节点**不解字符**
（真实场景这里换成 YOLO+CRNN 之类），它做三件事来证明接口够用：
  1. 用 cv2.imread 真的把抓拍图读出来，量尺寸和全图均值 -> 说明图像拿得到、可用；
  2. 从 PlateCapture 透传抓拍时刻与车辆位姿 -> 三要素里的两要素本来就在消息里；
  3. 发布 PlateResult 时把 algorithm 写成 `external_stub_v1` -> 下游（记录/导出/告警/
     看板）不认算法是谁，只认这条消息，所以换算法不用改任何下游代码。

用法：python3 external_algo_demo.py [运行秒数=150]
配合：bash tools/external_algo_check.sh（它负责先停内置识别器、跑这个桩、再拉回来）
"""
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image

from v2x_msgs.msg import PlateCapture, PlateResult

ALGO = 'external_stub_v1'
IMG_TOPICS = ['/gate_j3_north/gate_cam/image_raw', '/gate_j2_south/gate_cam/image_raw']


class External(Node):
    def __init__(self):
        super().__init__('external_algo_demo')
        self.pub = self.create_publisher(PlateResult, '/plate/result', 50)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, 50)
        self.frames = {}
        self.n_frames = {}
        qos = QoSProfile(depth=1)
        for t in IMG_TOPICS:
            self.create_subscription(Image, t, lambda m, k=t: self.on_img(k, m), qos)
        self.n = 0

    def on_img(self, topic, m):
        self.n_frames[topic] = self.n_frames.get(topic, 0) + 1
        self.frames[topic] = (m.height, m.width, m.encoding,
                              m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def on_capture(self, c):
        img = cv2.imread(c.image_path)
        if img is None:
            print('!! 读不到抓拍图 %s' % c.image_path)
            mean = -1.0
        else:
            mean = float(img.mean())
        r = PlateResult()
        r.capture_id = c.capture_id
        r.gate_id = c.gate_id
        r.capture_stamp = c.header.stamp          # 抓拍时刻：透传
        r.vehicle_pose = c.pose                   # 车辆位置：透传
        r.speed = c.speed
        # 桩算法不解字符，这里回一个明显的假号，专门用来确认"下游认的是消息不是算法"
        r.plate_number = '外部算法A1'
        r.truth_number = ''
        r.matched = False
        r.confidence = 0.0
        r.bbox_x, r.bbox_y = 0, 0
        r.bbox_w = max(0, min(65535, img.shape[1] if img is not None else 0))
        r.bbox_h = max(0, min(65535, img.shape[0] if img is not None else 0))
        r.algorithm = ALGO
        r.process_ms = 0.0
        r.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(r)
        self.n += 1
        p = c.pose.position
        wh = '%dx%d' % (img.shape[1], img.shape[0]) if img is not None else '读不到'
        print('  [外部算法] %s 卡口=%s 图=%s 均值=%.1f 号=%s 时刻=%.3f s 位置=(%.2f, %.2f, %.2f) 速度=%.2f' % (
            c.capture_id, c.gate_id, wh, mean, r.plate_number,
            c.header.stamp.sec + c.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, c.speed))

    def report_images(self):
        print('\n=== 图像话题在线采样（外部算法可订阅的视频流）===')
        for t in IMG_TOPICS:
            f = self.frames.get(t)
            n = self.n_frames.get(t, 0)
            print('  %-40s %s' % (t, '窗口内收到 %d 帧 %dx%d %s，最新帧戳 %.3f' % (
                n, f[1], f[0], f[2], f[3]) if f else '窗口内无新帧'))


def main():
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 150
    rclpy.init()
    node = External()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < wait_s:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.report_images()
    print('桩算法共输出 %d 条 PlateResult' % node.n)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.n > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
