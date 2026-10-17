#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端实机验证：换牌 -> 等车过卡口 -> 读 /plate/result -> 量车牌在画面里的真实像素尺寸。

离线自检已经 20/20，这里要回答的是另一件事：world 里那两台相机的几何
（抓拍斜距 4.50 m、俯角 0.55 rad、fov 0.6 @ 1920）到底把 440x140 mm 的牌拍成了多大，
以及龙门架立杆还会不会挡镜头。预测值是约 303x82 px，量出来对不上就说明 world 的
<focus>/<pose> 和注释里的推导脱了节。

用法：python3 verify_live_recognizer.py [等待秒数=240]
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import PlateCapture, PlateResult

from plate_recognizer_node import find_plate

PAYLOAD = 'car_1|rear|沪A12345'


def quad_edges(corners):
    """返回 (宽, 高)：四边形上下边均值 / 左右边均值。"""
    if corners is None:
        return 0.0, 0.0
    w = (np.linalg.norm(corners[1] - corners[0]) + np.linalg.norm(corners[2] - corners[3])) / 2
    h = (np.linalg.norm(corners[2] - corners[1]) + np.linalg.norm(corners[3] - corners[0])) / 2
    return float(w), float(h)


class LiveCheck(Node):
    def __init__(self, wait_s):
        super().__init__('verify_live_recognizer')
        self.wait_s = wait_s
        self.results = []
        self.images = {}
        self.t0 = time.time()

        self.create_subscription(PlateResult, '/plate/result', self.on_result, 50)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, 50)
        self.pub = self.create_publisher(String, '/plate/request', 10)

        while self.pub.get_subscription_count() == 0 and time.time() - self.t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            print('!! /plate/request 无订阅者，plate_gen_node 未运行')
            return
        self.pub.publish(String(data=PAYLOAD))
        print('已请求换牌: %s，等待车辆过卡口（最长 %d s）...' % (PAYLOAD, wait_s))

    def on_capture(self, msg):
        self.images[msg.capture_id] = msg.image_path

    def on_result(self, r):
        self.results.append(r)
        print('%-22s %-10s 真值=%-10s 置信%.2f %s  bbox %dx%d'
              % (r.capture_id, r.plate_number or '(空)', r.truth_number or '(无)',
                 r.confidence, '命中' if r.matched else '未命中', r.bbox_w, r.bbox_h))

    def run(self):
        while time.time() - self.t0 < self.wait_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            if any(r.plate_number for r in self.results):
                break
        return self.report()

    def report(self):
        hits = [r for r in self.results if r.plate_number]
        print('\n共收到 %d 条识别结果，其中 %d 条读出了号码' % (len(self.results), len(hits)))
        for r in hits[-3:]:
            path = self.images.get(r.capture_id)
            img = cv2.imread(path) if path else None
            if img is None:
                print('%s 抓拍图不可读: %s' % (r.capture_id, path))
                continue
            corners, area = find_plate(img)
            w, h = quad_edges(corners)
            print('%s 图 %s %dx%d | 车牌四边形 %.0fx%.0f px（预测 303x82）面积 %.0f'
                  % (r.capture_id, os.path.basename(path), img.shape[1], img.shape[0],
                     w, h, area))
            print('   识别=%r 真值=%r %s 耗时 %.0f ms'
                  % (r.plate_number, r.truth_number, '命中' if r.matched else '未命中',
                     r.process_ms))
            src = '/mnt/d/desktop1/9.26.1/live_%s.png' % r.capture_id
            vis = img.copy()
            if corners is not None:
                cv2.polylines(vis, [corners.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.imwrite(src, vis)
            print('   证据图 ->', src)
        return 0 if any(r.matched for r in hits) else 1


def main():
    rclpy.init()
    node = LiveCheck(float(sys.argv[1]) if len(sys.argv) > 1 else 240)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
