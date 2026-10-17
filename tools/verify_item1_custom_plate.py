#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""条目 1 在线验证：卡口电子警察 + 可自定义虚拟车牌，端到端跑一次。

发一个**从没在这条时线上出现过**的号码到 /plate/request，等它被卡口拍到、被识别器读回，
把抓拍号、卡口、图路径、识别值/真值、命中、置信、耗时、时刻、位姿一次打全。
跑完把车牌还原，避免污染后面的测试。

用法：python3 verify_item1_custom_plate.py [采集秒数=180]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import PlateCapture, PlateResult

MODEL = 'car_1'
CUSTOM = '渝D7K418'
ORIG = '沪A12345'


class Verify(Node):
    def __init__(self, wait_s):
        super().__init__('verify_item1')
        self.wait = wait_s
        self.req = self.create_publisher(String, '/plate/request', 10)
        self.caps = {}
        self.done = None
        self.create_subscription(PlateCapture, '/plate/capture', self.on_cap, 50)
        self.create_subscription(PlateResult, '/plate/result', self.on_res, 50)

    def on_cap(self, c):
        self.caps[c.capture_id] = c

    def on_res(self, r):
        if self.done is not None or r.truth_number != CUSTOM:
            return
        self.done = (r, self.caps.get(r.capture_id))

    def set_plate(self, number):
        for pos in ('rear', 'front'):
            m = String()
            m.data = '%s|%s|%s' % (MODEL, pos, number)
            # 连发三次：/plate/request 是易失话题，一次发布可能正好落在插件建立订阅之前
            for _ in range(3):
                self.req.publish(m)
                rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        # 必须先等 plate_gen 被发现到再发：开局那次发布丢过一整条指令，
        # 结果 240 s 窗口里 car_1 一直被读成旧牌，看起来像"卡口没拍到"。
        t0 = time.time()
        while self.req.get_subscription_count() == 0 and time.time() - t0 < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
        print('下发前 /plate/request 订阅者 %d 个（等了 %.1f s）' % (
            self.req.get_subscription_count(), time.time() - t0))
        print('下发自定义号码 %s|%s|%s（车尾/车头都换）' % (MODEL, 'rear+front', CUSTOM))
        self.set_plate(CUSTOM)
        t0 = time.time()
        while rclpy.ok() and self.done is None and time.time() - t0 < self.wait:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.done is None:
            print('!! %d s 内没有拍到 %s（车没过卡口，或识别器没起来）' % (self.wait, CUSTOM))
            rc = 1
        else:
            r, c = self.done
            print('\n=== 端到端结果 ===')
            print('抓拍号      %s' % r.capture_id)
            print('卡口        %s' % r.gate_id)
            print('触发车辆    %s' % (c.vehicle_id if c else '?'))
            print('抓拍图      %s' % (c.image_path if c else '?'))
            print('图像规格    %dx%d %s' % (c.image_width, c.image_height,
                                            c.image_encoding) if c else '')
            print('识别号码    %s' % r.plate_number)
            print('真值号码    %s' % r.truth_number)
            print('逐字命中    %s' % r.matched)
            print('置信 / 耗时 %.2f / %.1f ms' % (r.confidence, r.process_ms))
            print('算法        %s' % r.algorithm)
            print('抓拍时刻    仿真 %.3f s（capture_stamp）/ 识别完成 %.3f s（header.stamp）' % (
                r.capture_stamp.sec + r.capture_stamp.nanosec * 1e-9,
                r.header.stamp.sec + r.header.stamp.nanosec * 1e-9))
            p = r.vehicle_pose.position
            print('车辆位置    x=%.2f y=%.2f z=%.2f  速度 %.2f m/s' % (
                p.x, p.y, p.z, r.speed))
            print('外接框      x=%d y=%d w=%d h=%d' % (
                r.bbox_x, r.bbox_y, r.bbox_w, r.bbox_h))
            rc = 0 if r.matched else 2
        print('\n还原 %s -> %s' % (MODEL, ORIG))
        self.set_plate(ORIG)
        return rc


def main():
    wait_s = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 180
    rclpy.init()
    node = Verify(wait_s)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
