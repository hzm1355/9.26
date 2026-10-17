#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五车换牌 + 双卡口识别率统计：一次跑完就把"哪台卡口、哪辆车、成没成"摊开。

为什么不让 shell 传中文：wsl.exe 的引号层会把 UTF-8 参数拆坏，号码一律写死在本文件里。
PlateResult 里没有 vehicle_id（协议上它只描述识别结论），所以按 capture_id 和
/plate/capture 做一次性 join，不在消息里加字段。

用法：python3 gate_roundtrip.py [采集秒数=300]
"""
import collections
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import PlateCapture, PlateResult

# 号码按车道分配：A 环（car_1/car_3）过北环卡口，B 环（car_2/car_4/car_5）过南环卡口。
PLATES = [
    'car_1|rear|沪A12345',
    'car_3|rear|京B88990',
    'car_2|rear|粤Z1A2B3',
    'car_4|rear|苏E66F88',
    'car_5|rear|浙A66666',
]


class RoundTrip(Node):
    def __init__(self, wait_s):
        super().__init__('gate_roundtrip')
        self.wait_s = wait_s
        self.vehicle = {}          # capture_id -> (gate_id, vehicle_id, speed)
        self.rows = []             # (gate, vehicle, speed, plate, truth, matched)
        self.t0 = time.time()

        self.pub = self.create_publisher(String, '/plate/request', 10)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, 50)
        self.create_subscription(PlateResult, '/plate/result', self.on_result, 50)

        t0 = time.time()
        while rclpy.ok() and self.pub.get_subscription_count() == 0 \
                and time.time() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() == 0:
            print('!! /plate/request 无订阅者，plate_gen_node 未运行')
            return
        for p in PLATES:
            self.pub.publish(String(data=p))
            time.sleep(0.3)
        print('已给 %d 辆车下发车牌，采集 %d s（车辆巡航一周约 110 s）...'
              % (len(PLATES), wait_s))

    def on_capture(self, m):
        self.vehicle[m.capture_id] = (m.gate_id, m.vehicle_id, m.speed)

    def on_result(self, r):
        g, v, sp = self.vehicle.get(r.capture_id, (r.gate_id, '?', r.speed))
        self.rows.append((g, v, sp, r.plate_number, r.truth_number, r.matched,
                          r.confidence))
        print('%-16s %-6s %4.1f m/s  %-10s 真值%-10s 置信%.2f %s'
              % (g, v, sp, r.plate_number or '(空)', r.truth_number or '(无)',
                 r.confidence, '命中' if r.matched else '未命中'))

    def run(self):
        while time.time() - self.t0 < self.wait_s:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.report()
        return 0 if self.rows and all(r[5] for r in self.rows if r[4]) else 1

    def report(self):
        tab = collections.defaultdict(lambda: [0, 0])
        for g, v, _sp, _p, truth, matched, _c in self.rows:
            key = (g, v if truth else v + '(无牌)')
            tab[key][0] += 1
            tab[key][1] += matched
        print('\n%-16s %-12s %6s %6s %8s' % ('卡口', '车辆', '抓拍', '命中', '命中率'))
        for (g, v), (n, hit) in sorted(tab.items()):
            print('%-16s %-12s %6d %6d %7.0f%%' % (g, v, n, hit, 100.0 * hit / n))
        tot = sum(n for n, _ in tab.values())
        hits = sum(h for _, h in tab.values())
        print('合计 %d 次抓拍，%d 次逐字命中 (%.0f%%)'
              % (tot, hits, 100.0 * hits / tot if tot else 0.0))


def main():
    rclpy.init()
    node = RoundTrip(float(sys.argv[1]) if len(sys.argv) > 1 else 300)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
