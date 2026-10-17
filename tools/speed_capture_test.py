#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按速度档统计过卡抓拍：条目 3 的"车辆以不同速度通过卡口"取证。

判据不是"识别对不对"这一列就够——高速档真正会坏掉的是**抓拍本身**：车在两帧
位姿之间跑过的距离变长，触发区判定、相机取景窗口、以及抓拍瞬间车头/车尾朝向
都会跟着变。所以这里同时统计每台车在每个卡口被拍到几次（漏拍）、抓拍瞬间的
实测速度分布（配置 6 m/s 不等于抓拍时真的 6 m/s，路口会减速）、以及牌面在图里
的像素宽度（决定运动模糊一个像素代表多少厘米，报告要拿它换算）。

只统计"起跑之后新产生"的抓拍：/plate/capture、/plate/result 都带历史，
新订阅者会先收到上一轮的旧消息，混进来会把 0.7 m/s 的样本算进 6 m/s 档。

用法：python3 speed_capture_test.py <标签> <秒数> [csv路径]
"""
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node

from v2x_msgs.msg import PlateCapture, PlateResult


def yaw_of(q):
    """平面姿态（Gazebo 差速车，roll/pitch 恒为 0）的航向角"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SpeedRun(Node):
    def __init__(self, label, secs, csv_path):
        super().__init__('speed_capture_test')
        self.label, self.secs, self.csv_path = label, secs, csv_path
        self.caps = {}          # capture_id -> PlateCapture
        self.res = {}           # capture_id -> PlateResult
        self.baseline = set()
        self.t_start = None

        self.create_subscription(PlateCapture, '/plate/capture', self.on_cap, 100)
        self.create_subscription(PlateResult, '/plate/result', self.on_res, 100)

    def on_cap(self, m):
        if self.t_start is None:
            return
        self.caps[m.capture_id] = m

    def on_res(self, m):
        if self.t_start is None:
            return
        self.res[m.capture_id] = m

    def run(self):
        # 先静默收 3 s：把上一轮残留在 latched/在途消息里的旧抓拍记成基线
        t0 = time.time()
        while time.time() - t0 < 3:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.baseline = set(self.res) | set(self.caps)
        self.caps, self.res = {}, {}
        self.t_start = time.time()
        while time.time() - self.t_start < self.secs:
            rclpy.spin_once(self, timeout_sec=0.1)

        rows = []
        for cid, r in sorted(self.res.items()):
            if cid in self.baseline:
                continue
            c = self.caps.get(cid)
            rows.append({
                'label': self.label,
                'capture_id': cid,
                'gate_id': r.gate_id,
                'vehicle_id': (c.vehicle_id if c else ''),
                'speed_ms': round(c.speed, 3) if c else '',
                'capture_x': round(c.pose.position.x, 2) if c else '',
                'capture_y': round(c.pose.position.y, 2) if c else '',
                'yaw_deg': round(math.degrees(yaw_of(c.pose.orientation)), 1)
                          if c else '',
                'plate': r.plate_number or '(空)',
                'truth': r.truth_number or '(无)',
                'matched': bool(r.matched),
                'confidence': round(float(r.confidence), 4),
                'process_ms': round(float(r.process_ms), 1),
                'bbox_w': int(r.bbox_w),
                'bbox_h': int(r.bbox_h),
                'image_path': r.capture_id,
            })
        rows.sort(key=lambda d: d['capture_id'])

        if rows:
            d = os.path.dirname(self.csv_path)
            if d and not os.path.exists(d):
                os.makedirs(d)
            new = not os.path.exists(self.csv_path)
            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                if new:
                    w.writeheader()
                w.writerows(rows)

        gates, veh = {}, {}
        for r in rows:
            g = gates.setdefault(r['gate_id'], [0, 0, []])
            g[0] += 1
            g[1] += 1 if r['matched'] else 0
            if r['speed_ms'] != '':
                g[2].append(r['speed_ms'])
            v = veh.setdefault(r['vehicle_id'], set())
            v.add(r['gate_id'])
        print('\n[%s] 窗口 %d s，抓拍 %d 张，逐字命中 %d 张' % (
            self.label, self.secs, len(rows), sum(1 for r in rows if r['matched'])))
        for gid, (n, hit, sp) in sorted(gates.items()):
            print('  %-15s 抓拍 %-3d 命中 %-3d (%3.0f%%)  实测速度 %.2f~%.2f 均 %.2f m/s  牌宽 %s px' % (
                gid, n, hit, 100.0 * hit / n if n else 0,
                min(sp), max(sp), sum(sp) / len(sp),
                ','.join(str(r['bbox_w']) for r in rows if r['gate_id'] == gid)[:60]))
        for vid, gs in sorted(veh.items()):
            print('  %-7s 覆盖卡口 %d/2 %s' % (vid, len(gs), sorted(gs)))
        for r in rows:
            if not r['matched']:
                print('  !! 未命中 %s %s 识别=%s 真值=%s 速度=%s' % (
                    r['capture_id'], r['gate_id'], r['plate'], r['truth'], r['speed_ms']))
        return 0


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else 'run'
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 120
    csv_path = sys.argv[3] if len(sys.argv) > 3 else '/tmp/v2x_records/speed_sweep.csv'
    rclpy.init()
    node = SpeedRun(label, secs, csv_path)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
