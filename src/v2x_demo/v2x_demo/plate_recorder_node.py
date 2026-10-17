#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三（三）条目 4/5：识别记录落盘 + 数据导出 + 黑名单告警。

  输入  /plate/result        v2x_msgs/PlateResult    识别结论
  输入  /plate/capture       v2x_msgs/PlateCapture   原始抓拍（取 image_path）
  输入  /gate/lighting/current std_msgs/String       当前光照预设，进记录字段
  服务  /plate/blacklist     v2x_msgs/SetBlacklist   黑名单增删查
  服务  /plate/export        std_srvs/Trigger        导出 CSV/JSON 并回统计
  输出  /plate/alert         v2x_msgs/PlateAlert     命中黑名单时逐条告警

记录是**逐条追加**到 plate_records.jsonl 的：识别是低频事件（一辆车一圈一次），
JSONL 既能 tail -f 看，也是 pandas/awk 直接吃的格式，不必为了"导出"再造一层
数据库。导出服务只是把已累积的记录改写成 CSV 并顺手算一遍成功率。

光照预设必须进记录，否则条目 3 的"逆光/弱光/夜间识别率"事后无从区分——
抓拍图和识别结论里都不含光照信息，只有当时生效的预设名能把它钉住。
"""
import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from v2x_msgs.msg import PlateAlert, PlateCapture, PlateResult
from v2x_msgs.srv import SetBlacklist

LEVEL_NAME = {0: 'INFO', 1: 'WARN', 2: 'CRITICAL'}
CSV_FIELDS = ['ts', 'sim_ts', 'capture_id', 'gate_id', 'vehicle_id', 'plate_number',
              'truth_number', 'matched', 'confidence', 'algorithm', 'process_ms',
              'speed', 'x', 'y', 'z', 'image_path', 'lighting', 'alert_level']


class PlateRecorderNode(Node):
    def __init__(self):
        super().__init__('plate_recorder_node')
        self.declare_parameter('record_dir', '/tmp/v2x_records')
        self.declare_parameter('max_keep', 5000)
        self.dir = self.get_parameter('record_dir').value
        self.keep = int(self.get_parameter('max_keep').value)
        os.makedirs(self.dir, exist_ok=True)
        self.jsonl = os.path.join(self.dir, 'plate_records.jsonl')

        self.records = []
        self.captures = {}          # capture_id -> PlateCapture（只留 image_path 等轻字段）
        self.blacklist = {}         # plate -> (reason, level)
        self.lighting = 'day'
        self.alerts = 0

        self.alert_pub = self.create_publisher(PlateAlert, '/plate/alert', 50)
        self.create_subscription(PlateResult, '/plate/result', self.on_result, 50)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, 50)
        self.create_subscription(String, '/gate/lighting/current', self.on_light, 10)
        self.create_service(SetBlacklist, '/plate/blacklist', self.on_blacklist)
        self.create_service(Trigger, '/plate/export', self.on_export)

        self.get_logger().info('[recorder] 记录写入 %s，黑名单服务 /plate/blacklist'
                               % self.jsonl)

    # ---------------------------------------------------------------- 采集
    def on_light(self, m):
        self.lighting = m.data

    def on_capture(self, m):
        self.captures[m.capture_id] = m
        while len(self.captures) > self.keep:
            self.captures.pop(next(iter(self.captures)))

    def on_result(self, r):
        cap = self.captures.get(r.capture_id)
        rec = {
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': r.capture_stamp.sec + r.capture_stamp.nanosec * 1e-9,
            'capture_id': r.capture_id,
            'gate_id': r.gate_id,
            'vehicle_id': cap.vehicle_id if cap else '',
            'plate_number': r.plate_number,
            'truth_number': r.truth_number,
            'matched': bool(r.matched),
            'confidence': round(float(r.confidence), 4),
            'algorithm': r.algorithm,
            'process_ms': round(float(r.process_ms), 2),
            'speed': round(float(r.speed), 3),
            'x': round(float(r.vehicle_pose.position.x), 3),
            'y': round(float(r.vehicle_pose.position.y), 3),
            'z': round(float(r.vehicle_pose.position.z), 3),
            'image_path': cap.image_path if cap else '',
            'lighting': self.lighting,
            'alert_level': '',
        }
        # 黑名单判定必须在落盘**之前**：JSONL 是追加写的，先写后回填就只有内存里的
        # 记录带 alert_level，文件里那一行永远是空串——CSV 导出看着是对的，
        # 谁要是直接拿 jsonl 做统计，告警就凭空消失了（实测踩过）。
        hit = self.blacklist.get(r.plate_number) if r.plate_number else None
        if hit:
            rec['alert_level'] = LEVEL_NAME.get(hit[1], str(hit[1]))
        self.records.append(rec)
        if len(self.records) > self.keep:
            self.records.pop(0)
        with open(self.jsonl, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')

        if hit:
            self.raise_alert(r, cap, hit)
        self.get_logger().info(
            '[recorder] %s %s -> %s (%s) 置信%.2f %s%s' % (
                r.gate_id, rec['vehicle_id'] or '?', r.plate_number or '(空)',
                self.lighting, r.confidence,
                '命中' if r.matched else '未命中',
                '  **黑名单告警**' if hit else ''))

    def raise_alert(self, r, cap, hit):
        reason, level = hit
        a = PlateAlert()
        a.header.stamp = self.get_clock().now().to_msg()
        a.level = level
        a.plate_number = r.plate_number
        a.reason = reason
        a.capture_id = r.capture_id
        a.gate_id = r.gate_id
        a.vehicle_pose = r.vehicle_pose
        a.speed = r.speed
        a.image_path = cap.image_path if cap else ''
        self.alert_pub.publish(a)
        self.alerts += 1

    # ---------------------------------------------------------------- 黑名单
    def on_blacklist(self, req, resp):
        if req.op == SetBlacklist.Request.OP_ADD:
            for i, num in enumerate(req.plate_numbers):
                reason = req.reasons[i] if i < len(req.reasons) else '未注明'
                level = (req.levels[i] if i < len(req.levels)
                         else PlateAlert.LEVEL_WARN)
                self.blacklist[num] = (reason, level)
            resp.success = True
            resp.message = '已加入 %d 条' % len(req.plate_numbers)
        elif req.op == SetBlacklist.Request.OP_REMOVE:
            for num in req.plate_numbers:
                self.blacklist.pop(num, None)
            resp.success = True
            resp.message = '已移除'
        elif req.op == SetBlacklist.Request.OP_CLEAR:
            n = len(self.blacklist)
            self.blacklist.clear()
            resp.success = True
            resp.message = '已清空 %d 条' % n
        else:
            resp.success = True
            resp.message = '当前 %d 条' % len(self.blacklist)
        resp.blacklist = list(self.blacklist.keys())
        resp.blacklist_reasons = [v[0] for v in self.blacklist.values()]
        resp.blacklist_levels = [v[1] for v in self.blacklist.values()]
        return resp

    # ---------------------------------------------------------------- 导出
    def on_export(self, _req, resp):
        base = os.path.splitext(self.jsonl)[0]
        with open(base + '.csv', 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(self.records)
        with open(base + '.json', 'w', encoding='utf-8') as f:
            json.dump(self.records, f, ensure_ascii=False, indent=1)

        graded = [r for r in self.records if r['truth_number']]
        hits = [r for r in graded if r['matched']]
        by = {}
        for r in graded:
            k = r['lighting'] + '/' + r['gate_id']
            by.setdefault(k, [0, 0])
            by[k][0] += 1
            by[k][1] += r['matched']
        detail = '  '.join('%s %d/%d' % (k, h, n) for k, (n, h) in sorted(by.items()))
        resp.success = True
        resp.message = ('导出 %d 条 -> %s.csv/.json；逐字命中 %d/%d (%.0f%%)；'
                        '黑名单告警 %d 次；分组 %s' % (
                            len(self.records), base, len(hits), len(graded),
                            100.0 * len(hits) / len(graded) if graded else 0.0,
                            self.alerts, detail or '无'))
        self.get_logger().info('[recorder] ' + resp.message)
        return resp


def main():
    rclpy.init()
    node = PlateRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
