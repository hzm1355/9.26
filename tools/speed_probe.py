#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测速话题探针：订阅 /speed/* 若干秒，把每条抓拍/区间/告警/统计原样打出来。

用法：
  python3 speed_probe.py 180            # 采 180 s
  python3 speed_probe.py 180 /tmp/x.txt # 同时写文件（取证脚本要往片内灌证据）

开跑前先**spin 着等发布者**（不是看进程在不在）：/speed/capture 一个发布者都没有，
就说明检测节点没起来或还没被 DDS 发现，这时跑满 180 s 只会得到一张空表——
任务三为此作废过两轮整轮取证，见 memory「长跑脚本先预检依赖」。
"""
import sys
import time

import rclpy
from rclpy.node import Node

from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats

TOPICS = [('/speed/capture', SpeedCapture), ('/speed/segment', SpeedSegment),
          ('/speed/alert', SpeedAlert), ('/speed/stats', SpeedStats)]


class Probe(Node):
    def __init__(self, out):
        super().__init__('speed_probe')
        self.rows = {'cap': [], 'seg': [], 'alert': [], 'stats': {}}
        self.out = out
        for topic, T in TOPICS:
            key = {SpeedCapture: 'cap', SpeedSegment: 'seg',
                   SpeedAlert: 'alert', SpeedStats: 'stats'}[T]
            self.create_subscription(T, topic, self.make_cb(key), 50)

    def make_cb(self, key):
        def cb(m):
            line = self.fmt(key, m)
            if key == 'stats':
                self.rows['stats'][m.zone_id] = line
            else:
                self.rows[key].append(line)
            self.emit(line)
        return cb

    @staticmethod
    def fmt(key, m):
        if key == 'cap':
            return ('[瞬时] sim_t=%.2fs %s %s 牌=%s 雷达=%.2f 位姿=%.2f 峰=%.2f 阈值=%.2f '
                    '夹角=%.1f° 距=%.1fm %s @ (%.2f,%.2f) %s' % (
                        m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                        m.zone_id, m.vehicle_id, m.plate_number or '-', m.speed_kmh,
                        m.speed_pose_kmh, m.speed_peak_kmh, m.limit_kmh,
                        m.beam_angle_deg, m.radar_range_m,
                        '**超速**' if m.over else '正常',
                        m.pose.position.x, m.pose.position.y, m.capture_id))
        if key == 'seg':
            return ('[区间] %s %s 牌=%s L=%.1fm t=%.2fs 均速=%.2f 峰=%.2f 阈值=%.2f %s '
                    '(%s->%s) %s' % (
                        m.zone_id, m.vehicle_id, m.plate_number or '-', m.length_m,
                        m.travel_time_s, m.avg_kmh, m.max_kmh, m.limit_kmh,
                        '**超速**' if m.over else '正常',
                        ('%.1f' % m.entry_speed_kmh), ('%.1f' % m.exit_speed_kmh),
                        m.segment_id))
        if key == 'alert':
            return ('**告警** %s %s %s 牌=%s %.2f/%.2f 超 %.2f @ (%.2f,%.2f) 时刻 %.2f %s' % (
                '瞬时' if m.kind == SpeedAlert.KIND_INSTANT else '区间',
                m.zone_id, m.vehicle_id, m.plate_number or '-', m.speed_kmh,
                m.limit_kmh, m.over_kmh, m.pose.position.x, m.pose.position.y,
                m.ts, m.note))
        return ('[统计] %s %s(%s) 阈值%.2f 窗%.0fs 流量%d 车%d 均%.2f 峰%.2f 谷%.2f '
                '超速%d(%.0f%%) 累计%d/%d 在途%d' % (
                    m.zone_id, '区间' if m.mode == SpeedStats.MODE_SEGMENT else '瞬时',
                    m.road or '-', m.limit_kmh, m.window_s, m.flow, m.vehicles,
                    m.avg_kmh, m.max_kmh, m.min_kmh, m.over_count,
                    100.0 * m.over_ratio, m.total_flow, m.total_over,
                    m.pending_segment))

    def emit(self, line):
        print(line, flush=True)
        if self.out:
            with open(self.out, 'a', encoding='utf-8') as f:
                f.write(line + '\n')


def count_publishers(node, topic, deadline=15.0):
    t0 = time.time()
    while time.time() - t0 < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        n = node.count_publishers(topic)
        if n:
            return n
    return 0


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
    out = sys.argv[2] if len(sys.argv) > 2 else ''
    rclpy.init()
    node = Probe(out)
    ready = [t for t, _ in TOPICS if count_publishers(node, t)]
    if not ready:
        print('预检失败：/speed/* 四个话题一个发布者都没有，检测节点没起来。中止。',
              flush=True)
        rclpy.shutdown()
        sys.exit(2)
    print('预检通过：有发布者的话题 %s（其余暂无数据源，属正常空窗）' %
          ' '.join(ready), flush=True)
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(node, timeout_sec=0.2)
    print('--- %0.fs 小结：瞬时 %d 条 / 区间 %d 条 / 告警 %d 条 / 统计点位 %d 个 ---' % (
        dur, len(node.rows['cap']), len(node.rows['seg']), len(node.rows['alert']),
        len(node.rows['stats'])), flush=True)
    for line in list(node.rows['stats'].values()):
        print(line, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
