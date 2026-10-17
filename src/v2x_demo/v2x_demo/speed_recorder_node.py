#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务四（三）.(四) 条目 5：测速数据记录存储 + 原始数据导出。

  输入  /speed/capture   v2x_msgs/SpeedCapture   定点瞬时测速抓拍
  输入  /speed/segment   v2x_msgs/SpeedSegment   区间平均测速结果
  输入  /speed/alert     v2x_msgs/SpeedAlert     超速告警
  输入  /speed/stats     v2x_msgs/SpeedStats     每点位流量统计（只留最新一条）
  服务  /speed/export    std_srvs/Trigger        导出 CSV/JSON 并回统计摘要

三类记录分文件落盘（speed_captures / speed_segments / speed_alerts 各一份 JSONL）：
瞬时抓拍和区间均速的字段集完全不同，硬塞进一张表会有一半列恒空；告警单独一份是因为
取证时"要看的就那一堆"，不必从全量里再过滤。导出服务把内存里的三份改写成套 CSV，
并把逐点位的车流量/平均车速/超速次数算进回包 —— 与 /speed/stats 的窗口口径无关，
这里是**自节点启动以来的全量**，两边都对得上才算闭环。

告警行里带 zone_id/vehicle_id/plate_number/speed_kmh/ts/pose，条目 3 要求的
"车辆编号、车牌、瞬时速度、超速时间、位置"四样在告警 CSV 里一行集齐。
"""
import csv
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats

CAP_FIELDS = ['ts', 'sim_ts', 'capture_id', 'zone_id', 'vehicle_id', 'plate_number',
              'speed_kmh', 'speed_pose_kmh', 'speed_peak_kmh', 'limit_kmh',
              'over_kmh', 'over', 'heading_deg', 'beam_angle_deg', 'radar_range_m',
              'noise_kmh', 'x', 'y', 'z', 'alert']
SEG_FIELDS = ['ts', 'sim_ts', 'segment_id', 'zone_id', 'vehicle_id', 'plate_number',
              'entry_ts', 'exit_ts', 'travel_time_s', 'length_m', 'avg_kmh',
              'limit_kmh', 'over_kmh', 'over', 'max_kmh', 'entry_speed_kmh',
              'exit_speed_kmh', 'entry_x', 'entry_y', 'exit_x', 'exit_y', 'alert']
AL_FIELDS = ['ts', 'sim_ts', 'kind', 'ref_id', 'zone_id', 'vehicle_id',
             'plate_number', 'speed_kmh', 'limit_kmh', 'over_kmh', 'x', 'y', 'z',
             'note']


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class SpeedRecorderNode(Node):
    def __init__(self):
        super().__init__('speed_recorder_node')
        self.declare_parameter('record_dir', '/tmp/v2x_records')
        self.declare_parameter('max_keep', 5000)
        self.dir = self.get_parameter('record_dir').value
        self.keep = int(self.get_parameter('max_keep').value)
        os.makedirs(self.dir, exist_ok=True)

        self.caps = []
        self.segs = []
        self.als = []
        self.stats = {}            # zone_id -> 最新一条 SpeedStats 的字典形式
        self.stats_round = 0.0     # 最近一轮统计的 stamp，用它识别"新一轮整表换新"

        self.create_subscription(SpeedCapture, '/speed/capture', self.on_capture, 50)
        self.create_subscription(SpeedSegment, '/speed/segment', self.on_segment, 50)
        self.create_subscription(SpeedAlert, '/speed/alert', self.on_alert, 50)
        self.create_subscription(SpeedStats, '/speed/stats', self.on_stats, 20)
        self.create_service(Trigger, '/speed/export', self.on_export)

        self.get_logger().info('[speed-rec] 记录写入 %s/speed_*.jsonl，导出服务 /speed/export'
                               % self.dir)

    # ---------------------------------------------------------------- 采集
    def append(self, lst, path, rec):
        lst.append(rec)
        if len(lst) > self.keep:
            lst.pop(0)
        with open(path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(rec, ensure_ascii=False) + '\n')

    def on_capture(self, m):
        rec = {
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
            'capture_id': m.capture_id,
            'zone_id': m.zone_id,
            'vehicle_id': m.vehicle_id,
            'plate_number': m.plate_number,
            'speed_kmh': round(float(m.speed_kmh), 3),
            'speed_pose_kmh': round(float(m.speed_pose_kmh), 3),
            'speed_peak_kmh': round(float(m.speed_peak_kmh), 3),
            'limit_kmh': round(float(m.limit_kmh), 3),
            'over_kmh': round(float(m.over_kmh), 3),
            'over': bool(m.over),
            'heading_deg': round(float(m.heading_deg), 2),
            'beam_angle_deg': round(float(m.beam_angle_deg), 2),
            'radar_range_m': round(float(m.radar_range_m), 2),
            'noise_kmh': round(float(m.noise_kmh), 3),
            'x': round(float(m.pose.position.x), 3),
            'y': round(float(m.pose.position.y), 3),
            'z': round(float(m.pose.position.z), 3),
            'alert': '',
        }
        # 与卡口记录同一套规矩：告警标记必须在**追加写盘之前**填好，
        # 否则 JSONL 里那一行永远是空串，只有内存副本带得上。
        if rec['over']:
            rec['alert'] = 'INSTANT'
        self.append(self.caps, os.path.join(self.dir, 'speed_captures.jsonl'), rec)
        self.get_logger().info(
            '[speed-rec] 瞬时 %s %s 牌=%s %.2f/%.2f km/h %s' % (
                m.zone_id, m.vehicle_id, m.plate_number or '-',
                m.speed_kmh, m.limit_kmh, '超速' if m.over else '正常'))

    def on_segment(self, m):
        rec = {
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
            'segment_id': m.segment_id,
            'zone_id': m.zone_id,
            'vehicle_id': m.vehicle_id,
            'plate_number': m.plate_number,
            'entry_ts': round(float(m.entry_ts), 3),
            'exit_ts': round(float(m.exit_ts), 3),
            'travel_time_s': round(float(m.travel_time_s), 3),
            'length_m': round(float(m.length_m), 3),
            'avg_kmh': round(float(m.avg_kmh), 3),
            'limit_kmh': round(float(m.limit_kmh), 3),
            'over_kmh': round(float(m.over_kmh), 3),
            'over': bool(m.over),
            'max_kmh': round(float(m.max_kmh), 3),
            'entry_speed_kmh': round(float(m.entry_speed_kmh), 3),
            'exit_speed_kmh': round(float(m.exit_speed_kmh), 3),
            'entry_x': round(float(m.entry_pose.position.x), 3),
            'entry_y': round(float(m.entry_pose.position.y), 3),
            'exit_x': round(float(m.exit_pose.position.x), 3),
            'exit_y': round(float(m.exit_pose.position.y), 3),
            'alert': 'SEGMENT' if m.over else '',
        }
        self.append(self.segs, os.path.join(self.dir, 'speed_segments.jsonl'), rec)
        self.get_logger().info(
            '[speed-rec] 区间 %s %s 牌=%s L=%.1fm t=%.2fs 均速%.2f/%.2f km/h %s' % (
                m.zone_id, m.vehicle_id, m.plate_number or '-', m.length_m,
                m.travel_time_s, m.avg_kmh, m.limit_kmh,
                '超速' if m.over else '正常'))

    def on_alert(self, m):
        rec = {
            'ts': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
            'kind': 'INSTANT' if m.kind == SpeedAlert.KIND_INSTANT else 'SEGMENT',
            'ref_id': m.ref_id,
            'zone_id': m.zone_id,
            'vehicle_id': m.vehicle_id,
            'plate_number': m.plate_number,
            'speed_kmh': round(float(m.speed_kmh), 3),
            'limit_kmh': round(float(m.limit_kmh), 3),
            'over_kmh': round(float(m.over_kmh), 3),
            'x': round(float(m.pose.position.x), 3),
            'y': round(float(m.pose.position.y), 3),
            'z': round(float(m.pose.position.z), 3),
            'note': m.note,
        }
        self.append(self.als, os.path.join(self.dir, 'speed_alerts.jsonl'), rec)
        # 回填：告警比抓拍/区间晚到不了多少（同一帧内先后 publish），但要兼容
        # 顺序颠倒的情况，所以两侧都找一遍。
        for lst, key in ((self.caps, 'capture_id'), (self.segs, 'segment_id')):
            for r in reversed(lst):
                if r.get(key) == m.ref_id:
                    r['alert'] = rec['kind']
                    break
        self.get_logger().info(
            '[speed-rec] **告警** %s %s %s 牌=%s %.2f 超 %.2f km/h @ (%.1f,%.1f)' % (
                rec['kind'], m.zone_id, m.vehicle_id, m.plate_number or '-',
                m.speed_kmh, m.over_kmh, m.pose.position.x, m.pose.position.y))

    def on_stats(self, m):
        row = {
            'zone_id': m.zone_id,
            'mode': '区间平均' if m.mode == SpeedStats.MODE_SEGMENT else '定点瞬时',
            'road': m.road,
            'limit_kmh': round(float(m.limit_kmh), 3),
            'window_s': round(float(m.window_s), 1),
            'flow': int(m.flow),
            'vehicles': int(m.vehicles),
            'avg_kmh': round(float(m.avg_kmh), 3),
            'max_kmh': round(float(m.max_kmh), 3),
            'min_kmh': round(float(m.min_kmh), 3),
            'over_count': int(m.over_count),
            'over_ratio': round(float(m.over_ratio), 4),
            'total_flow': int(m.total_flow),
            'total_over': int(m.total_over),
            'pending_segment': int(m.pending_segment),
            'sim_ts': round(stamp_s(m.header.stamp), 3),
        }
        st = row['sim_ts']
        if st > self.stats_round:
            # publish_stats 一轮给每个在册点位各发一条、stamp 相同：新轮一到就整表换新。
            # 只做 update 的话，/speed/zones 删掉的运行期点位会永远留在 stats 里，
            # 导出的 speed_stats.csv 就多一行早就不存在的阈值（实测残留 limit=1.0 那条）。
            self.stats_round = st
            self.stats = {m.zone_id: row}
        else:
            self.stats[m.zone_id] = row

    # ---------------------------------------------------------------- 导出
    def write_csv(self, name, fields, rows):
        path = os.path.join(self.dir, name)
        # utf-8-sig：Excel 直接双击打开中文表头/车牌不乱码（卡口那边同款）
        with open(path, 'w', newline='', encoding='utf-8-sig') as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        with open(os.path.join(self.dir, name.replace('.csv', '.json')),
                  'w', encoding='utf-8') as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        return path

    def on_export(self, _req, resp):
        p1 = self.write_csv('speed_captures.csv', CAP_FIELDS, self.caps)
        p2 = self.write_csv('speed_segments.csv', SEG_FIELDS, self.segs)
        p3 = self.write_csv('speed_alerts.csv', AL_FIELDS, self.als)
        rows = [self.stats[k] for k in sorted(self.stats)]
        with open(os.path.join(self.dir, 'speed_stats.csv'), 'w',
                  newline='', encoding='utf-8-sig') as f:
            if rows:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

        # 全量口径：直接从落盘记录重算一遍，与 /speed/stats 的滑动窗口互为对照
        by = {}
        for r in self.caps:
            b = by.setdefault(r['zone_id'], {'cap': 0, 'seg': 0, 'kmh': [], 'over': 0,
                                             'veh': set()})
            b['cap'] += 1
            b['kmh'].append(r['speed_kmh'])
            b['veh'].add(r['vehicle_id'])
            b['over'] += bool(r['over'])
        for r in self.segs:
            b = by.setdefault(r['zone_id'], {'cap': 0, 'seg': 0, 'kmh': [], 'over': 0,
                                             'veh': set()})
            b['seg'] += 1
            b['kmh'].append(r['avg_kmh'])
            b['veh'].add(r['vehicle_id'])
            b['over'] += bool(r['over'])
        detail = '  '.join(
            '%s 流量%d(瞬%d/区%d) 均速%.2f 超速%d 车%d' % (
                k, v['cap'] + v['seg'], v['cap'], v['seg'],
                sum(v['kmh']) / len(v['kmh']) if v['kmh'] else 0.0,
                v['over'], len(v['veh']))
            for k, v in sorted(by.items()))
        n_over = sum(1 for r in self.caps if r['over']) + \
            sum(1 for r in self.segs if r['over'])
        resp.success = True
        resp.message = ('导出 瞬时%d 区间%d 告警%d -> %s / %s / %s；'
                        '超速判定 %d/%d (%.1f%%)；逐点位 %s' % (
                            len(self.caps), len(self.segs), len(self.als),
                            os.path.basename(p1), os.path.basename(p2),
                            os.path.basename(p3), n_over,
                            len(self.caps) + len(self.segs),
                            100.0 * n_over / (len(self.caps) + len(self.segs))
                            if (self.caps or self.segs) else 0.0,
                            detail or '无'))
        self.get_logger().info('[speed-rec] ' + resp.message)
        return resp


def main():
    rclpy.init()
    node = SpeedRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
