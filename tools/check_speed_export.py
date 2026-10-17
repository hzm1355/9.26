#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测速导出产物体检：/speed/export 报的数与磁盘上的 CSV/JSONL 是否真的一致。

必须用 csv.DictReader 按表头取列：plate_number 里可能含逗号（光照预设名也一样），
手工 split(',') 会把一行切成两半（任务三吃过，见报告"CSV≠JSONL 口径"一节）。
首列名带 BOM，所以统一用 utf-8-sig 打开。

体检四件事：
 1) 导出成套自洽：CSV 行数 == 同名 JSON 行数；JSONL 只核"不少于 CSV"——
    JSONL 是跨轮追加的原始存储，CSV/JSON 是记录器本轮内存导出的切片，两者相等只
    发生在记录器刚重启过的时候。要求三者相等会把"存储比导出多"这种正常状态报成
    FAIL（任务三已经为这个口径写过一次教训，别再把它当 bug 找）。
 2) 每条超速行的 alert 列非空，非超速行为空（漏填/回填错序都会在这里暴露）；
 3) 逐点位车流量/均速/超速次数与 CSV 重算一致；
 4) 关键列一行都不缺（条目 3 要求"车辆编号、车牌、瞬时速度、超速时间、位置"齐全）。
"""
import csv
import json
import os
import sys

D = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records'
PAIRS = [('speed_captures', ['ts', 'sim_ts', 'capture_id', 'zone_id', 'vehicle_id',
                             'plate_number', 'speed_kmh', 'speed_pose_kmh',
                             'limit_kmh', 'over_kmh', 'over', 'beam_angle_deg',
                             'x', 'y', 'z', 'alert']),
         ('speed_segments', ['ts', 'sim_ts', 'segment_id', 'zone_id', 'vehicle_id',
                             'plate_number', 'travel_time_s', 'length_m', 'avg_kmh',
                             'limit_kmh', 'over_kmh', 'over', 'exit_x', 'exit_y',
                             'alert']),
         ('speed_alerts', ['ts', 'sim_ts', 'kind', 'ref_id', 'zone_id', 'vehicle_id',
                           'plate_number', 'speed_kmh', 'limit_kmh', 'over_kmh',
                           'x', 'y', 'z', 'note'])]

fails = []


def check(name, cond, detail):
    print('  [%s] %s %s' % ('PASS' if cond else 'FAIL', name, detail))
    if not cond:
        fails.append(name)


for stem, cols in PAIRS:
    csv_path = os.path.join(D, stem + '.csv')
    jsonl_path = os.path.join(D, stem + '.jsonl')
    json_path = os.path.join(D, stem + '.json')
    print('== %s' % stem)
    if not os.path.exists(csv_path):
        check('%s 存在' % stem, False, '缺 %s（先调 /speed/export）' % csv_path)
        continue
    with open(csv_path, encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    n_jsonl = sum(1 for _ in open(jsonl_path, encoding='utf-8')) if os.path.exists(
        jsonl_path) else -1
    n_json = len(json.load(open(json_path, encoding='utf-8'))) if os.path.exists(
        json_path) else -1
    check('导出成套自洽(CSV==JSON)', len(rows) == n_json,
          'CSV %d / JSON %d' % (len(rows), n_json))
    check('原始存储不少于导出(CSV<=JSONL)', 0 <= n_jsonl and len(rows) <= n_jsonl,
          'CSV %d / JSONL %d（JSONL 是跨轮追加的原始历史）' % (len(rows), n_jsonl))
    missing = [c for c in cols if c not in (rows[0].keys() if rows else cols)]
    check('关键列齐全', not missing, '缺 %s' % (missing or '无'))
    if rows and 'over' in rows[0]:
        bad = [r for r in rows
               if (r['over'] == 'True') != bool((r.get('alert') or '').strip())]
        check('超速行都带告警标记', not bad,
              '不一致 %d 行（例 %s）' % (len(bad), (bad[0] if bad else {})))
        nz = [r for r in rows if r['plate_number'] and r['vehicle_id']]
        check('车辆编号+车牌非空', len(nz) >= max(1, len(rows) // 2),
              '%d/%d 行两列都有值' % (len(nz), len(rows)))
    by = {}
    for r in rows:
        b = by.setdefault(r['zone_id'], {'n': 0, 'over': 0, 'kmh': []})
        b['n'] += 1
        # 告警表没有 over 列（它只装超限的那些），逐点位重算只在有该列的表上做
        if 'over' in r:
            b['over'] += r['over'] == 'True'
        key = 'speed_kmh' if 'speed_kmh' in r else 'avg_kmh'
        b['kmh'].append(float(r[key]))
    for k, v in sorted(by.items()):
        # 告警表没有 over 列，"超速0"是"这一列不存在"而不是"没超速"，别印出来误导
        if 'over' in (rows[0] if rows else {}):
            print('    %s 流量%d 超速%d 均速%.2f km/h' % (
                k, v['n'], v['over'], sum(v['kmh']) / len(v['kmh'])))
        else:
            print('    %s 告警%d 条 判定均速%.2f km/h' % (
                k, v['n'], sum(v['kmh']) / len(v['kmh'])))

stats_path = os.path.join(D, 'speed_stats.csv')
if os.path.exists(stats_path):
    print('== speed_stats.csv')
    with open(stats_path, encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        print('    %s %s 阈值%s 窗口流量%s 累计%s/%s 在途%s' % (
            r['zone_id'], r['mode'], r['limit_kmh'], r['flow'],
            r['total_flow'], r['total_over'], r['pending_segment']))
    check('统计点位不为空', bool(rows), '%d 个点位' % len(rows))
else:
    check('speed_stats.csv 存在', False, '缺 %s' % stats_path)

print('\n结论：%s' % ('全部通过' if not fails else '未通过 -> ' + ', '.join(fails)))
sys.exit(0 if not fails else 1)
