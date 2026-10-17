#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 CSV 语义核对导出记录：告警行、含逗号的光照字段、以及车辆×卡口命中分布。

custom:0.03,0.02,0.05,0,0 这类预设名本身含逗号，naive 的 awk -F, 会多切四列，
所以量 alert_level 必须用 csv.DictReader（DictWriter 默认 QUOTE_MINIMAL 会加引号）。
车辆×卡口那张表用来挑"哪个卡口取景最稳的车"，也是空号主要落在谁身上的证据。
"""
import csv
import sys
from collections import Counter

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records/plate_records.csv'
with open(path, newline='', encoding='utf-8-sig') as f:
    rows = list(csv.DictReader(f))
print('行数 %d，列数 %d，字段为 None 的行 %d' % (
    len(rows), len(rows[0]),
    sum(1 for r in rows if any(v is None for v in r.values()))))

hits = [r for r in rows if r.get('alert_level')]
print('带 alert_level 的行 %d:' % len(hits))
for r in hits:
    print('  %s %s %s 识别=%s 真值=%s 命中=%s -> %s' % (
        r['ts'], r['capture_id'], r['gate_id'], r['plate_number'],
        r['truth_number'], r['matched'], r['alert_level']))
print('光照字段取值: %s' % dict(Counter(r['lighting'] for r in rows)))

by = {}
for r in rows:
    k = (r['vehicle_id'], r['gate_id'])
    n, h, e = by.get(k, (0, 0, 0))
    by[k] = (n + 1, h + (r['matched'] == 'True'),
             e + (1 if not r['plate_number'] else 0))
print('车辆 × 卡口  命中/抓拍 (命中率) 空号：')
for k in sorted(by):
    n, h, e = by[k]
    print('  %-7s %-14s %3d/%3d (%3.0f%%) 空号 %d' % (
        k[0], k[1], h, n, 100.0 * h / max(1, n), e))
