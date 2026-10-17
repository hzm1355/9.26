#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从导出记录里把"非内置算法"的条目挑出来，验证外部算法的结果被下游原样收下。

用法：python3 show_external_records.py [csv] [起始时间 "YYYY-MM-DD HH:MM:SS"]
导出清单是跨场次累计的，第二个参数把结果限制在"本次录制开始之后"，
否则表里会混进上一轮桩运行的条目，条数就对不上本次桩自己报的输出数。
"""
import csv
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records/plate_records.csv'
since = sys.argv[2] if len(sys.argv) > 2 else ''
rows = list(csv.DictReader(open(path, newline='', encoding='utf-8-sig')))
ext = [r for r in rows if r['algorithm'] != 'builtin_opencv' and r['ts'] >= since]
print('外部算法记录 %d 条（ts >= "%s"；其余 %d 条是内置 builtin_opencv）：' % (
    len(ext), since or '不限', len(rows) - len(ext)))
for r in ext[-8:]:
    print('  %s %-18s %-20s 号=%s 时刻=%s 位置=(%s, %s, %s) 速度=%s' % (
        r['ts'], r['algorithm'], r['capture_id'], r['plate_number'], r['sim_ts'],
        r['x'], r['y'], r['z'], r['speed']))
