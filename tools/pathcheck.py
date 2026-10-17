#!/usr/bin/env python3
"""分析 /tmp/signal_drive_path.csv：按绝对位移识别瞬移(复位)，统计里程/越界/超速。"""
import csv, math, sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
rows = list(csv.reader(open(path)))
rows = [r for r in rows if len(r) >= 8 and r[0] not in ('t', '')]
data = defaultdict(list)
for r in rows:
    try:
        data[r[1]].append((float(r[0]), float(r[2]), float(r[3]), float(r[4]),
                           float(r[5]), r[6], r[7]))
    except ValueError:
        continue

print(f'车辆={len(data)} 总样本={sum(len(v) for v in data.values())}')
print(f'{"车":<7}{"样本":>6}{"时长s":>8}{"里程m":>8}{"峰值v":>7}{"瞬移":>6}{"越界":>6}{"红灯停样本":>11}{"停比":>7}')
for name in sorted(data):
    seq = data[name]
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(seq, seq[1:]))
    tp = sum(1 for a, b in zip(seq, seq[1:])
             if math.hypot(b[1] - a[1], b[2] - a[2]) > 2.0)
    oob = sum(1 for s in seq if abs(s[1]) > 23.0 or abs(s[2]) > 23.0)
    stp = sum(1 for s in seq if s[4] < 0.05)
    red = sum(1 for s in seq if s[5] and 'stop' in s[5] or s[5] in ('',))
    print(f'{name:<7}{len(seq):>6}{seq[-1][0] - seq[0][0]:>8.0f}{dist:>8.1f}'
          f'{max(s[4] for s in seq):>7.2f}{tp:>6}{oob:>6}{stp:>11}{stp / len(seq):>7.0%}')

tot = defaultdict(int)
for name in data:
    for s in data[name]:
        if s[5]:
            tot[s[5].split('(')[0]] += 1
print('\n限停原因分布(top):', ', '.join(f'{k}={v}' for k, v in sorted(tot.items(), key=lambda kv: -kv[1])[:12]))
