#!/usr/bin/env python3
"""167：把 pk166 落盘的 shuttle 状态行摊成时间序列表，看朝向误差是怎么翻转的。

状态行里能取到的口径：目标车头（航点要求）、实测航向（/model_states 位姿）、
指令 v/w（**整形之后**真正发出去的那一拍）、note（本航点已累计转向 / 翻转次数）。
逐拍差 = wrap(目标车头 − 实测航向)。用法：
  python3 pk167_aligntab308.py <log> <起> <止> [每几步打一行]
"""
import math
import re
import sys

ROW = re.compile(
    r'^(?P<t>[\d.]+) \S+ phase=(?P<ph>\S+).*?车头=(?P<th>[-+\d.]+).*?'
    r'指令 v=(?P<v>[-+\d.]+) w=(?P<w>[-+\d.]+).*?｜(?P<note>.*)$')
POSE = re.compile(r'位置=\((?P<x>[-+\d.]+),(?P<y>[-+\d.]+)\) 距离=(?P<d>[-+\d.]+) '
                  r'航向=(?P<yaw>[-+\d.]+)')


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 4
    rows = []
    for line in open(path, encoding='utf-8', errors='replace'):
        m = ROW.match(line)
        if not m:
            continue
        t = float(m.group('t'))
        if not (t0 <= t <= t1):
            continue
        p = POSE.search(line)
        rows.append((t, m.group('ph'), float(m.group('th')), m.group('v'),
                     float(m.group('w')), p, m.group('note').strip()))
    # 状态行每拍发两条（同一指令的两个订阅端回调各记一次时间戳），按 (t 取整到 0.01)
    # 和实测航向去重，否则表里全是重复行，翻转次数会被数成两倍。
    seen = set()
    keep = []
    for r in rows:
        key = (round(r[0], 2), r[2], r[4])
        if key in seen:
            continue
        seen.add(key)
        keep.append(r)
    print('窗口内 %d 拍（原始 %d 行）' % (len(keep), len(rows)))
    print('%-9s %-8s %6s %6s %6s %6s %6s %5s %s' % (
        'sim时间', 'phase', '目标', '实测', '误差', '指令w', '累计转', 'v', 'note'))
    flips = 0
    prev = None
    for i, (t, ph, th, v, w, p, note) in enumerate(keep):
        if p is None:
            continue
        yaw = float(p.group('yaw'))
        err = wrap(th - yaw)
        if prev is not None and err * prev < 0 and abs(err) > 0.05:
            flips += 1
        prev = err
        if i % step:
            continue
        acc = re.search(r'累计转向 (\d+\.\d)', note)
        print('%-9.1f %-8s %+6.2f %+6.2f %+6.2f %+6.2f %6s %5s %s' % (
            t, ph, th, yaw, err, w, acc.group(1) if acc else '-', v,
            note[:56]))
    print('误差过零（>0.05 rad 才算翻转）次数：%d' % flips)


if __name__ == '__main__':
    main()
