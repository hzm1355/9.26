#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 speed_sweep.csv 汇成报告里的 markdown 表（每档 × 每卡口一行）。

用法：python3 tools/sweep_table.py <csv> [档位标签...]
不给档位标签就按 CSV 里出现过的顺序全打。
命中 = 识别号 == 真值号；串牌单独计数，因为它跟速度不是一条轴上的失效。
"""
import collections
import sys


def main():
    path = sys.argv[1]
    want = sys.argv[2:]
    rows = [l.rstrip('\n').split(',') for l in open(path, encoding='utf-8')][1:]
    rows = [r for r in rows if len(r) >= 15]

    g = collections.OrderedDict()
    for r in rows:
        label, cid, gate, veh, spd = r[0], r[1], r[2], r[3], r[4]
        plate, truth, matched = r[8], r[9], r[10]
        conf, w = r[11], r[13]
        if want and label not in want:
            continue
        k = (label, gate)
        d = g.setdefault(k, dict(n=0, hit=0, empty=0, cross=0, spds=[], ws=[], conf=[]))
        d['n'] += 1
        try:
            d['spds'].append(float(spd))
        except ValueError:
            pass
        if w not in ('', '0'):
            try:
                d['ws'].append(float(w))
            except ValueError:
                pass
        if matched == 'True':
            d['hit'] += 1
            try:
                d['conf'].append(float(conf))
            except ValueError:
                pass
        elif plate in ('', '-', '未识别', 'none', 'None', '(空)'):
            d['empty'] += 1
        else:
            d['cross'] += 1

    print('| 配置速度 | 卡口 | 抓拍 | 命中 | 命中率 | 空号 | 串牌 | 抓拍瞬间实测速度 | 牌宽 px |')
    print('|---|---|---|---|---|---|---|---|---|')
    for (label, gate), d in sorted(g.items(), key=lambda x: (float(x[0][0]), x[0][1])):
        rate = 100.0 * d['hit'] / d['n'] if d['n'] else 0.0
        sp = '%.1f~%.1f' % (min(d['spds']), max(d['spds'])) if d['spds'] else '-'
        wp = '%.0f' % (sum(d['ws']) / len(d['ws'])) if d['ws'] else '-'
        print('| %s m/s | %s | %d | %d | %.0f%% | %d | %d | %s | %s |'
              % (label, gate, d['n'], d['hit'], rate, d['empty'], d['cross'], sp, wp))

    print()
    print('## 按**抓拍瞬间实测速度**分桶（真正的速度—识别率曲线）')
    print()
    print('配置速度只是名义值：北环卡口压在 J3 停车线上，排队/复位会让车在过卡时')
    print('根本不是那个速度。所以结论按实测速度分桶，停车（0~0.2）单独一桶，')
    print('它量到的是"被前车挡住"而不是"跑太快糊了"。')
    print()
    BINS = [(0.0, 0.2, '停稳(排队/红灯)'), (0.2, 1.2, '0.2~1.2'),
            (1.2, 2.5, '1.2~2.5'), (2.5, 4.5, '2.5~4.5'),
            (4.5, 7.0, '4.5~7.0'), (7.0, 99.0, '7.0+')]
    b = {k: dict(n=0, hit=0, empty=0, cross=0) for k in BINS}
    for r in rows:
        if want and r[0] not in want:
            continue
        try:
            v = float(r[4])
        except ValueError:
            continue
        for lo, hi, lab in BINS:
            if lo <= v < hi:
                d = b[(lo, hi, lab)]
                d['n'] += 1
                if r[10] == 'True':
                    d['hit'] += 1
                elif r[8] in ('', '-', '(空)'):
                    d['empty'] += 1
                else:
                    d['cross'] += 1
                break
    print('| 实测速度 m/s | 抓拍 | 命中 | 命中率 | 空号 | 串牌 |')
    print('|---|---|---|---|---|---|')
    for key in BINS:
        d = b[key]
        if not d['n']:
            continue
        print('| %s | %d | %d | %.0f%% | %d | %d |'
              % (key[2], d['n'], d['hit'], 100.0 * d['hit'] / d['n'], d['empty'], d['cross']))

    print()
    print('未命中明细：')
    for r in rows:
        if r[10] == 'True':
            continue
        if want and r[0] not in want:
            continue
        print('  %s %-18s %-6s 实测%.1f m/s 读出=%s 真值=%s 牌宽=%s'
              % (r[0], r[1], r[3], float(r[4] or 0), r[8] or '(空)', r[9], r[13] or '-'))


if __name__ == '__main__':
    main()
