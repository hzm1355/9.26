#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查 cmd_pose_trace 的表有没有把"同一个时钟刻里的第二帧"保住。

判据的根据是老代码里那句 `if t - prev[0] <= 1e-6: continue` —— 它按构造就不会写出
两行 t 相同的记录。所以：
  · 老代码的表：同 t 行 0 条（这就是被吞掉的那些帧的去向）；
  · 新代码的表：瞬移那几格会出现同 t 的相邻行，且这些行的 jump 累加要能对上首末净位移。

用法（WSL）：
  python3 tools/dedupe_check.py --jump=0.3 /mnt/d/desktop1/9.26.1/_tr77e/cmd_pose_car_5.csv
  python3 tools/dedupe_check.py --jump=0.3 /mnt/d/desktop1/9.26.1/_tr77d/cmd_pose_car_5.csv
"""
import csv
import math
import sys

JUMP_DEFAULT = 0.3


def load(path):
    with open(path, encoding='utf-8-sig', newline='') as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k in r:
            r[k] = float(r[k])
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--jump=')]
    jump_lo = JUMP_DEFAULT
    for a in sys.argv[1:]:
        if a.startswith('--jump='):
            jump_lo = float(a.split('=', 1)[1])
    if not args:
        print('用法：dedupe_check.py --jump=0.3 <cmd_pose_car_*.csv>')
        return 2

    for path in args:
        rows = load(path)
        print('\n===== %s（%d 行，t %.1f→%.1f）' % (path, len(rows), rows[0]['t'], rows[-1]['t']))
        big = [i for i, r in enumerate(rows) if r['jump'] >= jump_lo]
        same_t = [i for i in big if i > 0 and rows[i - 1]['t'] == rows[i]['t']]
        print('  jump>=%g 的行：%d 条，其中与前一行同 t 的：%d 条' % (jump_lo, len(big), len(same_t)))
        print('  （老代码按 sim_t 去重，同 t 行必然是 0 —— 0 不等于"没瞬移"）')
        for i in big:
            r = rows[i]
            nb = '同t' if i > 0 and rows[i - 1]['t'] == r['t'] else '--'
            print('    %s t=%8.2f (%7.2f,%7.2f) jump=%5.2f  cmd=(%+.2f,%+.2f) mv=%5.2f'
                  % (nb, r['t'], r['x'], r['y'], r['jump'], r['cmd_v'], r['cmd_w'], r['mv']))
        # 邻行 x,y 差分是"车确实瞬移过"的独立证据。它和大跳行数不相等的那些表，
        # 说明瞬移落在被丢弃的帧里 —— jump 列当时看着一片干净，事后读表就是"车没动"。
        adj = [i for i in range(1, len(rows))
               if math.hypot(rows[i]['x'] - rows[i - 1]['x'],
                             rows[i]['y'] - rows[i - 1]['y']) >= jump_lo]
        miss = [i for i in adj if rows[i]['jump'] < jump_lo]
        print('  邻行位移>=%g m 的有 %d 处，其中 jump 列没记上的 %d 处（就是被吞掉的帧）'
              % (jump_lo, len(adj), len(miss)))
        for i in miss:
            d = math.hypot(rows[i]['x'] - rows[i - 1]['x'], rows[i]['y'] - rows[i - 1]['y'])
            print('    漏 t=%8.2f (%7.2f,%7.2f) <- (%7.2f,%7.2f)  实际位移 %6.2f m，jump 列只写 %.2f'
                  % (rows[i]['t'], rows[i]['x'], rows[i]['y'],
                     rows[i - 1]['x'], rows[i - 1]['y'], d, rows[i]['jump']))
        # 位移守恒：把所有大跳行按连续段分组，比较段内 jump 累加与首末净位移
        groups = []
        for i in big:
            if groups and i - groups[-1][-1] <= 2:
                groups[-1].append(i)
            else:
                groups.append([i])
        for g in groups:
            a, b = rows[g[0] - 1] if g[0] else rows[g[0]], rows[g[-1]]
            net = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
            acc = sum(rows[i]['jump'] for i in g)
            print('  段 t=%.2f..%.2f：jump 累加 %.2f m，首末净位移 %.2f m（差 %.2f m）'
                  % (a['t'], b['t'], acc, net, abs(acc - net)))
        ts = [r['t'] for r in rows]
        dup = sum(1 for i in range(1, len(ts)) if ts[i] == ts[i - 1])
        print('  全表同 t 相邻行共 %d 对' % dup)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
