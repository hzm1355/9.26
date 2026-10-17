#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读 cmd_pose_trace.py 的对照表，回答一个问题：car_3 的横偏是控制器自己打出来的，
还是物理侧（碰撞/ODE）把车推着走。

两条判据，都按"连续段"看而不是单格 —— 差速模型在转角的加减速瞬间本来就会冲过
指令上限一两格，那不是证据：

  · 控制器自激：|cmd| 在 ±上限之间来回翻（振荡），实测 ≤ 上限。
    看 osc 段数与 cmd 的符号翻转次数。
  · 物理被推：实测越界成段（≥ min_run 格）而 |cmd| 已经顶满甚至为零 ——
    指令给不出这么多能量，只能是碰撞在灌。

用法：
  python3 tools/cmd_pose_report.py /mnt/d/desktop1/9.26.1/_cmd_pose_car3.csv
"""
import argparse
import csv


def load(path):
    with open(path, encoding='utf-8-sig', newline='') as fp:
        rows = list(csv.DictReader(fp))
    # 记录器每 200 格整体重写文件，读的时候可能正好切在半个行上；丢字段就跳行。
    good = [r for r in rows if all(r.get(k) not in (None, '') for k in
                                   ('t', 'x', 'y', 'cmd_v', 'cmd_w', 'mv', 'mw', 'jump'))]
    if len(good) != len(rows):
        print('跳过 %d 个残缺行' % (len(rows) - len(good)))
    return [{k: float(v) for k, v in r.items()} for r in good]


def runs(idxs):
    """把格号折成连续段（允许断 1 格仍算同一段，因为采样偶发丢一帧）。"""
    out = []
    for i in idxs:
        if out and i - out[-1][-1] <= 2:
            out[-1].append(i)
        else:
            out.append([i])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_file')
    ap.add_argument('--vceil', type=float, default=0.70, help='yaml 里的 cruise_speed')
    ap.add_argument('--wceil', type=float, default=0.45, help='max_angular')
    ap.add_argument('--vlimit', type=float, default=0.80, help='线速越界口径（>上限 14%%）')
    ap.add_argument('--wlimit', type=float, default=0.55, help='角速越界口径（>上限 22%%）')
    ap.add_argument('--min_run', type=int, default=4, help='连续几格以上才算一段')
    a = ap.parse_args()
    rows = load(a.csv_file)
    if len(rows) < 20:
        print('样本 %d 格太少，别下结论' % len(rows))
        return 1
    span = rows[-1]['t'] - rows[0]['t']
    print('样本 %d 格，t %.1f→%.1f s（%.0f s，%.1f Hz）'
          % (len(rows), rows[0]['t'], rows[-1]['t'], span, len(rows) / span))

    verdict = []
    for key, ck, lim, ceil_, name in (('mv', 'cmd_v', a.vlimit, a.vceil, '线速'),
                                      ('mw', 'cmd_w', a.wlimit, a.wceil, '角速')):
        peak = max(rows, key=lambda r: abs(r[key]))
        over = [i for i, r in enumerate(rows) if abs(r[key]) > lim]
        print('\n%s：峰值 %.3f @t=%.1f（指令上限 %.2f）；越界(>%.2f) %d/%d 格'
              % (name, abs(peak[key]), peak['t'], ceil_, lim, len(over), len(rows)))
        gr = [g for g in runs(over) if len(g) >= a.min_run]
        if not gr:
            print('  成段越界（≥%d 格）：0 段 —— 零散越界只是转角加减速的瞬态' % a.min_run)
            continue
        print('  成段越界 %d 段：' % len(gr))
        for g in gr:
            seg = rows[g[0]:g[-1] + 1]
            cabs = [abs(r[ck]) for r in seg]
            cmax, cmean = max(cabs), sum(cabs) / len(cabs)
            dur = seg[-1]['t'] - seg[0]['t']
            kind = ('物理被推' if cmax < 0.5 * ceil_ else
                    '指令顶满仍超（碰撞灌能量）' if cmax >= 0.9 * ceil_ else
                    '指令中等（增益跟随）')
            print('    t %7.1f→%7.1f（%.1f s, %d 格）%s 峰值 %.2f｜同列 |cmd| 峰 %.2f 均 %.2f'
                  ' → %s @(%5.1f,%5.1f)'
                  % (seg[0]['t'], seg[-1]['t'], dur, len(seg), name,
                     max(abs(r[key]) for r in seg), cmax, cmean, kind,
                     seg[len(seg) // 2]['x'], seg[len(seg) // 2]['y']))
            if kind != '指令中等（增益跟随）':
                verdict.append((name, dur, kind))

    # 指令侧的振荡：cmd 符号翻转密度 + 幅值，这是"控制器自激"的直接读数
    for ck, ceil_, name in (('cmd_w', a.wceil, 'cmd_w'), ('cmd_v', a.vceil, 'cmd_v')):
        vals = [r[ck] for r in rows]
        flips = sum(1 for i in range(1, len(vals))
                    if vals[i] * vals[i - 1] < 0 and abs(vals[i]) > 0.1)
        sat = sum(1 for v in vals if abs(v) >= 0.98 * ceil_)
        print('\n%s：符号翻转 %d 次（%.2f 次/s），顶满格数 %d（%.1f%%）'
              % (name, flips, flips / span, sat, 100.0 * sat / len(vals)))

    idle = [i for i, r in enumerate(rows)
            if abs(r['cmd_v']) < 0.02 and abs(r['cmd_w']) < 0.02 and abs(r['mv']) > 0.15]
    gi = [g for g in runs(idle) if len(g) >= a.min_run]
    print('零指令仍在动（|cmd|<0.02 而 v>0.15）：%d 格，成段 %d 段' % (len(idle), len(gi)))
    for g in gi:
        seg = rows[g[0]:g[-1] + 1]
        print('    t %7.1f→%7.1f v 峰 %.2f @(%5.1f,%5.1f)'
              % (seg[0]['t'], seg[-1]['t'], max(abs(r['mv']) for r in seg),
                 seg[len(seg) // 2]['x'], seg[len(seg) // 2]['y']))

    jumps = [i for i, r in enumerate(rows) if abs(r['jump']) > 0.5]
    print('瞬移(单格位移>0.5 m) %d 格：%s' % (
        len(jumps), ', '.join('t=%.1f/%.2f' % (rows[i]['t'], rows[i]['jump'])
                             for i in jumps[:8])))

    print('\n结论：' + ('窗口内没出现成段越界，也没抓到发散蛇形 —— 这段不能定因，要加长窗口'
                       if not verdict and not gi else
                       '出现成段实测越界 %d 段，见上；配合 cmd 顶满情况判断' % len(verdict)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
