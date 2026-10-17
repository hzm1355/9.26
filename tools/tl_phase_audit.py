#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从一份录制文件里量出灯色时序的合规性（任务二 条目1 的量化判据）。

条目1 要的是"灯色切换具备真实时序特性"，光看时间轴肉眼数不行，得给出可判定的三条：
  ① 机动车相序恒为 GREEN -> YELLOW -> RED -> GREEN，非法跳变必须为 0；
  ② 黄灯时长稳定（各段时长离散度小），不能出现"黄灯被吞掉"的 0 秒段；
  ③ 行人灯是独立子相位机 WALK -> FLASHING -> DONT_WALK，且 **WALK 期间机动车不得为 GREEN**
     （人车放行必须错开，这条最容易在改配时的过程中被破坏）。
录制文件里同一时刻会混着 state / pedestrian / state_detail 三路事件，占位空串很多，
所以取相变一律先按"非空且与上一值不同"过滤。

用法：python3 tl_phase_audit.py <录制json> [信号组...]
"""
import json
import statistics
import sys

ILLEGAL = {('GREEN', 'RED'), ('RED', 'YELLOW'), ('YELLOW', 'GREEN')}
PED_ILLEGAL = {('WALK', 'DONT_WALK'), ('DONT_WALK', 'FLASHING'), ('FLASHING', 'WALK')}
LEGAL_PED = ('WALK', 'FLASHING', 'DONT_WALK')


def edges(events, group, key):
    """该组某一路字段的相变序列：[(t, 值)]，忽略空串与重复值"""
    out, last = [], None
    for e in events:
        if e.get('type') == 'vehicle' or e.get('group') != group:
            continue
        v = e.get(key, '')
        if not v or v == last:
            continue
        last = v
        out.append((e['t'], v))
    return out


def durs(seq, end):
    return [(seq[i + 1][0] - seq[i][0], seq[i][1]) for i in range(len(seq) - 1)]


def main():
    path = sys.argv[1]
    d = json.load(open(path, encoding='utf-8'))
    ev = d['events']
    span = max(e['t'] for e in ev)
    groups = sys.argv[2:] or d['meta'].get('groups') or []
    print('文件 %s：录制 %.1f s，事件 %d，信号组 %s' % (
        path, span, len(ev), ','.join(groups)))
    print('%-7s %-6s %-6s %-6s %-24s %-6s %-24s' % (
        '组', '车相变', '非法', '行人变', '黄灯时长(s)', '人非法', 'WALK/FLASH 时长(s)'))
    tot_v = tot_bad = tot_p = tot_pbad = 0
    conflict = 0
    for g in groups:
        v = edges(ev, g, 'vehicle_phase')
        p = edges(ev, g, 'pedestrian_phase')
        vd = durs(v, span)
        pd = durs(p, span)
        bad = [(a, b) for (t, a), (t2, b) in zip(v[:-1], v[1:]) if (a, b) in ILLEGAL]
        pbad = [(a, b) for (t, a), (t2, b) in zip(p[:-1], p[1:]) if (a, b) in PED_ILLEGAL]
        yel = [dt for dt, ph in vd if ph == 'YELLOW']
        walk = ['%s%.1f' % (ph[0], dt) for dt, ph in pd if ph in LEGAL_PED][:6]
        # 人车冲突：取每个 WALK 段起点时刻，看该时刻机动车相位是不是 GREEN
        for t, ph in p:
            if ph != 'WALK':
                continue
            cur = [x for x in v if x[0] <= t]
            if cur and cur[-1][1] == 'GREEN':
                conflict += 1
        tot_v += len(vd); tot_bad += len(bad); tot_p += len(pd); tot_pbad += len(pbad)
        print('%-7s %-6d %-6d %-6d %-24s %-6d %-24s' % (
            g, len(vd), len(bad), len(pd),
            ('min %.2f / 中位 %.2f / max %.2f' % (min(yel), statistics.median(yel), max(yel)))
            if yel else '（窗口内无完整黄灯段）',
            len(pbad), ' '.join(walk)))
    print('--- 合计：机动车相变 %d 次，非法 %d 次；行人相变 %d 次，非法 %d 次；'
          'WALK 起点与机动车 GREEN 冲突 %d 次 ---' % (tot_v, tot_bad, tot_p, tot_pbad, conflict))
    return 0 if (tot_bad == 0 and tot_pbad == 0 and conflict == 0) else 1


if __name__ == '__main__':
    sys.exit(main())
