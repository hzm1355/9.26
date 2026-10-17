#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通行记录离线重算器：只读 parking_pass.jsonl，把每个过滤维度的命中数算出来。

存在的唯一理由是"两份账当面碰"：`parking_cli.py query` 回的是记录器内存环形表口径，
而内存表在记录器重启时清零、超过 max_records_in_memory 还会挤掉最老的。JSONL 是追加式
全量上游。两者**本来就允许不等**，所以必须把两个口径同时摆在字幕里，让读者看到差多少、
差在哪（哪一段通行是重启之前的），而不是看到一句"命中 12 条"就当它是全部历史。

期望值基线 = 2026-10-16 在环真值（control.plates 里 car_6 已退役，在环的两台是
car_3=京B88990 摆渡车、car_2=粤Z1A2B3 临时车）。这里不写死任何计数断言，
只做重算与分布打印 —— 计数会随录制增长，写死就成了会过期的假判据。

用法：
  python3 pk_pass_census.py                        # 默认 /tmp/v2x_parking
  python3 pk_pass_census.py --dir-name /tmp/v2x_parking --show-reason
"""
import argparse
import json
import os
import sys
from collections import Counter

# ParkingPass 的枚举值（v2x_msgs/msg/ParkingPass.msg，改了要同步）
DIR_NAME = {0: '进场', 1: '出场'}
RESULT_NAME = {0: '放行', 1: '拒绝', 2: '挂起'}
KIND_NAME = {0: '月租', 1: '临时', 2: '未登记', 3: '黑名单'}


def as_int(v):
    """JSONL 里枚举既可能是数值也可能是字符串；读空/读坏一律 None，不参与匹配。"""
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return None


def load(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                rows.append(json.loads(ln))
            except ValueError:
                # 追加写被进程打断会在尾部留半行；半行不算一条记录，但必须报出来，
                # 不然重算值比 CLI 少 1 时没人知道是这条烂行造成的。
                print('  !! 该行不是完整 JSON，已跳过：%s' % ln[:80])
                continue
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir-name', default='/tmp/v2x_parking',
                    help='落盘目录（注意：变量名不能叫 --dir，argparse 会和内置抢）')
    ap.add_argument('--pass-file', default='parking_pass.jsonl')
    ap.add_argument('--show-reason', action='store_true',
                    help='把 result!=放行 的那几条的 reason 原文列出来')
    args = ap.parse_args()

    pj = os.path.join(args.dir_name, args.pass_file)
    rows = load(pj)
    print('=== 离线重算（数据源：%s，%d 行）===' % (pj, len(rows)))
    if not rows:
        # 零条是合法状态（刚轮转完），但下游过滤全为 0 时"对得上"没有意义，明确说清。
        print('  账本为空：本轮会话还没有一条通行落盘，过滤矩阵各维度应全为 0。')
        return 0

    # ── 编号的两套口径必须点名（读表人最容易在这里把"第 1 条"读成"史上第 1 条"）──
    # seq 是记录器进程内的流水，重启即从 1 重来；pass_id 是 manager 生成的全局回指键，
    # 跨重启从遗留 JSONL 尾部续号。两者不等长就直接证明了"当前账本只是历史的一段"。
    ids = [str(r.get('pass_id')) for r in rows if r.get('pass_id')]
    seqs = [as_int(r.get('seq')) for r in rows if as_int(r.get('seq')) is not None]
    print('  编号口径      seq %s..%s（进程内流水，重启归 1）｜pass_id %s..%s（全局键，跨重启续号）'
          % (seqs[0] if seqs else '?', seqs[-1] if seqs else '?',
             ids[0] if ids else '?', ids[-1] if ids else '?'))

    # ── 维度分布：先看数据长什么样，再定过滤值 ──
    for key, names in (('vehicle_id', None), ('plate_number', None),
                       ('gate_id', None), ('kind', KIND_NAME),
                       ('direction', DIR_NAME), ('result', RESULT_NAME),
                       ('whitelist_hit', {0: '未查', 1: '命中', 2: '未命中', 3: '黑名单'})):
        c = Counter()
        for r in rows:
            v = r.get(key)
            if names:
                v = names.get(as_int(v), '坏值(%s)' % v)
            c[str(v)] += 1
        print('  %-14s %s' % (key, '  '.join('%s×%d' % kv for kv in sorted(c.items()))))

    # ── 时序完整性：-1 表示这一段没发生，读表时要和"时刻为 0"区分开 ──
    for key in ('arrive_s', 'raise_s', 'clear_s', 'lower_s'):
        n = sum(1 for r in rows if as_num(r.get(key)) is not None and r.get(key) != -1)
        print('  %-14s 有真实时刻 %d / %d 条（其余为 -1）' % (key, n, len(rows)))

    # ── 过滤矩阵：与 vid_parking_5.sh 的 5c 逐项同序，便于当面碰 ──
    out = []
    for label, pred in [
        ('全量', lambda r: True),
        ('只查进场', lambda r: as_int(r.get('direction')) == 0),
        ('只查出场', lambda r: as_int(r.get('direction')) == 1),
        ('只查放行', lambda r: as_int(r.get('result')) == 0),
        ('只查拒绝', lambda r: as_int(r.get('result')) == 1),
        ('挂起（等人/让位）', lambda r: as_int(r.get('result')) == 2),
        ('车牌子串', lambda r: '京B' in str(r.get('plate_number') or '')),
        ('指定闸机 gate_in', lambda r: str(r.get('gate_id')) == 'gate_in'),
        ('指定闸机 gate_out', lambda r: str(r.get('gate_id')) == 'gate_out'),
        ('摆渡车 car_3', lambda r: str(r.get('vehicle_id')) == 'car_3'),
        ('临时车 car_2', lambda r: str(r.get('vehicle_id')) == 'car_2'),
        ('组合（进场+放行+京B）', lambda r: as_int(r.get('direction')) == 0
            and as_int(r.get('result')) == 0
            and '京B' in str(r.get('plate_number') or '')),
    ]:
        n = sum(1 for r in rows if pred(r))
        out.append((label, n))
    print('=== 与 5c 过滤矩阵同序的重算命中数（CLI 的 matched 必须逐个等于这里）===')
    for label, n in out:
        print('  %-22s %d' % (label, n))

    # 已退役的 car_6 只有在历史账本里才可能现身；它一出现就说明账本跨了会话，
    # 这时候"内存表 = JSONL"必然不成立，得先说清是重启前的遗留。
    legacy = [r for r in rows if str(r.get('vehicle_id')) not in ('car_3', 'car_2')]
    if legacy:
        print('  注意：有 %d 条不属于当前在环车辆（%s），是重启/轮转前的遗留账。' % (
            len(legacy), ' '.join(sorted(set(str(r.get('vehicle_id')) for r in legacy)))))

    if args.show_reason:
        bad = [r for r in rows if as_int(r.get('result')) != 0]
        print('=== 未放行条目原文（%d 条）===' % len(bad))
        for r in bad[-8:]:
            print('  %s %s %s result=%s reason=%s' % (
                r.get('pass_id'), r.get('gate_id'), r.get('plate_number'),
                RESULT_NAME.get(as_int(r.get('result')), '?'),
                str(r.get('reason'))[:70]))
    return 0


def as_num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


if __name__ == '__main__':
    sys.exit(main())
