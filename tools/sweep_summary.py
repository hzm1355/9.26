#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""汇总 speed_sweep.csv：按速度档给出抓拍数 / 逐字命中数 / 实测速度分布 / 牌宽分布。

判定高速档"识别成功率"必须先看抓拍数——高速时漏拍（触发区在两帧位姿之间被跑过去）
和识别失败是两件事，混成一列会让 0% 看起来像识别器坏了。
静止抓拍（实测速度 < 0.2 m/s，通常是车卡在路口）单独列出，不计入命中率分母。
"""
import csv
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records/speed_sweep.csv'
rows = list(csv.DictReader(open(path, newline='', encoding='utf-8-sig')))
tiers = defaultdict(list)
for r in rows:
    tiers[r['label']].append(r)


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


print('%-8s %6s %6s %8s %6s %6s %14s %10s' % (
    '档位', '抓拍', '命中', '命中率', '静止', '有效', '有效实测速度', '牌宽中位'))
for label in sorted(tiers, key=lambda s: num(s) if num(s) is not None else 999):
    rs = tiers[label]
    moving = [r for r in rs if (num(r['speed_ms']) or 0) >= 0.2]
    still = len(rs) - len(moving)
    hit = sum(1 for r in moving if r['matched'] == 'True')
    sp = [num(r['speed_ms']) for r in moving if num(r['speed_ms']) is not None]
    bw = sorted(num(r['bbox_w']) or 0 for r in moving)
    print('%-8s %6d %6d %7.0f%% %6d %6d %8.2f~%-5.2f %10s' % (
        label + ' m/s', len(rs), sum(1 for r in rs if r['matched'] == 'True'),
        100.0 * hit / len(moving) if moving else 0, still, len(moving),
        min(sp) if sp else 0, max(sp) if sp else 0,
        int(bw[len(bw) // 2]) if bw else '-'))
print()
for label in sorted(tiers, key=lambda s: num(s) if num(s) is not None else 999):
    rs = tiers[label]
    miss = [r for r in rs if r['matched'] != 'True']
    # 高速档的"没认出"分两种，成因完全不同：牌面根本没进框/糊到定位不了（bbox=0），
    # 和定位到了但字符读错。混成一个 0% 会被读成"识别器坏了"。
    nolocate = [r for r in miss if (num(r['bbox_w']) or 0) == 0]
    wrong = [r for r in miss if (num(r['bbox_w']) or 0) > 0]
    print('[%s] 抓拍 %d 张 = 命中 %d + 牌面未定位 %d + 读出错字 %d' % (
        label, len(rs), sum(1 for r in rs if r['matched'] == 'True'),
        len(nolocate), len(wrong)))
    for r in miss[:6]:
        print('   %-16s %-15s 车=%-6s 速=%-6s 识别=%-10s 真值=%-10s 牌宽=%sx%s 置信=%s' % (
            r['capture_id'], r['gate_id'], r['vehicle_id'], r['speed_ms'],
            r['plate'], r['truth'], r['bbox_w'], r['bbox_h'], r['confidence']))
