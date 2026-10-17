#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#105 车队侧的 ω 判据：/tmp/signal_drive_path.csv 里车到底有没有"边开边转"。

#104 在 car_6 上测到 angular.z 只贡献 1% 的指令转角（且变号仍前冲）。而 car_1~car_5 的
diff_drive 插件块与模型本体和 car_6 逐字相同 —— 若它们真能转 90° 弯，就说明 ω 通道不是
全局坏的，问题只在 car_6；若日志里根本找不到"行驶中 yaw 连续变化 ≈1.5 rad"的窗口，
那就支持"车队是靠切角/被顶偏在走，不是在转向"。

判据：按车分组，用相邻行差分求 |d(yaw)/dt|，统计每车
  - yaw 总变化幅度（去卷绕后的累积 |Δ|，以及首末净变化）
  - 单拍最大 yaw 速率
  - "连续转够 1.0 rad 且期间平均线速 ≥0.15 m/s"的最短窗口时长（真转弯的样子）
"""

import csv
import math
import os
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/signal_drive_path.csv'
TURN_NEED = 1.0      # rad，够这么大才算一次转弯
MIN_SPEED = 0.15     # m/s，转弯得是在开着的时候


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    if not os.path.exists(PATH):
        print('!! 没有 %s（signal_drive 这轮还没写盘或已重启）' % PATH)
        return 2
    print('文件 %s  大小 %d 字节' % (PATH, os.path.getsize(PATH)))
    rows = defaultdict(list)
    with open(PATH, newline='', encoding='utf-8-sig') as fh:
        for r in csv.DictReader(fh):
            try:
                t = float(r['t'])
                y = float(r['yaw'])
                sp = float(r['speed'])
            except (TypeError, ValueError, KeyError):
                continue
            rows[r['car']].append((t, y, sp))
    if not rows:
        print('!! CSV 里没有任何车的数据行')
        return 2
    print('车数 %d，行数 %d' % (len(rows), sum(len(v) for v in rows.values())))
    for car in sorted(rows):
        v = rows[car]
        if len(v) < 5:
            print('  %-8s 行数 %d，太少' % (car, len(v)))
            continue
        acc = 0.0
        max_rate = 0.0
        best = {}          # sign -> (时长, 转角, 均速)
        del v[:-60000]     # 取最近 6 万拍：左右转弯都要样本，且避免 O(n^2)
        unw = [v[0][1]]
        for k in range(1, len(v)):
            unw.append(unw[-1] + wrap(v[k][1] - v[k - 1][1]))
        for sign in (+1.0, -1.0):
            j = 0
            i = 0
            while i < len(v) - 1:
                while j < len(v) - 1 and sign * (unw[j] - unw[i]) < TURN_NEED:
                    j += 1
                if sign * (unw[j] - unw[i]) >= TURN_NEED:
                    dt = v[j][0] - v[i][0]
                    mean_sp = sum(s[2] for s in v[i:j + 1]) / (j - i + 1)
                    if dt > 0 and mean_sp >= MIN_SPEED:
                        cur = best.get(sign)
                        if cur is None or dt < cur[0]:
                            best[sign] = (dt, sign * (unw[j] - unw[i]), mean_sp)
                    i = j + 1      # 已达标，从窗口尾继续找下一段
                    j = max(j, i)
                else:
                    i += 1
        for k in range(1, len(v)):
            dt = v[k][0] - v[k - 1][0]
            if dt > 1e-6:
                rate = abs(wrap(v[k][1] - v[k - 1][1])) / dt
                acc += rate * dt
                max_rate = max(max_rate, rate)
        net = abs(wrap(v[-1][1] - v[0][1]))
        print('  %-8s 行=%5d 时长=%6.1fs 累积|Δyaw|=%7.2f rad 单拍最大=%5.2f rad/s '
              '净变化=%5.2f rad' % (
                  car, len(v), v[-1][0] - v[0][0], acc, max_rate, net))
        for sign, name in ((+1.0, '左转'), (-1.0, '右转')):
            b = best.get(sign)
            if b:
                print('      %s：达标 %.1f s 转 %.2f rad，均速 %.2f m/s，等效 ω=%+.2f rad/s' % (
                    name, b[0], b[1], b[2], b[1] / b[0] * sign))
            else:
                print('      %s：没有"边开边转够 %.1f rad 且均速≥%.2f"的窗口' % (
                    name, TURN_NEED, MIN_SPEED))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
