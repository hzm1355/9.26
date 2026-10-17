#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 pk138_idlecheck251 的落盘日志里判"这一路的驱动实例健康不健康"。

为什么不直接在 shell 里 grep：memory「证据命令别用猜的过滤式」——过滤式一失配就
静默产出空证据，空结果在 shell 里永远读成"没超标"。这里把三条数一起读，读不到
就当作不达标并点名是哪一条读不到，宁可拦下来放车。

阈值口径（car_6 健康实例基准：线性增益 0.66、ω 增益 0.62）：
  * 线性增益 >= 0.40 —— 低于这个数就是"给了油不走"，楔死形态之一；
  * ω 增益 >= 0.30 —— 原地转失效时它是 0.00（155 轮 car_2/car_3 实测）；
  * 静置窗口不得出现"轮子在转"或"被外力顶"，否则先处理静息态再谈算法。
退出码 0 才允许把 shuttle 节点放上去。
"""
import re
import sys

L_V = 0.40
L_W = 0.30


def main():
    path = sys.argv[1]
    try:
        with open(path, encoding='utf-8', errors='strict') as fh:
            txt = fh.read()
    except OSError as exc:
        print('  !! 读不到增益日志 %s：%s' % (path, exc))
        return 1
    got_v = re.search(r'实际增益 ([0-9.]+)', txt)
    got_w = re.search(r'ω 增益 ([0-9.]+)', txt)
    rc = 0
    if got_v is None:
        print('  !! 日志里没有"实际增益"那一行：pk138 没跑完或没落盘')
        rc = 1
    else:
        v = float(got_v.group(1))
        print('  线性增益 %.2f（阈值 %.2f，car_6 基准 0.66）' % (v, L_V))
        if v < L_V:
            print('     !! 不达标：驱动实例仍未恢复，别放 shuttle')
            rc = 1
    if got_w is None:
        print('  !! 日志里没有"ω 增益"那一行')
        rc = 1
    else:
        w = float(got_w.group(1))
        print('  ω 增益 %.2f（阈值 %.2f，car_6 基准 0.62）' % (w, L_W))
        if w < L_W:
            print('     !! 不达标：原地转仍失效')
            rc = 1
    if '静置不位移' in txt:
        print('  ok 静置无自发爬行')
    elif '轮子在转' in txt or '被外力顶' in txt:
        line = [ln.strip() for ln in txt.splitlines()
                if '轮子在转' in ln or '被外力顶' in ln]
        print('     !! 静置态异常：%s' % (line[0] if line else '?'))
        rc = 1
    else:
        print('     !! 日志里没有静置定性行，无法确认静息态')
        rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
