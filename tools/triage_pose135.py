#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""比较两次 /model_states 采样，按模型名报位移（triage135.sh 的第二段用）。

写成独立文件而不是 heredoc：内联那段在 WSL 的一次性命令里挂过，而且判据脚本读不懂
就等于没有。输入是 triage135.sh 落好的三个文件（name 一次、pose 两次）。
"""
import re
import sys


def load_poses(path):
    txt = open(path, encoding='utf-8', errors='replace').read()
    return [tuple(float(v) for v in m.groups()) for m in re.finditer(
        r'position:\s*\{\s*x:\s*([-\d.e+]+)[,\s]+y:\s*([-\d.e+]+)', txt)]


def main():
    names_txt = open('/tmp/_n135a.txt', encoding='utf-8', errors='replace').read()
    names = re.findall(r'^- *(\S+) *$', names_txt, re.M)
    if not names:
        names = re.findall(r'^(\S+)$', names_txt, re.M)
    a, b = load_poses('/tmp/_p135a.txt'), load_poses('/tmp/_p135b.txt')
    print('  解析到 name %d 个、pose %d/%d 组' % (len(names), len(a), len(b)))
    if not names or not a or not b:
        print('  !! 回包解析不出来，看 /tmp/_n135a.txt 与 /tmp/_p135[ab].txt 原文')
        return 1
    n = min(len(names), len(a), len(b))
    moving = total = 0
    for i in range(n):
        if not names[i].startswith('car_'):
            continue
        d = ((a[i][0] - b[i][0]) ** 2 + (a[i][1] - b[i][1]) ** 2) ** 0.5
        total += 1
        moving += d > 0.05
        print('  %-10s (%8.2f,%8.2f) → (%8.2f,%8.2f)  位移 %.2f m' % (
            names[i], a[i][0], a[i][1], b[i][0], b[i][1], d))
    print('  两段之间在动的车：%d / %d' % (moving, total))
    return 0


if __name__ == '__main__':
    sys.exit(main())
