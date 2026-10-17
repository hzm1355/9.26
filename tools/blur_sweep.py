#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫参：给 classify 的低通核选一个形状，判据不只是"认对"，还要看**领先第二名多少**。

只挑正确率的话，28/28 的候选里可能藏着一个每格都只赢 0.005 的配置——实机
gate_j2_south-20 就是这么把"6"读成"S"的。所以这里同时统计最小领先量。
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'v2x_demo', 'v2x_demo'))

import plate_recognizer_node as PR  # noqa: E402
from recognizer_selftest import synth, BODY_BGR, GREEN_BODY_BGR  # noqa: E402

CASES = ['沪A12345', '京B88990', '粤Z1A2B3', '苏E66F88']
SHOTS = [
    ('正对', {}),
    ('斜视15', {'yaw_deg': 15.0}),
    ('横滚4', {'roll_deg': 4.0}),
    ('逆光', {'gain': 1.6}),
    ('弱光', {'gain': 0.55}),
    ('绿车身', {'body': GREEN_BODY_BGR}),
    ('绿车身斜视', {'body': GREEN_BODY_BGR, 'yaw_deg': 12.0}),
]
REAL = ['/tmp/v2x_gates/gate_j2_south-20.png', '/tmp/v2x_gates/gate_j2_south-23.png',
        '/tmp/v2x_gates/gate_j3_north-21.png', '/tmp/v2x_gates/gate_j2_south-12.png']


def cell_margins(crop, tpls, k):
    src = cv2.GaussianBlur(crop, k, 0).astype(np.float32) if k[0] > 1 else crop.astype(np.float32)
    r = sorted((float(cv2.matchTemplate(src, t, cv2.TM_CCOEFF_NORMED).max()), ch)
               for ch, t in tpls)
    return r[-1][1], r[-1][0], r[-1][0] - r[-2][0]


def evaluate(ksize, tpls):
    """返回 (合成集正确数, 合成集总数, 平均领先量, 最小领先量)"""
    PR.BLUR_KSIZE = ksize
    ok = tot = 0
    margins = []
    for num in CASES:
        for label, kw in SHOTS:
            img = synth(num, '/tmp/v2x_sweep.png', **kw)
            quad, _ = PR.find_plate(img)
            if quad is None:
                tot += 1
                continue
            cells = PR.split_cells(cv2.cvtColor(PR.rectify(img, quad), cv2.COLOR_BGR2GRAY))
            text = ''
            for (crop, _), pool in zip(cells, PR.SLOT_POOLS):
                ch, _s, m = cell_margins(crop, tpls[ksize][pool], ksize)
                text += ch
                margins.append(m)
            tot += 1
            ok += (text == num)
    return ok, tot, margins


def real_margins(tpls, ksize):
    PR.BLUR_KSIZE = ksize
    out = []
    for p in REAL:
        img = cv2.imread(p)
        if img is None:
            continue
        quad, _ = PR.find_plate(img)
        if quad is None:
            out.append((os.path.basename(p), '-', 0.0, 0.0))
            continue
        cells = PR.split_cells(cv2.cvtColor(PR.rectify(img, quad), cv2.COLOR_BGR2GRAY))
        text, ms = '', []
        for (crop, _), pool in zip(cells, PR.SLOT_POOLS):
            ch, _s, m = cell_margins(crop, tpls[ksize][pool], ksize)
            text += ch
            ms.append(m)
        out.append((os.path.basename(p), text, float(np.mean(ms)), min(ms)))
    return out


def main():
    ksizes = [(1, 1), (3, 3), (5, 5), (5, 9), (7, 7), (7, 11), (9, 9), (9, 15), (11, 11)]
    tpls = {}
    for k in ksizes:
        PR.BLUR_KSIZE = k
        tpls[k] = PR.build_templates()

    print('%-9s %-8s %-9s %-9s  实机帧（号 / 均值 / 最小领先）' % (
        '核', '合成正确', '平均领先', '最小领先'))
    for k in ksizes:
        ok, tot, ms = evaluate(k, tpls)
        rm = real_margins(tpls, k)
        print('%-9s %-8s %-9.3f %-9.3f  %s' % (
            '%dx%d' % k, '%d/%d' % (ok, tot),
            float(np.mean(ms)) if ms else 0.0,
            float(np.min(ms)) if ms else 0.0,
            '  '.join('%s:%s %.3f/%.3f' % (n.split('.')[0].replace('gate_', ''),
                                           t, a, m) for n, t, a, m in rm)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
