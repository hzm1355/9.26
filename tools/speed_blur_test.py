#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把"通过速度"换算成像素运动模糊，离线量识别器的容忍上限。

为什么需要离线这一半：Gazebo Classic 的相机传感器不做快门积分——每帧都是该时刻
的瞬时几何渲染，所以实机跑 6 m/s 也只是抓拍位姿更"跳"，画面本身不会糊。实机数据
因此只能证明"触发/取景在高速下会坏"，证明不了"识别器怕糊"。真车的糊是
smear = v × 曝光时间，这一段只能离线补。

换算用实机量到的牌面像素宽度：北环/南环抓拍里 440 mm 车牌宽约 300 px，
即 1 m ≈ 680 px。于是 v=6 m/s、曝光 1/100 s 时 smear ≈ 0.06 m ≈ 41 px，
而 1/1000 s 只有 4 px——同一个速度差一个数量级，这正是电子警察必须短曝光的原因。

模糊方向取图像水平：车尾牌面在两个卡口的取景里都接近水平放置，车是横穿画面，
所以糊沿着牌字基线方向（最坏情况，比斜向糊更考验识别器）。

用法：python3 speed_blur_test.py [px_per_m]   默认 680
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'v2x_demo', 'v2x_demo'))

import plate_recognizer_node as PR  # noqa: E402
from recognizer_selftest import synth  # noqa: E402

CASES = ['沪A12345', '京B88990', '粤Z1A2B3', '苏E66F88', '浙A66666']
# 电子警察常用曝光档；1/100 是"没做短曝光"的下界对照
EXPOSURES = [(1.0 / 1000, '1/1000s'), (1.0 / 500, '1/500s'), (1.0 / 100, '1/100s')]
SPEEDS = [0.7, 2.0, 4.0, 6.0, 10.0, 15.0, 25.0]
# SPEEDS=5,6,7,8 EXPOSURES=1/500 python3 tools/speed_blur_test.py
# 用于在主表跑完后单独夹住"糊几个 px 就崩"的拐点，不必重跑整张表。
if os.environ.get('SPEEDS'):
    SPEEDS = [float(x) for x in os.environ['SPEEDS'].split(',')]
if os.environ.get('EXPOSURES'):
    # 档名写成 1/500 或 1/500s 都认（表里显示的是带 s 的）
    keep = set(os.environ['EXPOSURES'].replace('s', '').split(','))
    EXPOSURES = [e for e in EXPOSURES if e[1].replace('s', '') in keep]


def motion_blur(img, length_px):
    """沿水平的盒式运动模糊。length<=1 时原样返回。"""
    n = int(round(length_px))
    if n <= 1:
        return img
    k = np.ones((1, n), np.float32) / n
    return cv2.filter2D(img, -1, k, borderType=cv2.BORDER_REPLICATE)


def recognize(img, tpls):
    """跑线上那条识别链（find_plate -> rectify -> split_cells -> 逐格滑窗匹配）。

    不直接调 PR.assemble：它只返回平均置信，看不出"这一格是稳稳赢还是只赢
    0.005"。这里自己逐格取前三，顺手把领先量算出来——低通核必须和 classify
    里那次 GaussianBlur 一致，否则量到的领先量和线上不是同一个东西。
    """
    quad, _ = PR.find_plate(img)
    if quad is None:
        return None, 0.0, 0.0, '无牌面'
    cells = PR.split_cells(cv2.cvtColor(PR.rectify(img, quad), cv2.COLOR_BGR2GRAY))
    if len(cells) != 7:
        return None, 0.0, 0.0, '只切出 %d 格' % len(cells)
    text, scores, gaps = '', [], []
    for (crop, _), pool in zip(cells, PR.SLOT_POOLS):
        src = cv2.GaussianBlur(crop, PR.BLUR_KSIZE, 0).astype(np.float32)
        r = sorted((float(cv2.matchTemplate(src, t, cv2.TM_CCOEFF_NORMED).max()), ch)
                   for ch, t in tpls[pool])
        text += r[-1][1]
        scores.append(r[-1][0])
        gaps.append(r[-1][0] - r[-2][0])
    return text, float(np.mean(scores)), float(np.min(gaps)), ''


def main():
    px_per_m = float(sys.argv[1]) if len(sys.argv) > 1 else 680.0
    tpls = PR.build_templates()
    print('牌面像素换算 %.0f px/m（实机抓拍 440 mm 牌宽 ≈ %.0f px）；'
          '识别器低通核 %dx%d（线上默认）' % (px_per_m, 0.44 * px_per_m,
                                              PR.BLUR_KSIZE[0], PR.BLUR_KSIZE[1]))
    print('\n%-9s %-8s %-6s %-9s %-9s %-9s %s' % (
        '速度m/s', '曝光', '糊px', '命中', '平均置信', '最小领先', '失败原因'))
    for exp, elabel in EXPOSURES:
        for v in SPEEDS:
            smear = v * exp * px_per_m
            ok = 0
            confs, margins, why = [], [], []
            for num in CASES:
                img = motion_blur(synth(num, '/tmp/v2x_blur.png'), smear)
                text, s, m, reason = recognize(img, tpls)
                hit = (text == num)
                ok += hit
                if text:
                    confs.append(s)
                    margins.append(m)
                if not hit:
                    why.append(reason or ('读成 %s' % text))
            print('%-9.1f %-8s %-6.1f %-9s %-9.3f %-9s %s' % (
                v, elabel, smear, '%d/%d' % (ok, len(CASES)),
                float(np.mean(confs)) if confs else 0.0,
                '%.3f' % float(np.min(margins)) if margins else '-',
                '; '.join(why[:3])))
        print('')
    return 0


if __name__ == '__main__':
    sys.exit(main())
