#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在抓拍帧里找蓝色车牌块，输出每帧的蓝色像素数与包围盒，便于挑出有效帧。"""
import glob
import sys

import cv2
import numpy as np

for p in sorted(glob.glob(sys.argv[1] + '/*.png')):
    f = cv2.imread(p)
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    # 蓝牌：色相约 100~130，饱和度不能太低（逆光/夜问要放宽，这里先看白天）
    m = cv2.inRange(hsv, (100, 60, 40), (130, 255, 255))
    n = int(cv2.countNonZero(m))
    box = ''
    if n > 40:
        ys, xs = np.where(m > 0)
        box = 'x=%d..%d y=%d..%d w=%d h=%d' % (
            xs.min(), xs.max(), ys.min(), ys.max(),
            xs.max() - xs.min(), ys.max() - ys.min())
    print('%s blue=%-6d %s' % (p.split('/')[-1], n, box))
