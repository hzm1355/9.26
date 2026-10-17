#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把高位相机原帧按给定矩形裁出来并放大 N 倍，落盘好直接肉眼判读。
用法：crop_view174.py <in.png> <out.png> <x> <y> <w> <h> [放大倍数，默认 2]
"""
import sys

import cv2

src, dst = sys.argv[1], sys.argv[2]
x, y, w, h = (int(v) for v in sys.argv[3:7])
k = float(sys.argv[7]) if len(sys.argv) > 7 else 2.0

im = cv2.imread(src)
if im is None:
    print('读不到 %s' % src)
    sys.exit(1)
x0 = max(0, min(x, im.shape[1] - 1))
y0 = max(0, min(y, im.shape[0] - 1))
crop = im[y0:min(y0 + h, im.shape[0]), x0:min(x0 + w, im.shape[1])]
big = cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(dst, big)
print('%s -> %s  裁 %dx%d@%d,%d  放大 %.1fx  输出 %dx%d  均值 %.1f' % (
    src.split('/')[-1], dst.split('/')[-1], crop.shape[1], crop.shape[0],
    x0, y0, k, big.shape[1], big.shape[0], big.mean()))
