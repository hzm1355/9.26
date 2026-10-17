#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按蓝色包围盒裁剪并放大，用于肉眼确认车牌纹理与字符是否可读。"""
import sys

import cv2
import numpy as np

src, dst = sys.argv[1], sys.argv[2]
f = cv2.imread(src)
hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
m = cv2.inRange(hsv, (100, 60, 40), (130, 255, 255))
ys, xs = np.where(m > 0)
x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
pad = max(30, (x1 - x0))
x0, x1 = max(0, x0 - pad), min(f.shape[1], x1 + pad)
y0, y1 = max(0, y0 - pad), min(f.shape[0], y1 + pad)
crop = f[y0:y1, x0:x1]
scale = max(1, 600 // max(1, crop.shape[1]))
big = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(dst, big)
print('%s -> %s  bbox=%dx%d  放大=%dx' % (src.split('/')[-1], dst.split('/')[-1],
                                          x1 - x0, y1 - y0, scale))
