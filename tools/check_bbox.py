#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线回归：把所有历史抓拍图过一遍 find_plate，报出外接框越界的张。

存在的理由：bbox_* 在 PlateResult 里是 uint16，牌贴到画面边缘时 minAreaRect 的角点
会落在图像外（x 或 y 为负），识别器赋值时直接 AssertionError 崩掉整个节点——
而它一死，长跑测试只会静默收 0 行。这个脚本用来回答"这类图有几张、长什么样"，
并确认钳位之后不再有任何一张能把消息构造搞崩。

用法：python3 tools/check_bbox.py /tmp/v2x_gates
"""
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo'))
# plate_recognizer_node 里是 `from plate_layout import ...` 这种平铺导入
# （作为 ROS 节点跑时包目录本身就在 sys.path 上），离线用要把包目录也加进来
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))
from v2x_demo.plate_recognizer_node import find_plate  # noqa: E402


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_gates'
    files = sorted(glob.glob(os.path.join(d, '*.png')))
    neg = found = 0
    for f in files:
        img = cv2.imread(f)
        corners, _ = find_plate(img)
        if corners is None:
            continue
        found += 1
        x, y, w, h = cv2.boundingRect(corners.astype(np.int32))
        ih, iw = img.shape[:2]
        if x < 0 or y < 0 or x + w > iw or y + h > ih:
            neg += 1
            print('越界 %s  x=%d y=%d w=%d h=%d  图 %dx%d' % (
                os.path.basename(f), x, y, w, h, iw, ih))
    print('共 %d 张图，定位到牌 %d 张，其中外接框越界 %d 张' % (len(files), found, neg))
    return 0


if __name__ == '__main__':
    sys.exit(main())
