#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把识别器的每一级中间图拼成一张 montage 存盘，用于肉眼定位是哪一步坏的。

用法: dbg_recognizer.py <输出 png> [车牌号] [投影参数 kw..]
画的是：合成帧 -> 校正图(叠字符格) -> 七个格各自的最优匹配字模与得分。

sys.path 由本文件自己加，因为 recognizer_selftest 依赖 plate_layout 可导入。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))
import cv2
import numpy as np

from recognizer_selftest import synth
from plate_layout import CELL_H_MM, CELL_W_MM, CELL_X0, CELL_Y0, SLOT_POOLS
from plate_recognizer_node import (CAN_H, CAN_W, MARGIN, build_templates,
                                   classify, find_plate, rectify, split_cells)

out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dbg_recognizer.png'
number = sys.argv[2] if len(sys.argv) > 2 else '沪A12345'

img = synth(number, '/tmp/dbg_frame.png')
corners, area = find_plate(img)
print('定位面积 %.0f 角点:' % area, corners.tolist() if corners is not None else None)

plate = rectify(img, corners)
grid = plate.copy()
for i in range(7):
    x0, y0 = CELL_X0[i] - MARGIN, CELL_Y0 - MARGIN
    x1, y1 = CELL_X0[i] + CELL_W_MM + MARGIN, CELL_Y0 + CELL_H_MM + MARGIN
    cv2.rectangle(grid, (x0, y0), (x1, y1), (0, 255, 0), 1)

tpl = build_templates()
cells = split_cells(cv2.cvtColor(plate, cv2.COLOR_BGR2GRAY))
print('格子数 %d 尺寸' % len(cells), [c[0].shape for c in cells])

tiles = [cv2.resize(img, (960, 540)), cv2.resize(grid, (CAN_W * 2, CAN_H * 2))]
for (crop, _), pool in zip(cells, SLOT_POOLS):
    ch, s = classify(crop, tpl, pool)
    t = np.zeros((CELL_H_MM + 2 * MARGIN + 14, CELL_W_MM + 2 * MARGIN, 3), np.uint8)
    t[:crop.shape[0], :crop.shape[1]] = np.dstack([crop] * 3)
    cv2.putText(t, '%s %.2f' % (ch, s), (1, t.shape[0] - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
    tiles.append(t)

hmax = max(t.shape[0] for t in tiles)
row = np.hstack([np.pad(t, ((0, hmax - t.shape[0]), (0, 0), (0, 0)), constant_values=30)
                 for t in tiles])
cv2.imwrite(out, row)
print('montage ->', out)
