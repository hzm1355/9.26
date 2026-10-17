#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""解剖一张真实抓拍图：打印定位四边形、导出校正画布，并列出每个字符格的前三名校验。

用法：python3 dbg_rectify.py <抓拍 PNG 路径>
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'v2x_demo', 'v2x_demo'))

from plate_layout import SLOT_POOLS  # noqa: E402
from plate_recognizer_node import (  # noqa: E402
    BLUR_KSIZE, CAN_H, CAN_W, build_templates, find_plate, rectify, split_cells)

WIN = os.path.join(HERE, '..', '..', 'gatechk')
if not os.path.isdir(WIN):
    WIN = '/tmp/v2x_dbg'


def ranking(crop, tpls):
    src = cv2.GaussianBlur(crop, BLUR_KSIZE, 0).astype(np.float32)
    return sorted(((float(cv2.matchTemplate(src, t, cv2.TM_CCOEFF_NORMED).max()), ch)
                   for ch, t in tpls), reverse=True)


def main():
    path = sys.argv[1]
    img = cv2.imread(path)
    if img is None:
        print('读图失败 %s' % path)
        return 1
    print('读入 %s -> %s' % (path, img.shape))

    quad, area = find_plate(img)
    if quad is None:
        print('定位失败：图中没有可用车牌候选')
        return 1
    print('定位四边形 面积 %.0f px' % area)
    for i, (x, y) in enumerate(quad):
        print('  P%d = (%.1f, %.1f)' % (i, x, y))
    e = [float(np.linalg.norm(quad[(i + 1) % 4] - quad[i])) for i in range(4)]
    print('  边长 P0P1 %.1f  P1P2 %.1f  P2P3 %.1f  P3P0 %.1f' % tuple(e))

    canvas = rectify(img, quad)
    gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
    cells = split_cells(gray)
    print('校正画布 %dx%d，切格 %d 枚' % (CAN_W, CAN_H, len(cells)))

    templates = build_templates()
    text, conf = '', []
    for (crop, cx), pool in zip(cells, SLOT_POOLS):
        top = ranking(crop, templates[pool])[:3]
        s, ch = top[0]
        text += ch
        conf.append(s)
        print('  格%d cx=%.0f 底均值 %.0f -> %s' % (
            len(text), cx, crop.mean(),
            ' '.join('%s %.3f' % (c, v) for v, c in top)))
    print('识别 = %s  置信均值 %.2f' % (text, float(np.mean(conf))))

    tag = os.path.splitext(os.path.basename(path))[0]
    big = cv2.resize(canvas, (CAN_W * 3, CAN_H * 3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(os.path.join(WIN, 'rect_%s.png' % tag), big)
    strip = np.full((cells[0][0].shape[0] + 8,
                     len(cells) * (cells[0][0].shape[1] + 6) + 6), 40, np.uint8)
    x = 6
    for crop, _ in cells:
        strip[4:4 + crop.shape[0], x:x + crop.shape[1]] = crop
        x += crop.shape[1] + 6
    cv2.imwrite(os.path.join(WIN, 'cells_%s.png' % tag), strip)
    cv2.imwrite(os.path.join(WIN, 'quad_%s.png' % tag),
                cv2.drawContours(img, [quad.reshape(-1, 1, 2).astype(np.int32)],
                                 -1, (0, 255, 0), 2))
    # 源图上的车牌原样（未经任何 warp），用来区分"字本来就是斜的"和"校正把它拧斜了"
    x0 = int(max(0, quad[:, 0].min() - 20)); x1 = int(min(img.shape[1], quad[:, 0].max() + 20))
    y0 = int(max(0, quad[:, 1].min() - 20)); y1 = int(min(img.shape[0], quad[:, 1].max() + 20))
    raw = img[y0:y1, x0:x1]
    cv2.imwrite(os.path.join(WIN, 'raw_%s.png' % tag),
                cv2.resize(raw, (raw.shape[1] * 4, raw.shape[0] * 4),
                           interpolation=cv2.INTER_NEAREST))
    print('已导出 rect_%s.png / cells_%s.png / quad_%s.png / raw_%s.png'
          % (tag, tag, tag, tag))
    return 0


if __name__ == '__main__':
    sys.exit(main())
