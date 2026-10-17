#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐轮廓探针：把 find_plate 每一道门的判据都打出来，定位"车牌为什么没被选中"。

find_plate 只回一个 bool 结论，蓝牌/车身连成一片这类问题光看结论查不出来，
必须看到每个候选的 area / ratio / fill / ink / sep。
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))

from recognizer_selftest import synth                      # noqa: E402
from plate_recognizer_node import _ink_fraction, find_plate   # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                   'dbg_find.png')


def main():
    if len(sys.argv) > 1:
        # 直接解剖一张真实抓拍图（离线合成图永远复刻不出实车的那点毛病）
        img = cv2.imread(sys.argv[1])
        print('实图 %s %dx%d' % (sys.argv[1], img.shape[1], img.shape[0]))
    else:
        img = synth('沪A12345', '/tmp/v2x_dbg_find.png')

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(hsv, (100, 60, 40), (130, 255, 255))
    op = cv2.morphologyEx(raw, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))

    for tag, mask in (('raw', raw), ('open', op)):
        cs, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        print('--- %s: %d 个轮廓' % (tag, len(cs)))
        for c in sorted(cs, key=cv2.contourArea, reverse=True)[:6]:
            area = cv2.contourArea(c)
            if area < 200:
                continue
            rect = cv2.minAreaRect(c)
            w, h = rect[1]
            ratio = max(w, h) / max(1e-6, min(w, h))
            ink, sep = _ink_fraction(img, rect)
            print('  area=%-8.0f rect=%4.0fx%-4.0f ratio=%4.2f fill=%.2f '
                  'ink=%.2f sep=%5.2f' % (area, w, h, ratio, area / max(1.0, w * h),
                                          ink, sep))

    # 蒙太奇：原图 / raw 掩膜 / open 掩膜，纵向排
    corners, area = find_plate(img)
    if corners is None:
        print('find_plate -> None（未定位到车牌）')
    else:
        wd = (np.linalg.norm(corners[1] - corners[0])
              + np.linalg.norm(corners[2] - corners[3])) / 2
        ht = (np.linalg.norm(corners[2] - corners[1])
              + np.linalg.norm(corners[3] - corners[0])) / 2
        print('find_plate -> %.0fx%.0f px 面积 %.0f' % (wd, ht, area))
    m = np.vstack([img,
                   cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR),
                   cv2.cvtColor(op, cv2.COLOR_GRAY2BGR)])
    crop = m[:, 500:1450]
    cv2.imwrite(OUT, crop)
    print('蒙太奇 -> %s' % os.path.abspath(OUT))


if __name__ == '__main__':
    main()
