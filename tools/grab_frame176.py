#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从成片里抽某一秒的一帧并按矩形裁一块，存成 PNG 好肉眼比对机位/遮挡。
用法：grab_frame176.py <视频> <秒> <out.png> [x y w h k]
不给矩形就存整帧（放大 1 倍）。
"""
import sys

import cv2

path, sec, dst = sys.argv[1], float(sys.argv[2]), sys.argv[3]
cap = cv2.VideoCapture(path)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
cap.set(cv2.CAP_PROP_POS_FRAMES, int(sec * fps))
ok, fr = cap.read()
cap.release()
if not ok:
    print('读不到 %s 处的一帧' % sec)
    sys.exit(1)
if len(sys.argv) > 5:
    x, y, w, h = (int(v) for v in sys.argv[4:8])
    k = float(sys.argv[8]) if len(sys.argv) > 8 else 1.0
    fr = fr[y:y + h, x:x + w]
    if k != 1.0:
        fr = cv2.resize(fr, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
cv2.imwrite(dst, fr)
print('%s @%.0fs -> %s  %dx%d 均值 %.1f' % (
    path.split('/')[-1], sec, dst.split('/')[-1], fr.shape[1], fr.shape[0], fr.mean()))
