#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""列出视频每 5 s 的主视区亮度，用来定位"哪一段没收到相机帧"。
用法：python3 vid_dark.py <mp4> [步长秒=5]
"""
import sys

import cv2

p = sys.argv[1]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
cap = cv2.VideoCapture(p)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
row = []
t = 0.0
while t < n / fps:
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
    ok, fr = cap.read()
    row.append('%3.0f:%3.0f' % (t, fr[40:h - 70, 0:w - 460].mean() if ok else -1))
    t += step
cap.release()
for i in range(0, len(row), 8):
    print('  ' + '  '.join(row[i:i + 8]))
