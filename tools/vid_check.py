#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视频体检：逐段量画面主视区亮度（抓"黑屏/没收到帧"）+ 报时长与体积。
用法：python3 vid_check.py <mp4> [mp4...]
"""
import os
import sys

import cv2
import numpy as np

for p in sys.argv[1:]:
    cap = cv2.VideoCapture(p)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    step = max(1, int(n / 24))
    means = []
    for i in range(0, n, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, fr = cap.read()
        if ok:
            means.append((i / fps, float(fr[40:h - 70, 0:w - 460].mean())))
    cap.release()
    v = np.array([m for _, m in means]) if means else np.zeros(1)
    dark = [t for t, m in means if m < 20.0]
    print('%-56s %5.0fs %5d帧 %dx%d %5.1fMB 亮度 min%5.1f 中位%5.1f max%5.1f%s' % (
        os.path.basename(p), n / fps, n, w, h, os.path.getsize(p) / 1e6,
        v.min(), np.median(v), v.max(),
        ('  暗帧@' + ','.join('%.0f' % d for d in dark[:8]) + 's') if dark else '  无暗帧'))
