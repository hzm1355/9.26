#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从视频里抽若干帧存成 JPG，用来肉眼核对录制版面。
用法：python3 vid_frame.py <视频> <输出前缀> [秒...]
"""
import sys

import cv2

path, pref = sys.argv[1], sys.argv[2]
secs = [float(x) for x in sys.argv[3:]] or [1.0]
cap = cv2.VideoCapture(path)
fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print('fps=%.2f 帧数=%d 时长=%.1fs 尺寸=%dx%d' % (
    fps, n, n / fps, cap.get(3), cap.get(4)))
for s in secs:
    cap.set(cv2.CAP_PROP_POS_FRAMES, min(n - 1, int(s * fps)))
    ok, fr = cap.read()
    if ok:
        out = '%s_%05.1f.jpg' % (pref, s)
        cv2.imwrite(out, fr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        print('wrote', out)
cap.release()
