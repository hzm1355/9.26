#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""截 VcXsrv 根窗口一帧存到 D 盘，用来自查 gzclient 到底渲染出了什么。

为什么不用 rqt_image_view / 也不让用户反复截图：WSL 里只有 Pillow 的 ImageGrab 能直接
拿到 X 根窗口（2026-10-06 实测），而 rqt 在 WSLg 权限下会崩。
用法：xshot142.py [输出.png]
"""
import sys

from PIL import ImageGrab

OUT = sys.argv[1] if len(sys.argv) > 1 else '/mnt/d/desktop1/9.26.1/_live142/xroot.png'
im = ImageGrab.grab()
im.save(OUT)
print('已存 %s：%dx%d' % (OUT, im.size[0], im.size[1]))
