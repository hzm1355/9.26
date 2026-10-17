#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抽几帧看：把取证视频右栏字幕带放大存成 PNG，核对"下发命令 + 回包数值"真写进了画面。

vid_check.py 只量亮度（查暗帧），字幕内容对不对得看图。用法：
    python3 vid_frames.py <视频> <秒[,秒...]> <输出目录>
"""
import os
import sys

import cv2


def main():
    path = sys.argv[1]
    secs = [int(x) for x in sys.argv[2].split(',') if x.strip()]
    outdir = sys.argv[3]
    os.makedirs(outdir, exist_ok=True)
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 10.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    panel = int(W * 0.32)          # 右栏实时数据（vid_rec.py 里 PANEL=450 / W=1440）
    band = int(H * 0.26)           # 主视区底部字幕带
    base = os.path.splitext(os.path.basename(path))[0]
    for s in secs:
        idx = min(int(s * fps), n - 1)
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = cap.read()
        if not ok:
            print('%3ds  取帧失败' % s)
            continue
        for tag, crop, sc in (('字幕', fr[H - band:, :W - panel], 1.4),
                              ('面板', fr[:, W - panel:], 1.7)):
            crop = cv2.resize(crop, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
            p = os.path.join(outdir, '%s_%03ds_%s.png' % (base, s, tag))
            cv2.imwrite(p, crop)
            print('%3ds  %s -> %s' % (s, tag, p))
    cap.release()
    return 0


if __name__ == '__main__':
    sys.exit(main())
