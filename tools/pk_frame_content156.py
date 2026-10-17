#!/usr/bin/env python3
"""离线量"停车场卡口抓拍帧到底是不是空帧"——只读 PNG，不碰 ROS、不改任何在跑的东西。

为什么要单独量这个：pk_show_capture.py 用"文件 >50 KB"当存在性判据，而停车场这两路
1920x1080 的实拍帧实测只有 43~48 KB，于是字幕上挂着"!! 太小，疑似空帧"。体积判据是从
高速卡口那两路（60~90 KB）搬来的，牌面占比小、背景是纯色地面时就压得动，所以它判的是
"压缩率"不是"有没有内容"。真正该量的是像素统计：空帧（整幅同色）标准差≈0，有车牌的帧
标准差显著非零，且外接框那一带的方差比四周大（字与底色对比）。

用法：python3 pk_frame_content156.py [张数]
"""
import os
import sys

from PIL import Image

DIRS = ['/tmp/v2x_parking_in', '/tmp/v2x_parking_out']
N = int(sys.argv[1]) if len(sys.argv) > 1 else 4


def stat(path):
    im = Image.open(path).convert('L')
    w, h = im.size
    small = im.resize((192, 108))
    px = list(small.getdata())
    n = len(px)
    mean = sum(px) / n
    var = sum((p - mean) ** 2 for p in px) / n
    std = var ** 0.5
    uniq = len(set(px))
    # 车牌在画面中部偏下：截一条横带量局部方差，空帧的带内方差也≈0
    band = [p for i, p in enumerate(im.getdata()) if h * 3 // 5 <= i // w <= h * 4 // 5]
    bm = sum(band) / len(band)
    bstd = (sum((p - bm) ** 2 for p in band) / len(band)) ** 0.5
    return w, h, mean, std, uniq, bstd


def main():
    bad = 0
    for d in DIRS:
        files = sorted([f for f in os.listdir(d) if f.endswith('.png')],
                       key=lambda f: os.path.getmtime(os.path.join(d, f)))
        print('== %s（共 %d 张，取最新 %d 张）==' % (d, len(files), min(N, len(files))))
        for f in files[-N:]:
            p = os.path.join(d, f)
            kb = os.path.getsize(p) / 1024.0
            try:
                w, h, mean, std, uniq, bstd = stat(p)
            except Exception as e:                      # 插件正在写这个文件时会读到半截
                print('  %-24s 读失败（可能正在被写入）：%s' % (f, e))
                continue
            verdict = '有内容' if std > 3.0 and uniq > 8 else '空帧'
            if verdict == '空帧':
                bad += 1
            print('  %-24s %4.0f KB  %dx%d  均值 %.1f  全图标准差 %.1f  '
                  '灰度级 %d  中下带标准差 %.1f  => %s'
                  % (f, kb, w, h, mean, std, uniq, bstd, verdict))
    print('判定：最新样本里空帧 %d 张' % bad)
    return 0


if __name__ == '__main__':
    sys.exit(main())
