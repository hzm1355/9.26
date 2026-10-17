#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对一张抓拍 PNG 做离线识别体检：把 find_plate 的所有候选、胜出的那一块、
以及 rectify 之后的牌面图和逐格分数都打出来/存出来。

为什么要这么一步：停车场闸机抓拍 gate_out-13 人眼看得清清楚楚是"沪A12345"，
识别器却给 0.30（阈值 0.35）判未识别。这种"图是对的、分是低的"只有两种可能：
① 检测框选错了目标；② 选对了但矫正后的图坏了。这两种的修法完全不同
（①改选优规则或取景，②改矫正/距离），只看最终分数分不出来。

这份脚本里怀疑的是①：画面里有**两块**牌 —— 下方车尾牌正对镜头，上方那块是车头牌
从车顶上方露出来的一条，左右镜像。find_plate 的选优是"面积最大者胜"，而镜像那块
离相机更近、投出来更宽。它一旦胜出，送去 OCR 的就是反字，均分正好落在 0.3 附近。
判据：把胜出框矫正出来的图存成 PNG 用眼睛看 —— 反字就是①，正字但糊掉就是②。

用法（WSL，要 source ROS 环境，因为这个模块 import rclpy + v2x_msgs）：
  python3 diag_capture139.py <抓拍.png> <输出前缀>
"""
import os
import sys

# 绝对路径而不是 __file__ 相对：这份脚本会被 tr 去 \r 后复制到 /tmp 再跑（D 盘的
# .sh/.py 带 CRLF，WSL 里直接跑会报行尾错误），那时 __file__ 已经不在仓库树里了。
sys.path.insert(0, '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo')

import cv2
import numpy as np

import plate_recognizer_node as pr


def main():
    png, out = sys.argv[1], sys.argv[2]
    # 第三个参数临时放宽形状门槛上界，用来做"同一张图、两种宽严"的对照。
    # 默认 5.5 = 识别器现在的值（卡口龙门那套安装几何下标出来的）。
    if len(sys.argv) > 3:
        pr.PLATE_RATIO_MAX = float(sys.argv[3])
        print('本次形状门槛上界 = %.2f（识别器默认 5.5）' % pr.PLATE_RATIO_MAX)
    img = cv2.imread(png)
    if img is None:
        print('读不到 %s' % png)
        return 2
    print('图 %dx%d' % (img.shape[1], img.shape[0]))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (100, 60, 40), (130, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    n_ok = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < 400:
            continue
        rect = cv2.minAreaRect(c)
        (w, h) = rect[1]
        if w <= 0 or h <= 0:
            continue
        ratio = max(w, h) / min(w, h)
        fill = area / (w * h)
        ink, sep = pr._ink_fraction(img, rect)
        ok = (pr.PLATE_RATIO_MIN <= ratio <= pr.PLATE_RATIO_MAX) and fill >= 0.55 \
            and 0.12 <= ink <= 0.65 and sep >= 1.3
        cx, cy = rect[0]
        if ok:
            n_ok += 1
            print('  候选 area=%-8.0f 中心=(%4.0f,%4.0f) 尺寸=%4.0fx%-4.0f '
                  '长宽比=%.2f 填充=%.2f 墨占比=%.2f 分离度=%.2f  合格' % (
                      area, cx, cy, max(w, h), min(w, h), ratio, fill, ink, sep))
        else:
            print('  候选 area=%-8.0f 中心=(%4.0f,%4.0f) 尺寸=%4.0fx%-4.0f '
                  '长宽比=%.2f 填充=%.2f 墨占比=%.2f 分离度=%.2f  不合格' % (
                      area, cx, cy, max(w, h), min(w, h), ratio, fill, ink, sep))
    print('合格候选数=%d' % n_ok)

    quad, area = pr.find_plate(img)
    if quad is None:
        print('find_plate: 没定位到车牌')
        return 1
    print('胜出框 area=%.0f 角点=%s' % (area, np.round(quad).astype(int).tolist()))
    plate = pr.rectify(img, quad)
    cv2.imwrite(out + '_crop.png', plate)
    templates = pr.build_templates()
    cells = pr.split_cells(cv2.cvtColor(plate, cv2.COLOR_BGR2GRAY))
    text, scores = pr.assemble(cells, templates)
    print('矫正后牌面 → %s（%dx%d）' % (out + '_crop.png', plate.shape[1], plate.shape[0]))
    print('逐格：' + '  '.join('%s=%.3f' % (ch, s) for ch, s in zip(text, scores)))
    print('拼出=%r 均分=%.3f（识别器阈值 %.2f）' % (
        text, float(np.mean(scores)) if scores else -1.0, pr.__dict__.get('MIN_CONF', 0.35)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
