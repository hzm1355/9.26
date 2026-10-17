#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自检参考识别器：不连 Gazebo，把车牌纹理按卡口成像几何合成一张"抓拍图"再识别。

为什么需要它：识别器要调的是"从 303x82 像素里认字"，这一步和 Gazebo 无关，
用真实渲染出的 tex.png 做针孔投影 + 倾斜，就能覆盖绝大多数失败模式，
比每轮都重启仿真、等车过卡口快两个数量级。

成像参数与 world 里 gate_j3_north 对齐：
  相机高 9.2、车牌中心 6.85 -> 落差 2.35 m；俯角 0.55 rad -> 水平 3.833 m；斜距 4.50 m
  horizontal_fov 0.6 @ 1920 宽 -> 焦距 f = 960/tan(0.3) = 3103 px
  入射角 31.5°（车牌法线水平、视线在竖直面内下倾），只压缩车牌**高度**：
    宽 0.440/4.50*3103 ≈ 303 px，高 0.140*cos(31.5)/4.50*3103 ≈ 82 px

合成帧一律带一块蓝色车尾面板：真实的卡口帧里车牌不是浮在灰底上，牌的外沿蓝
和车漆同色，只有 5 mm 白色边框线（落地约 3 px）把它们隔开。定位器的形态学
处理就是为这条缝设计的，不画面板就测不到它。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'v2x_demo', 'v2x_demo'))

import cv2
import numpy as np

from plate_layout import norm_number, render_plate
from plate_recognizer_node import (assemble, build_templates, find_plate,
                                   rectify, split_cells)

W_IMG, H_IMG = 1920, 1080
F_PX = 960.0 / np.tan(0.6 / 2.0)   # 3103 px，与卡口相机 fov/分辨率一致
Z0 = 4.50                          # 抓拍斜距
PLATE_M_W, PLATE_M_H = 0.44, 0.14  # 国标蓝牌物理尺寸
INCIDENCE = 31.5                   # 车牌法线与视线的夹角，等于相机俯角 0.55 rad
# 车尾面板：实测蓝漆 BGR(200,110,40) -> HSV(107,204,200)，稳稳落在定位器的蓝色窗口里。
# 车牌外沿那圈蓝和它是**连成一片**的，隔开两者的只有牌上 5 mm 的白色边框线。
BODY_BGR = (200, 110, 40)
# 绿车身（实机 car_2 采样：HSV(67,191,151)，色相在蓝色窗口之外，牌边那圈蓝漆
# 会被开运算整圈抹掉，候选层级和蓝车身完全不同）——必须有这一档，
# 否则"往内层收"的规则只在蓝车上被测到，绿车一上实机就把三辆车全打成空号。
GREEN_BODY_BGR = (63, 151, 38)
# 贴牌用的虚拟平面，比真车车尾面（约 0.8x0.4 m，投出来长宽比 1.85）略大一圈，
# 四周一留灰底，二让车身团块在长宽比上更贴近真实干扰项。
PLANE_M_W, PLANE_M_H = 1.00, 0.55


def _quad(tilt_deg, yaw_deg, roll_deg, w_m=PLATE_M_W, h_m=PLATE_M_H):
    """平面图形绕自身中心旋转后做针孔投影，返回左上起顺时针 4 个图像角点。

    局部系：x 沿面宽、y 沿面高（向下）、z 为法线。
    tilt 绕 x（宽度轴）压缩高度，对应相机俯角造成的入射角；
    yaw 绕 y（高度轴）压缩宽度，对应车辆不在车道正中时的斜视。
    """
    a, b, c = (np.radians(v) for v in (tilt_deg, yaw_deg, roll_deg))
    Rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    Rz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    R = Rx @ Ry @ Rz

    hw, hh = w_m / 2.0, h_m / 2.0
    pts = np.array([[-hw, -hh, 0.0], [hw, -hh, 0.0], [hw, hh, 0.0], [-hw, hh, 0.0]])
    pts = pts @ R.T
    pts[:, 2] += Z0
    u = F_PX * pts[:, 0] / pts[:, 2] + W_IMG / 2.0
    v = F_PX * pts[:, 1] / pts[:, 2] + H_IMG / 2.0
    return np.array(np.stack([u, v], 1), np.float32)


def _scene_plane(number, body):
    """把车牌纹理贴到（可选的）车身面板上，返回整块虚拟平面图及其物理尺寸。

    body 是车身 BGR 色，False 表示不画面板（牌直接浮在灰底上）。
    面板是**必须画**的回归项：定位器"车牌和车身连成一个轮廓"这个失败模式，
    在没有面板的灰色合成图上永远碰不到。它就是一路通过 20/20 自测、
    到实机上才暴露的，代价是一次完整的 gzserver 重启。
    """
    # 平面像素密度按"牌面在画面里仍是 2 倍面积降采样"定，与旧版 k 因子等价。
    px_per_m = 2.0 * 303.0 / PLATE_M_W
    wm, hm = (PLANE_M_W, PLANE_M_H) if body else (PLATE_M_W, PLATE_M_H)
    pw, ph = int(round(wm * px_per_m)), int(round(hm * px_per_m))
    canvas = np.full((ph, pw, 3), body if body else 128, np.uint8)

    plate = render_plate(number)
    ww, hh = int(round(PLATE_M_W * px_per_m)), int(round(PLATE_M_H * px_per_m))
    small = cv2.resize(plate, (ww, hh), interpolation=cv2.INTER_AREA)
    x0, y0 = (pw - ww) // 2, (ph - hh) // 2
    canvas[y0:y0 + hh, x0:x0 + ww] = small
    return canvas, wm, hm


def synth(number, out_png, tilt_deg=INCIDENCE, yaw_deg=0.0, roll_deg=0.0,
          gain=1.0, seed=0, body=BODY_BGR):
    """渲染车牌 + 车尾面板 -> 按卡口几何投影到 1920x1080。

    平面按**实际画面 2 倍**的密度绘制再 warpPerspective 下来。直接拿 1760x560
    的纹理 warp 到 303x82 是点采样，会引入 OGRE mipmap 三线性过滤下并不存在的
    走样；拿它调识别器会把参数往错方向拧。
    """
    plane, wm, hm = _scene_plane(number, body)
    ph, pw = plane.shape[:2]

    quad = _quad(tilt_deg, yaw_deg, roll_deg, wm, hm)
    m = cv2.getPerspectiveTransform(
        np.array([[0, 0], [pw, 0], [pw, ph], [0, ph]], np.float32), quad)
    warped = cv2.warpPerspective(plane, m, (W_IMG, H_IMG))
    cover = cv2.warpPerspective(np.full((ph, pw), 255, np.uint8), m, (W_IMG, H_IMG))

    rng = np.random.default_rng(seed)
    canvas = np.full((H_IMG, W_IMG, 3), 128, np.uint8)
    canvas = cv2.GaussianBlur(canvas, (5, 5), 0)
    canvas = np.clip(canvas.astype(np.float32) + rng.normal(0, 6, canvas.shape), 0, 255)

    a = (cover > 128).astype(np.float32)[:, :, None]
    img = (np.clip((canvas * (1 - a) + warped.astype(np.float32) * a) * gain, 0, 255)
           .astype(np.uint8))
    cv2.imwrite(out_png, img)
    return img


def run_one(number, tpl, tmp='/tmp/v2x_selftest.png', **kw):
    img = synth(number, tmp, **kw)
    corners, _ = find_plate(img)
    if corners is None:
        return None, 0.0, 0
    cells = split_cells(cv2.cvtColor(rectify(img, corners), cv2.COLOR_BGR2GRAY))
    text, scores = assemble(cells, tpl)
    return text, (float(np.mean(scores)) if scores else 0.0), len(cells)


def main():
    # 面板色必须真的落在定位器的蓝色窗口里，否则这块回归项就是空转：
    # 曾经把 BGR 写成反序，面板渲染成橘色，自测照样 20/20 全过，实机照样挂。
    bh, bs, bv = cv2.cvtColor(np.uint8([[[*BODY_BGR]]]), cv2.COLOR_BGR2HSV)[0][0]
    if not (100 <= bh <= 130 and bs >= 60 and bv >= 40):
        raise AssertionError('车身面板 HSV(%d,%d,%d) 不在蓝色窗口内，回归项失效'
                             % (bh, bs, bv))

    tpl = build_templates()
    cases = ['沪A12345', '京B88990', '粤Z1A2B3', '苏E66F88']
    shots = [
        ('正对', {}),
        ('斜视15', {'yaw_deg': 15.0}),
        ('横滚4', {'roll_deg': 4.0}),
        ('逆光', {'gain': 1.6}),
        ('弱光', {'gain': 0.55}),
        ('绿车身', {'body': GREEN_BODY_BGR}),
        ('绿车身斜视', {'body': GREEN_BODY_BGR, 'yaw_deg': 15.0}),
    ]
    ok = tot = 0
    for num in cases:
        for label, kw in shots:
            tot += 1
            text, score, nchars = run_one(num, tpl, **kw)
            hit = text == norm_number(num)
            ok += hit
            print('%-10s %-8s -> %-10s 置信%.2f 字块%d %s'
                  % (num, label, text or '(空)', score, nchars, 'OK' if hit else 'MISS'))
    print('\n识别率 %d/%d = %.0f%%' % (ok, tot, 100.0 * ok / tot))

    # 留一张带面板的合成帧，肉眼确认"牌—白框—车身"三层关系画对了
    dump = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', '..', 'selftest_body.png')
    synth('沪A12345', dump)
    print('样本帧 -> %s' % os.path.abspath(dump))
    return 0 if ok >= tot * 0.8 else 1


if __name__ == '__main__':
    sys.exit(main())
