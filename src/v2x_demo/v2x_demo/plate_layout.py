#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国标蓝牌版式：字符格几何 + 车牌字模渲染。生成端和识别端共用这一份。

为什么必须有这个模块：上一版识别器用连通域切字，省份汉字每次都认错。量一下蓝牌的
尺寸就明白这不是调参能解决的——440 mm 宽排 7 个字符，序号字符之间的净空只有 5 mm，
而"沪"的三点水和"户"之间的空、"苏"的草字头和"办"之间的空都比 5 mm 大。
也就是说**字内空隙 > 字间空隙**，连通域必然把一个字拆成几块、或者把相邻两个字粘成一块。
真实 LPR 系统对民用车牌本来就不做连通域切字，而是按版式定死字符格再格内匹配，
这里照做：识别端只认 7 个字符格，生成端必须把字画进同样的格子里。

字模也不是直接拿系统字体写上去的。国标字符格 45x90 mm 是 1:2 的瘦高比例，
任何现成字体都没有这个比例，所以按格高把墨迹拉齐、再按**整池统一**的横向压缩系数
压进格宽——统一系数而不是逐字归一，是为了保住 '1' 窄、'W' 宽这类相对宽度，
逐字压满格宽会让 '1' 变成一根粗棒、和 'T' 'I' 混掉（那是更早一版 0/20 的成因）。
"""
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

CJK = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
LAT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

MM = 4.0                                        # 纹理像素 / mm
PLATE_W_MM, PLATE_H_MM = 440, 140
TEX_W, TEX_H = int(PLATE_W_MM * MM), int(PLATE_H_MM * MM)     # 1760 x 560

CELL_W_MM, CELL_H_MM = 45, 90
CELL_PX_W, CELL_PX_H = int(CELL_W_MM * MM), int(CELL_H_MM * MM)   # 180 x 360
# 七个字符格左边缘（mm）。省份格与字母格之间留白最大，字母格与首序号格之间放墨点。
CELL_X0 = [13, 83, 164, 219, 274, 329, 384]
CELL_Y0 = (PLATE_H_MM - CELL_H_MM) // 2         # 25
DOT_CX_MM = 146.0                               # 墨点中心，落在 128~164 的空档里
DOT_R_MM = 5.0

# 边框线：距牌边 5 mm、线宽 5 mm（GA 36 的规定），所以边框**内侧**离边 10 mm。
# 这个 10 mm 是识别端的锚点：蓝底被白框切成框内、框外两块，框外那圈只有 10 mm 宽，
# 而它正是"蓝牌"和"蓝色车身"粘连在一起的桥。识别器靠腐蚀断桥，然后拿框内蓝底
# 当作车牌矩形，再按 INSET_MM 反推回整张牌的位置。
BORDER_OUT_MM = 5.0
BORDER_W_MM = 5.0
INSET_MM = BORDER_OUT_MM + BORDER_W_MM          # 10.0

BLUE = (20, 60, 160)                            # PIL RGB 顺序
WHITE = (255, 255, 255)

PROVINCES = '京津沪渝冀豫云辽黑湘皖鲁新苏浙赣鄂桂甘晋蒙陕吉闽贵粤青藏川宁琼'
AGENCIES = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
TAILS = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ'

POOLS = {'prov': (CJK, PROVINCES),
         'agency': (LAT, AGENCIES),
         'tail': (LAT, TAILS)}
# 版式固定为 [省份][字母]·[5 位序号]
SLOT_POOLS = ['prov', 'agency'] + ['tail'] * 5

# 墨迹目标尺寸：格高 90 mm 留 4 mm 上下边距，格宽 45 mm 留 2 mm 左右边距
INK_H_PX = int(86 * MM)
INK_W_PX = int(43 * MM)
_BASE = 320                                     # 字模基准字号，之后再保形缩放


def norm_number(s):
    return (s or '').strip().upper().replace(' ', '').replace('·', '')


def _ink(a):
    ys, xs = np.nonzero(a > 40)
    if len(xs) == 0:
        return None
    return a[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


_RAW = {}


def _raw(ch, font_path):
    key = (ch, font_path)
    if key not in _RAW:
        font = ImageFont.truetype(font_path, _BASE)
        img = Image.new('L', (_BASE * 2, _BASE * 2), 0)
        ImageDraw.Draw(img).text((_BASE // 2, _BASE // 2), ch, font=font, fill=255)
        _RAW[key] = _ink(np.array(img))
    return _RAW[key]


_SQUEEZE = {}


def squeeze(pool):
    """整池统一的横向压缩系数：让该池最宽的字刚好塞进格宽，其余字按比例留白。"""
    if pool not in _SQUEEZE:
        font_path, chars = POOLS[pool]
        widest = 0.0
        for ch in chars:
            a = _raw(ch, font_path)
            if a is not None:
                widest = max(widest, a.shape[1] * INK_H_PX / float(a.shape[0]))
        _SQUEEZE[pool] = min(1.0, INK_W_PX / widest) if widest > 0 else 1.0
    return _SQUEEZE[pool]


_GLYPH = {}


def cell_glyph(ch, pool):
    """字符 -> (CELL_PX_H x CELL_PX_W) uint8 位图，墨迹居中于字符格。"""
    key = (ch, pool)
    if key not in _GLYPH:
        font_path, _ = POOLS[pool]
        a = _raw(ch, font_path)
        if a is None:
            _GLYPH[key] = None
        else:
            h, w = a.shape
            nw = max(1, int(round(w * INK_H_PX / float(h) * squeeze(pool))))
            g = cv2.resize(a, (nw, INK_H_PX), interpolation=cv2.INTER_AREA)
            box = np.zeros((CELL_PX_H, CELL_PX_W), np.uint8)
            y0, x0 = (CELL_PX_H - INK_H_PX) // 2, (CELL_PX_W - nw) // 2
            box[y0:y0 + INK_H_PX, x0:x0 + nw] = g
            _GLYPH[key] = box
    return _GLYPH[key]


def cell_box_tex(i):
    """第 i 个字符格在 1760x560 纹理上的像素框 (x0, y0, x1, y1)。"""
    x0 = int(round(CELL_X0[i] * MM))
    y0 = int(round(CELL_Y0 * MM))
    return x0, y0, x0 + CELL_PX_W, y0 + CELL_PX_H


def render_plate_image(number):
    """按版式画一张蓝牌，返回 (H x W x 3) uint8 的 **BGR** 图。

    返回 BGR 而不是 PIL 对象：识别端和自检脚本都直接喂给 cv2，
    生成端要存 PNG 时再 cv2.imwrite 即可，两边都不必再碰 PIL。
    """
    number = norm_number(number)
    base = np.zeros((TEX_H, TEX_W, 3), np.uint8)
    base[:] = BLUE[::-1]                        # RGB -> BGR

    for i, ch in enumerate(number[:7]):
        g = cell_glyph(ch, SLOT_POOLS[i])
        if g is None:
            continue
        x0, y0, _, _ = cell_box_tex(i)
        a = (g.astype(np.float32) / 255.0)[:, :, None]
        roi = base[y0:y0 + CELL_PX_H, x0:x0 + CELL_PX_W].astype(np.float32)
        base[y0:y0 + CELL_PX_H, x0:x0 + CELL_PX_W] = \
            (roi * (1 - a) + np.array(WHITE[::-1], np.float32) * a)

    img = Image.fromarray(base[:, :, ::-1])     # BGR -> RGB，只为了用 PIL 画圆角边框
    d = ImageDraw.Draw(img)
    b = int(round(BORDER_OUT_MM * MM))
    d.rectangle([b, b, TEX_W - 1 - b, TEX_H - 1 - b],
                outline=WHITE, width=int(round(BORDER_W_MM * MM)))
    cx, cy = DOT_CX_MM * MM, TEX_H / 2.0
    r = DOT_R_MM * MM
    d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=WHITE)
    return np.array(img)[:, :, ::-1].copy()


def render_plate(number, out_png=None):
    """画牌；给了 out_png 就顺手落盘，返回 BGR 图。"""
    img = render_plate_image(number)
    if out_png:
        cv2.imwrite(out_png, img)
    return img
