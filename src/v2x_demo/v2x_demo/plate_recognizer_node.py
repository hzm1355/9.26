#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务三（三）内置 OpenCV 参考车牌识别节点。

职责：订阅卡口抓拍，跑一遍"定位 -> 透视校正 -> 格内匹配"，把结果发回 ROS2。
它同时是**外部算法的对照基线**：任何第三方识别节点只要改订 /plate/capture、
按同样的 PlateResult 字段发 /plate/result，就能直接替换本节点参与识别率统计。

  输入  /plate/capture   v2x_msgs/PlateCapture   卡口抓拍（含 image_path）
  输入  /plate/number    std_msgs/String         "car_1|rear|p00003|沪A12345" 真值
  输出  /plate/result    v2x_msgs/PlateResult

识别率统计需要真值，所以这里维护 model -> number 的最近一次映射；
没有真值时 matched 一律为 False，不影响 plate_number 输出。

蓝牌 HSV 范围 (100,60,40)-(130,255,255) 是 #28 spike 在实拍帧上标定的，
对应 plate_layout 的 (20,60,160) BGR 底色。

切字走**定死的字符格**而不是连通域，原因见 plate_layout 模块头：蓝牌序号字符之间净空
只有 5 mm，比汉字字内的部件空隙还小，按连通域切必然把"沪"切成"户"。
字模也来自 plate_layout，所以本节点考核的是"定位 -> 透视校正 -> 格内匹配"这条成像链路，
不是字体泛化能力；换第三方算法时按上面的话题契约替换即可。
"""
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import PlateCapture, PlateResult

from plate_layout import (CELL_H_MM, CELL_W_MM, CELL_X0, CELL_Y0, INSET_MM,
                          POOLS, SLOT_POOLS, cell_glyph, norm_number)

# 国标蓝牌 440x140 mm，校正后按 1 px/mm 落到这个画布，与 plate_layout 的毫米坐标一一对应。
# 不能再小：卡口原图里车牌只有约 303x82 px，画布取 220x70 会把字高从 62 px 压到 31 px，
# 模板匹配直接全表崩（自检 0/20）。也不能再大，超出原图分辨率只是插值出假细节。
CAN_W, CAN_H = 440, 140
# 车牌候选的形状门槛（minAreaRect 的长边/短边）。物理含义比数字重要：竖直车牌
# 投到画面里的长宽比 = 3.5 / sin(相机看牌的俯视角)，所以下限 2.2 挡的是"牌面几乎
# 正对相机却扁成方块"的东西（车尾面板 0.8x0.4 m 投出来就有 1.85），上限则是"相机
# 吊得够不够平"。
#
# 上限不能按一套安装几何标。卡口龙门俯角 39~53°→ 实测 4.4~5.6；停车场闸机相机吊在
# 4.7 m 高、离牌 4.4 m，俯角只有约 25°，同一块牌投出来实测 5.65（gate_out-13，
# 494x88 px）。原来按卡口标的 5.5 正好把它切在门槛外，于是整块牌被当成"太扁的条"
# 整块拒掉，识别器退去捡字符缝里 66x26 的小蓝块，拼出 粤LLDTYL 0.297 判未识别
# —— 图是对的、分是低的，人眼看画面完全看不出门槛的事。
# 7.0 = 覆盖闸机 5.65 并留 24% 余量（相机再往后挪也到不了 7）；卡口那侧放宽前后
# 在 12 个历史抓拍目录上逐张比对，胜出框/文本/分数一字不差（tools/diag_batch139.sh）。
# 写成常量而不是埋在 if 里，是为了让离线体检脚本能在同一批历史抓拍上对比宽严。
PLATE_RATIO_MIN = 2.2
PLATE_RATIO_MAX = 7.0
# 字符格四周多取的像素数，用来吸收透视校正带来的几像素错位
MARGIN = 4
# 字模和字符格两边同做的低通核。卡口帧里车牌只有约 300x70 px，投到 1 px/mm 的
# 画布上等于把字放大约 1.4 倍，校正图的高频全是插值出来的假细节；而字模是
# 4 px/mm 渲染后降采样、边缘干净。直接拿干净字模去比，同一枚"6"在不同格子里
# 能差出 0.77~0.95，中间那格就被形状相近的"S"抢走（实机 gate_j2_south-20）。
# 两边一起削掉采集链路根本承载不了的高频，只留笔画走向这种真传得下来的信息。
#
# 核大小由 tools/blur_sweep.py 扫参定的：合成自检集在 1x1~11x11 全都能 28/28，
# 只看它对核完全不敏感，真正拉开差距的是实机帧上"第一名领先第二名多少"。
# 以出问题那帧的最小领先量算，1x1 只有 0.003（等于掷硬币），5x5 起才稳定读对，
# 9x9 拿到 0.017 且另两帧正常抓拍仍保有 0.060/0.068；再往上糊到 11x11，好帧
# 开始掉到 0.05 以下。取 9x9。
BLUR_KSIZE = (9, 9)


def _glyph_small(ch, pool):
    """字符格位图从 4 px/mm 降到画布的 1 px/mm，INTER_AREA 保住笔画粗细比例。"""
    g = cell_glyph(ch, pool)
    if g is None:
        return None
    return cv2.resize(g, (CELL_W_MM, CELL_H_MM), interpolation=cv2.INTER_AREA)


def build_templates():
    """字模集合：{'prov': [(ch, float32 模板)], 'agency': [...], 'tail': [...]}"""
    out = {}
    for pool, (_font, chars) in POOLS.items():
        items = []
        for ch in chars:
            g = _glyph_small(ch, pool)
            if g is not None:
                items.append((ch, cv2.GaussianBlur(g, BLUR_KSIZE, 0).astype(np.float32)))
        out[pool] = items
    return out


def _order_quad(pts):
    """把 4 个角点排成 左上->右上->右下->左下。

    图像系 y 向下，所以：左上 = x+y 最小，右下 = x+y 最大，
    右上 = x-y 最大，左下 = x-y 最小。上一版把右上/左下取了反，校正图整张上下翻，
    七个字符全部认错。
    """
    s = pts.sum(axis=1)
    d = pts[:, 0] - pts[:, 1]
    q = np.array([pts[int(np.argmin(s))], pts[int(np.argmax(d))],
                  pts[int(np.argmax(s))], pts[int(np.argmin(d))]], np.float32)
    # 上下边比左右边短，说明长边被判给了竖直方向，循环挪一位转回水平
    if np.linalg.norm(q[1] - q[0]) < np.linalg.norm(q[2] - q[1]):
        q = np.roll(q, 1, axis=0)
    return q


def find_plate(img_bgr):
    """在抓拍帧里定位蓝牌外接四边形。返回 (角点 4x2 float32, 面积) 或 (None, 0)。

    两处不显眼但都是踩出来的：

    1) 形态学只能开、不能闭。蓝牌的外沿（边框线之外那 5 mm）和车身是同一种蓝，
       两者本来就是一片；把车牌内底和车身隔开的只有 5 mm 宽的白色边框线，
       落到 1920 宽的画面里只有约 3 px。原先这里做 MORPH_CLOSE 5x5，闭运算把这条
       3 px 的白缝直接糊掉，车牌和车身连成**一个**轮廓——实测整辆车糊成 644x579、
       长宽比 1.11 的团，被长宽比门槛拒掉，于是"未定位到车牌"。
       改成 MORPH_OPEN 7x7：凡是塞不进 7x7 的蓝色结构都被抹掉，3 px 的桥必然断，
       而约 300x80 的车牌内底开完还是它自己，尺寸不缩。
    2) 角点取轮廓的多边形近似而不是 minAreaRect：车辆斜着过卡口时车牌投出来是**梯形**
       （近的一端高、远的一端矮），外接矩形和梯形四个顶点能差十几像素，
       而字符格是按毫米坐标定死的，这点偏移足够把 '1' 认成 'A'。
    3) 框内"亮墨迹"用饱和度双峰判定，不用固定 V>=150 判白（见 _ink_fraction）。
    4) 轮廓检索必须用 RETR_LIST，不能用 RETR_EXTERNAL。车牌外沿那圈蓝和车身是同一种
       蓝，中间只隔着白色边框线，于是"车牌蓝底"在蓝色掩膜里是**嵌在车身轮廓的洞里**
       的一个分量。RETR_EXTERNAL 只返回最外层轮廓，被洞包住的组件一个都不给——
       实测整帧只出来 1 条轮廓、面积 222671、长宽比 2.20 的车身大团，
       车牌连被评判的机会都没有，症状和"画面里没牌"完全一样。
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (100, 60, 40), (130, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))

    cands = []
    contours, _ = cv2.findContours(blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area < 400:
            continue
        rect = cv2.minAreaRect(c)
        (w, h) = rect[1]
        if w <= 0 or h <= 0:
            continue
        ratio = max(w, h) / min(w, h)
        # 白框内侧的蓝底是 420:120=3.5。下限取 2.2 是为了把车身面板挡在外面：
        # 车尾面 0.8x0.4 m 投出来长宽比就有 1.85，再算上它框住车牌带来的白占比，
        # 门槛松一点它就会以"面积更大"抢走最优。
        if not (PLATE_RATIO_MIN <= ratio <= PLATE_RATIO_MAX):
            continue
        if area / (w * h) < 0.55:
            continue
        # 牌面是"蓝底 + 白字白框"，框内饱和度呈双峰；一整块蓝漆是单峰。
        # 占比区间卡掉斑马线白块（低饱和占绝大多数），分离度卡掉均匀蓝漆。
        ink, sep = _ink_fraction(img_bgr, rect)
        if not (0.12 <= ink <= 0.65) or sep < 1.3:
            continue
        cands.append((area, c, rect))

    if not cands:
        return None, 0.0
    cands.sort(key=lambda t: -t[0])
    area, c, rect = cands[0]
    # 同一条白色边框线会造出**两个**几乎重合的合格候选：边框线外沿（离牌边 5 mm）
    # 和蓝底外沿（离牌边 10 mm）。rectify 是按蓝底外沿定标的，取错外面那条，
    # 字符格会沿牌宽方向拉开 4~6 px，'1' 和 'I' 就分不开了。
    # 所以往回收一层：落在胜出框内、面积更小的候选里取最大的那个。
    # 面积必须和胜出框同量级（>=0.5 倍）才算"另一条牌沿"。这条约束是实机逼出来的：
    # 蓝色车身时边框线外沿能活过开运算，两层候选都在；换成绿车身，那圈 3 px 的
    # 牌边蓝漆会被开运算直接抹掉，胜出框已经是蓝底外沿了，此时若不卡面积，
    # 被"收进去"的就是 'A'、'B' 笔画围出来的那几个小蓝块（面积约牌的 4%），
    # 实测 car_2/car_3/car_5 三辆非蓝车全部识别为空，只有蓝车的 car_1/car_4 命中。
    inner = [(a2, c2, r2) for a2, c2, r2 in cands[1:]
             if a2 >= 0.5 * area
             and cv2.pointPolygonTest(c, _centroid(c2), False) >= 0]
    if inner:
        area, c, rect = inner[0]

    peri = cv2.arcLength(c, True)
    for eps in (0.01, 0.02, 0.03, 0.05):
        approx = cv2.approxPolyDP(c, eps * peri, True)
        if len(approx) == 4:
            return _order_quad(approx.reshape(4, 2).astype(np.float32)), area
    # 轮廓被遮挡或边缘毛糙到拟不出四边形，退回外接矩形（此时只认正对场景）
    return _order_quad(cv2.boxPoints(rect).astype(np.float32)), area


def _centroid(contour):
    m = cv2.moments(contour)
    if m['m00'] == 0:
        x, y, w, h = cv2.boundingRect(contour)
        return (float(x + w / 2.0), float(y + h / 2.0))
    return (m['m10'] / m['m00'], m['m01'] / m['m00'])


def _ink_fraction(img_bgr, rect):
    """框内**饱和度**双峰统计，返回 (低饱和墨迹占比, 高饱和均值/低饱和均值)。

    判据是 S 不是 V，两条都是踩出来的：
    - 固定 V>=150 判白扛不住乘性增益：弱光预设下白字 V 掉到 140 就被判成非白，
      占比归零、所有候选一起被否，报出的"未定位到车牌"和画面里真没牌长得一样。
    - 换成框内 V 的 Otsu 自动二分能治弱光，却又在逆光爆闪下失效：白字和蓝底
      一起被顶到 255，亮度上根本分不开。
    S = (max-min)/max 是比值量，白字的判据本质就是"没有颜色"，所以它对全局增益
    和削顶都免疫：蓝牌底 S≈223、白字 S≈0，比值一眼可分；而一整块蓝漆车身 S 均匀，
    Otsu 只能在噪声上瞎切，分离度接近 1。
    """
    box = cv2.boxPoints(rect).astype(np.int32)
    x, y, w, h = cv2.boundingRect(box)
    x0, y0 = max(x, 0), max(y, 0)
    x1, y1 = min(x + w, img_bgr.shape[1]), min(y + h, img_bgr.shape[0])
    if x1 - x0 < 8 or y1 - y0 < 8:
        return 0.0, 0.0

    poly = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.fillConvexPoly(poly, box - [x0, y0], 255)
    sat = cv2.cvtColor(img_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)[:, :, 1]
    vals = sat[poly > 0]
    if vals.size < 64:
        return 0.0, 0.0

    thr = cv2.threshold(vals.reshape(1, -1), 0, 255,
                        cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
    lo, hi = vals[vals <= thr], vals[vals > thr]
    if lo.size == 0 or hi.size == 0:
        return 0.0, 0.0
    return (float(lo.size) / float(vals.size),
            float(hi.mean()) / max(1.0, float(lo.mean())))


def rectify(img_bgr, corners):
    """把倾斜的车牌拉回 CAN_W x CAN_H 的正视图。

    corners 是**白框内侧**那圈蓝底的角点，不是整张牌的外沿，所以目标框要按
    plate_layout.INSET_MM 往里缩，缩完仍然落在整张牌的毫米坐标系里，
    下游的字符格坐标可以直接照用 CELL_X0/CELL_Y0。
    """
    i = INSET_MM
    dst = np.array([[i, i], [CAN_W - 1 - i, i],
                    [CAN_W - 1 - i, CAN_H - 1 - i], [i, CAN_H - 1 - i]], np.float32)
    m = cv2.getPerspectiveTransform(corners, dst)
    return cv2.warpPerspective(img_bgr, m, (CAN_W, CAN_H))


def split_cells(plate_gray):
    """校正图 -> 7 个带边距的字符格切片，按版式顺序排（不按 x 重排）。

    越界的格子直接跳过，让 assemble 报"格子数不对"而不是硬猜出一个错号。
    """
    out = []
    for i in range(7):
        x0 = CELL_X0[i] - MARGIN
        y0 = CELL_Y0 - MARGIN
        x1 = CELL_X0[i] + CELL_W_MM + MARGIN
        y1 = CELL_Y0 + CELL_H_MM + MARGIN
        if x0 < 0 or y0 < 0 or x1 > CAN_W or y1 > CAN_H:
            continue
        out.append((plate_gray[y0:y1, x0:x1], CELL_X0[i] + CELL_W_MM * 0.5))
    return out


def classify(crop, templates, pool):
    """格内滑动匹配，返回 (字符, 置信度)。

    用 TM_CCOEFF_NORMED 而不是手写 NCC：它自带零均值单位方差归一化，
    一次算完 (2*MARGIN+1)^2 个偏移取最优，把校正的几像素错位一起吃掉。
    字符格先按 BLUR_KSIZE 低通，理由见该常量的注释——它必须和字模同带宽。
    """
    src = cv2.GaussianBlur(crop, BLUR_KSIZE, 0).astype(np.float32)
    best, score = '?', -1.0
    for ch, t in templates[pool]:
        s = float(cv2.matchTemplate(src, t, cv2.TM_CCOEFF_NORMED).max())
        if s > score:
            best, score = ch, s
    return best, score


def assemble(cells, templates, log=None):
    """把 7 个字符格拼成车牌号。

    格子数不对时宁可输出空号也不输出错号——识别率统计里错号比空号更难排查。
    """
    if len(cells) != 7:
        if log:
            log('只取到 %d 个字符格，期望 7，放弃拼读' % len(cells))
        return '', []
    text, scores = '', []
    for (crop, _), pool in zip(cells, SLOT_POOLS):
        ch, s = classify(crop, templates, pool)
        text += ch
        scores.append(s)
    return text, scores


class PlateRecognizerNode(Node):
    def __init__(self):
        super().__init__('plate_recognizer_node')
        self.declare_parameter('algorithm', 'builtin_opencv')
        self.declare_parameter('min_confidence', 0.35)
        self.algorithm = self.get_parameter('algorithm').value
        self.min_conf = float(self.get_parameter('min_confidence').value)

        self.templates = build_templates()
        self.truth = {}

        self.pub = self.create_publisher(PlateResult, '/plate/result', 50)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, 50)
        self.create_subscription(String, '/plate/number', self.on_number, 10)
        self.get_logger().info('[recognizer] 参考识别器就绪，算法名 %s' % self.algorithm)

    def on_number(self, msg):
        parts = msg.data.split('|')
        if len(parts) == 4 and parts[3]:
            self.truth[parts[0]] = norm_number(parts[3])

    def on_capture(self, msg):
        t0 = time.perf_counter()
        r = PlateResult()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'world'
        r.capture_id = msg.capture_id
        r.gate_id = msg.gate_id
        r.capture_stamp = msg.header.stamp
        r.vehicle_pose = msg.pose
        r.speed = msg.speed
        r.algorithm = self.algorithm
        r.truth_number = self.truth.get(msg.vehicle_id, '')

        img = cv2.imread(msg.image_path) if msg.image_path else None
        if img is None:
            self.get_logger().warn('抓拍图不可读: %s' % msg.image_path)
        else:
            corners, _ = find_plate(img)
            if corners is None:
                self.get_logger().info('%s 未定位到车牌' % msg.capture_id)
            else:
                plate = rectify(img, corners)
                cells = split_cells(cv2.cvtColor(plate, cv2.COLOR_BGR2GRAY))
                text, scores = assemble(cells, self.templates,
                                        lambda m: self.get_logger().info(m))
                x, y, w, h = cv2.boundingRect(corners.astype(np.int32))
                # bbox_* 是 uint16，而 corners 来自 minAreaRect：牌贴到画面边缘时
                # 角点会落在图像外（x 或 y 为负），直接赋值会让整个节点抛 AssertionError
                # 崩掉——高速档"牌被裁掉一半"恰好最容易踩到。钳到图内再报。
                ih, iw = img.shape[:2]
                x2, y2 = min(iw, x + w), min(ih, y + h)
                x, y = max(0, x), max(0, y)
                r.bbox_x, r.bbox_y = x, y
                r.bbox_w, r.bbox_h = max(0, x2 - x), max(0, y2 - y)
                r.plate_number = text
                r.confidence = float(np.mean(scores)) if scores else 0.0
                if r.confidence < self.min_conf:
                    self.get_logger().info(
                        '%s 置信度 %.2f 低于阈值，判为未识别' % (msg.capture_id, r.confidence))
                    r.plate_number = ''

        r.matched = bool(r.plate_number) and r.plate_number == r.truth_number
        r.process_ms = (time.perf_counter() - t0) * 1000.0
        self.pub.publish(r)
        self.get_logger().info(
            '[recognizer] %s -> %r 置信%.2f 真值%r %s (%.0f ms)'
            % (msg.capture_id, r.plate_number, r.confidence, r.truth_number,
               '命中' if r.matched else '未命中', r.process_ms))


def main():
    rclpy.init()
    node = PlateRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
