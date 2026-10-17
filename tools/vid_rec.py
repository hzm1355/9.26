#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视频取证录制器：把 Gazebo 相机画面 + ROS2 实时状态合成一段带中文字幕的 MP4。

为什么不用 gzclient 窗口抓屏：WSL 侧没有 ffmpeg / imagemagick，x11grab 走不通；
而相机 image_raw 本身就是 Gazebo 渲染出来的 3D 画面，再叠加"当前条目、下发的指令、
六组灯态、识别结果"这些 ROS2 侧的真相，比单纯录窗口更适合做验收证据。

版面：左侧 1~2 路画面（上下堆叠），右侧信息面板，底部字幕条（读 caption 文件）。
字幕文件由驱动脚本逐行追加，录制器每帧重读，所以"做了什么动作"和"画面变化"同框。

画面源三种写法（--views 逗号分隔，混用）：
  http:<name>     走本机 :8080 四路实时回显（car1/speed/inter/monitor）。
                  首选。那个服务常驻运行、DDS 早已匹配好，取帧是纯 HTTP 拉取，
                  彻底避开"新建 image_raw 订阅随机 120s 才收到首帧"的问题。
  file:<glob>     取 glob 匹配里 mtime 最新的一张图（卡口抓拍 PNG 就靠这个）。
                  卡口相机没有 HTTP 端点，而 /tmp/v2x_gates/*.png 是插件按次落盘的
                  真实抓拍，录进去比实时流更贴"电子警察取证"这件事。
  /topic          ROS2 image_raw 订阅，兜底用。
  任一路后面可以接 `#crop=x:y:w:h`（原图像素，冒号分隔）只取一块并放大铺满该格：
  高位相机里停车场只占画面一角，不裁的话"闸杆抬起来"就十几个像素，读片的人看不出来。
  同一路画面允许写两次（一次全景一次特写），话题只订一次。

字体坑：系统里唯一的 CJK 字体 DroidSansFallbackFull.ttf 是**纯 fallback**，
不含 ASCII 字形，直接用它写数字/字母会全是方框。所以整行按字符拆成 ASCII 段
（DejaVuSans）与 CJK 段（Droid）分别渲染再拼接，结果按 (文本, 字号, 颜色) 缓存。

用法：
  python3 vid_rec.py --out /mnt/d/.../条目1.mp4 --dur 100 --mode tl \
      --views http:inter --title "任务二 条目1  灯色真实时序"
  --mode speed 时右侧面板换成测速路段（逐点位统计 / 瞬时抓拍 / 区间通行 / 告警五要素）。
  --mode parking 时换成停车场面板（闸机"指令角 vs 实测角" / 抬落计数 / 地感占用、
  通行记录三段耗时、闸机动作日志、car_6 摆渡车进度行）。画面源一路给卡口抓拍落盘图、
  一路给高位相机的停车场局部（实测 intersection_cam 800x600 里停车场在 200,330 起
  400x270 那块）：
  --views 'http:inter#crop=200:330:400:270,file:/tmp/v2x_parking_in/*.png'
"""
import argparse
import glob
import json
import math
import os
import threading
import time
import urllib.request

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from v2x_msgs.msg import (TrafficLightState, TrafficLightStats, PlateResult,
                          PlateAlert, PlateCapture,
                          SpeedCapture, SpeedSegment, SpeedAlert, SpeedStats,
                          BarrierState, ParkingPass, ParkingAction)

CJK = '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf'
LAT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
GRPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
PH_COLOR = {'GREEN': (60, 200, 90), 'YELLOW': (230, 200, 60), 'RED': (215, 80, 70),
            'WALK': (60, 200, 90), 'FLASHING': (230, 200, 60), 'DONT_WALK': (215, 80, 70)}
# 停车场面板的中文译名表，字段名与 live_view_server.py 那一套对齐：同一刻的看板截图和
# 取证视频要对同一条消息说同样的话，不然验收时对不上号。
BARRIER_STATE = {BarrierState.STATE_CLOSED: '落下', BarrierState.STATE_OPENING: '抬杆中',
                 BarrierState.STATE_OPEN: '抬起', BarrierState.STATE_CLOSING: '落杆中',
                 BarrierState.STATE_FAULT: '故障挂起'}
BARRIER_SCOL = {BarrierState.STATE_CLOSED: (215, 80, 70),
                BarrierState.STATE_OPENING: (230, 200, 60),
                BarrierState.STATE_OPEN: (60, 200, 90),
                BarrierState.STATE_CLOSING: (230, 200, 60),
                BarrierState.STATE_FAULT: (200, 120, 255)}
BARRIER_SRC = {BarrierState.SRC_AUTO: '自动', BarrierState.SRC_MANUAL: '手动',
               BarrierState.SRC_FAULT: '故障令'}
BARRIER_FAULT = {BarrierState.FAULT_NONE: '', BarrierState.FAULT_STUCK: '卡滞',
                 BarrierState.FAULT_RAISE_FAIL: '抬杆失败',
                 BarrierState.FAULT_NO_CLOSE: '落杆失败',
                 BarrierState.FAULT_SENSOR_FAIL: '地感失效'}
PASS_KIND = {ParkingPass.KIND_MONTHLY: '月租', ParkingPass.KIND_TEMP: '临时',
             ParkingPass.KIND_UNKNOWN: '未登记', ParkingPass.KIND_BLACKLIST: '黑名单'}
PASS_RESULT = {ParkingPass.RESULT_ALLOWED: '放行', ParkingPass.RESULT_DENIED: '拒绝',
               ParkingPass.RESULT_PENDING: '挂起'}
PASS_DIR = {ParkingPass.DIR_IN: '进场', ParkingPass.DIR_OUT: '出场'}
WL_NAME = {0: '未查', 1: '白名单命中', 2: '名单未命中', 3: '命中黑名单'}
ACT_NAME = {ParkingAction.ACT_RAISE: '抬杆到位', ParkingAction.ACT_LOWER: '落杆到位',
            ParkingAction.ACT_RAISE_CMD: '下达抬杆', ParkingAction.ACT_LOWER_CMD: '下达落杆',
            ParkingAction.ACT_HOLD_OPEN: '防砸重抬', ParkingAction.ACT_REFUSE: '拒绝动作',
            ParkingAction.ACT_FAULT: '故障生效', ParkingAction.ACT_FAULT_CLEAR: '故障清除',
            ParkingAction.ACT_MANUAL: '手动生效', ParkingAction.ACT_DEGRADE: '降级计时'}
# 只有这几类动作的 note 值得单独占一行（名单命中那种正常放行注记走"够位置才画"的分支）
ACT_NOTE_KEYS = (ParkingAction.ACT_REFUSE, ParkingAction.ACT_FAULT,
                 ParkingAction.ACT_MANUAL, ParkingAction.ACT_DEGRADE,
                 ParkingAction.ACT_HOLD_OPEN, ParkingAction.ACT_FAULT_CLEAR)
AP = argparse.ArgumentParser()
AP.add_argument('--out', required=True)
AP.add_argument('--dur', type=float, default=60.0)
AP.add_argument('--fps', type=float, default=10.0)
AP.add_argument('--views', default='/intersection_cam/intersection_cam/image_raw')
AP.add_argument('--title', default='')
AP.add_argument('--caption', default='/tmp/vid_caption.txt')
AP.add_argument('--mode', default='tl',
                choices=['tl', 'plate', 'both', 'speed', 'parking', 'none'])
A = AP.parse_args()
VIEWS = [v.strip() for v in A.views.split(',') if v.strip()]
# 每一路可以带 `#crop=x:y:w:h`（原图像素坐标）：高位相机里停车场只占一小块，
# 不裁的话"闸杆抬起来"这件事在画面上就是十几个像素，读片的人看不出来。裁的是
# 取回来的原图，缩放仍由 fit() 等比处理，不会比整帧显示更糊。
# 分隔符必须是冒号不是逗号：--views 自己就是逗号分隔的，写逗号会把一个框撕成四路。
# 同一路画面可以出现两次（一次全景一次特写），所以裁切框按"这一路"存，不按话题存。
KEY, BOX = {}, {}
for _v in list(VIEWS):
    base, _, tag = _v.partition('#crop=')
    KEY[_v] = base
    if _v == base:
        continue
    try:
        x, y, w, h = [int(t) for t in tag.split(':')]
    except ValueError:
        raise SystemExit('--crop 要写成 #crop=x:y:w:h（整数像素、冒号分隔），收到 %r' % _v)
    if w <= 0 or h <= 0:
        raise SystemExit('--crop 的宽高必须为正：#crop=%s' % tag)
    BOX[_v] = (x, y, w, h)
# 把每一路画面按前缀分流：ROS 订阅要在进程起来很久以后才随机匹配上，能绕开就绕开。
SRC, ROSV = {}, []
for v in dict.fromkeys(KEY.values()):        # 同一话题只订一次/只拉一次
    if v.startswith('http:'):
        SRC[v] = ('http', 'http://localhost:8080/cam/%s' % v[5:])
    elif v.startswith('file:'):
        SRC[v] = ('file', v[5:])
    else:
        SRC[v] = ('ros', v)
        ROSV.append(v)
W, H = 1440, 810
PANEL = 450
MAIN = W - PANEL
HEAD = 34


class Renderer:
    """整行渲染成 RGBA 位图再粘贴，避免逐字符 draw 拖慢帧率。"""

    def __init__(self):
        self.f = {}
        self.cache = {}

    def font(self, cjk):
        k = 'cjk' if cjk else 'lat'
        if k not in self.f:
            self.f[k] = {}
        return self.f[k]

    def _fonts(self, sz):
        d = self.font(True)
        if sz not in d:
            d[sz] = ImageFont.truetype(CJK, sz)
            self.font(False)[sz] = ImageFont.truetype(LAT, sz)
        return self.font(False)[sz], d[sz]

    @staticmethod
    def _is_ascii(s):
        # 0x2E80 之前都是 DejaVu 覆盖的拉丁/符号区（含 ↑、°、×），之后才交给 Droid CJK。
        # DroidSansFallback 不含这些字形，按"仅 ASCII 走拉丁"切会渲染成方框。
        return all(ord(c) < 0x2E80 for c in s)

    def img(self, s, sz, col):
        key = (s, sz, col)
        r = self.cache.get(key)
        if r is not None:
            return r
        lat, cjk = self._fonts(sz)
        runs = []
        i = 0
        while i < len(s):
            j = i
            ascii_run = self._is_ascii(s[i])
            while j < len(s) and self._is_ascii(s[j]) == ascii_run:
                j += 1
            runs.append((s[i:j], lat if ascii_run else cjk))
            i = j
        asc = sum((f.getlength(t) for t, f in runs), 0.0)
        im = Image.new('RGBA', (int(asc) + 2, sz + 10), (0, 0, 0, 0))
        d = ImageDraw.Draw(im)
        x = 1.0
        for t, f in runs:
            d.text((x, 2), t, font=f, fill=col + (255,))
            x += f.getlength(t)
        if len(self.cache) > 6000:
            self.cache.clear()
        self.cache[key] = im
        return im

    def text(self, im, x, y, s, sz, col):
        """返回下一行的 y（行高 = 字号 + 5）"""
        if not s:
            return y + sz + 5
        g = self.img(s, sz, col)
        im.paste(g, (int(x), int(y)), g)
        return y + sz + 5


R = Renderer()


class Rec(Node):
    """所有订阅都只留"最新一帧"，录制主线程按 fps 取用。"""

    def __init__(self):
        # 节点名带 pid：上一段录制被 kill -9 后同名节点的 DDS 残留会让新订阅收不到图像帧
        super().__init__('vid_rec_%d' % os.getpid())
        self.img = {}
        self.det, self.st = {}, {}
        self.adap, self.simt = None, None
        self.plates, self.alerts = [], []
        self.algo = {}
        self.n_cap = self.n_res = self.n_hit = 0
        self.s_caps, self.s_segs, self.s_als = [], [], []
        self.s_stats = {}
        self.s_round = 0.0
        self.n_scap = self.n_sseg = self.n_salt = 0
        self.gates, self.passes, self.acts = {}, [], []
        self.shut = ''
        self.n_pass = self.n_act = 0
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        for t in ROSV:
            self.create_subscription(RosImage, t, lambda m, t=t: self.on_img(t, m), q)
        if A.mode in ('tl', 'both'):
            for g in GRPS:
                self.create_subscription(
                    TrafficLightState, f'/traffic_light/{g}/state_detail',
                    lambda m, gid=g: self.det.__setitem__(gid, m), 10)
                self.create_subscription(
                    TrafficLightStats, f'/traffic_light/{g}/stats',
                    lambda m, gid=g: self.st.__setitem__(gid, m),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST))
            self.create_subscription(String, '/traffic_light/adaptive_stats',
                                     lambda m: setattr(self, 'adap', m.data), 10)
        if A.mode in ('plate', 'both'):
            self.create_subscription(PlateCapture, '/plate/capture', self.on_cap, q)
            self.create_subscription(PlateResult, '/plate/result', self.on_res, 20)
            self.create_subscription(PlateAlert, '/plate/alert', self.on_alert, 20)
        # 这四路必须用 RELIABLE + 足够深的队列，不能复用图像那套 depth=1 BEST_EFFORT：
        # publish_stats 一轮连发 5 个点位（stamp 相同），depth=1 的环形缓冲会在主线程
        # 取走之前把前 4 条覆盖掉，面板就只剩最后一个点位（实测"在册点位 1"）。
        # 抓拍/区间/告警同理——两辆车同刻过点时会丢事件，计数器也就假了。
        if A.mode == 'speed':
            self.create_subscription(SpeedCapture, '/speed/capture', self.on_scap, 50)
            self.create_subscription(SpeedSegment, '/speed/segment', self.on_sseg, 50)
            self.create_subscription(SpeedAlert, '/speed/alert', self.on_salt, 50)
            self.create_subscription(SpeedStats, '/speed/stats', self.on_sstats, 50)
        # 停车场这四路也要深队列，理由与测速那几路相同：一次抬杆在 /parking/actions 上是
        # "下达抬杆→抬杆到位"两条紧邻消息，depth=1 会在主线程取用前把前一条覆盖掉，
        # 取证里就只剩"到位"、看不到指令。发布侧（parking_manager_node）本来就是
        # RELIABLE depth 50，这里用 RELIABLE 不会 QoS 不兼容。
        if A.mode == 'parking':
            self.create_subscription(BarrierState, '/parking/gate_state',
                                     lambda m: self.gates.__setitem__(m.gate_id, m), 20)
            self.create_subscription(ParkingPass, '/parking/pass', self.on_pass, 20)
            self.create_subscription(ParkingAction, '/parking/actions', self.on_act, 50)
            self.create_subscription(String, '/parking/shuttle_status',
                                     lambda m: setattr(self, 'shut', m.data), 10)
        # gzserver 的 /clock 是 BEST_EFFORT 提供的，用默认 RELIABLE 订阅会 QoS 不兼容而静默无数据
        self.create_subscription(Clock, '/clock', self.on_clock, q)

    def on_img(self, topic, m):
        if m.encoding not in ('rgb8', 'bgr8'):
            return
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
        self.img[topic] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()

    def on_cap(self, m):
        self.n_cap += 1

    def on_res(self, m):
        self.n_res += 1
        self.n_hit += 1 if m.matched else 0
        self.algo[m.algorithm] = self.algo.get(m.algorithm, 0) + 1
        self.plates.append((m.capture_id, m.gate_id, m.plate_number, m.confidence,
                            m.speed, m.matched, m.algorithm,
                            m.capture_stamp.sec + m.capture_stamp.nanosec * 1e-9))
        if len(self.plates) > 40:
            del self.plates[0]

    def on_alert(self, m):
        self.alerts.append((m.level, m.plate_number, m.gate_id, m.capture_id, m.reason))
        if len(self.alerts) > 12:
            del self.alerts[0]

    def on_clock(self, m):
        self.simt = m.clock.sec + m.clock.nanosec * 1e-9

    def on_scap(self, m):
        self.n_scap += 1
        self.s_caps.append(m)
        if len(self.s_caps) > 6:
            del self.s_caps[0]

    def on_sseg(self, m):
        self.n_sseg += 1
        self.s_segs.append(m)
        if len(self.s_segs) > 4:
            del self.s_segs[0]

    def on_salt(self, m):
        self.n_salt += 1
        self.s_als.append(m)
        if len(self.s_als) > 4:
            del self.s_als[0]

    def on_sstats(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        # publish_stats 一轮给每个在册点位各发一条、stamp 相同：新轮一到就整表换新，
        # 这样 /speed/zones 删掉的点位不会一直挂在面板上。只看 st > round 不够——
        # 世界重置让 /clock 归零，旧会话的高水位 stamp 会把换新永久卡死（看板踩过）。
        if st > self.s_round or self.s_round - st > 30.0:
            self.s_round = st
            self.s_stats = {}
        self.s_stats[m.zone_id] = m

    def on_pass(self, m):
        self.n_pass += 1
        self.passes.append(m)
        if len(self.passes) > 6:
            del self.passes[0]

    def on_act(self, m):
        self.n_act += 1
        self.acts.append(m)
        # 6 条是版面撑下来的上限：面板纵向只有 ~770 px，一次抬杆就要占"下达+到位"两行，
        # 再叠故障注记的话第 7 条起会被画到画面外，读片的人看不见也猜不到。
        if len(self.acts) > 6:
            del self.acts[0]


def puller(r):
    """http:/file: 两类画面源的取帧线程，写进 r.img，与 ROS 回调共用一份缓冲。"""
    last = {}
    while True:
        for k, (kind, arg) in SRC.items():
            try:
                if kind == 'http':
                    u = '%s?t=%d' % (arg, time.time() * 1000)
                    b = urllib.request.urlopen(u, timeout=1.5).read()
                    im = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
                    if im is not None:
                        r.img[k] = im
                elif kind == 'file':
                    fs = glob.glob(arg)
                    if not fs:
                        continue
                    p = max(fs, key=os.path.getmtime)
                    m = os.path.getmtime(p)
                    if last.get(k) != m:
                        im = cv2.imread(p)
                        if im is not None:
                            last[k] = m
                            r.img[k] = im
            except Exception:
                pass                      # 单次拉取失败只是少一帧，不能让线程死掉
        time.sleep(1.0 / max(4.0, A.fps))


def crop_to(img, box):
    """按 --crop 的框切原图；框超出画面时夹到边界内，而不是抛异常把录制进程打死。"""
    if img is None or not box:
        return img
    x, y, w, h = box
    H0, W0 = img.shape[0], img.shape[1]
    x = max(0, min(x, W0 - 1))
    y = max(0, min(y, H0 - 1))
    out = img[y:min(y + h, H0), x:min(x + w, W0)]
    # 夹完什么都不剩（框整个在画面外）就退回原图：宁可画面偏小，也不要一帧黑。
    return out if out.shape[0] > 8 and out.shape[1] > 8 else img


def fit(img, w, h):
    """等比缩放置入 w×h，居中，四周留黑。"""
    if img is None:
        return np.zeros((h, w, 3), np.uint8)
    s = min(w / img.shape[1], h / img.shape[0])
    # 放大用 INTER_AREA 会出锯齿块（--crop 之后几乎都是在放大），只在缩小时才用它。
    r = cv2.resize(img, (int(img.shape[1] * s), int(img.shape[0] * s)),
                   interpolation=cv2.INTER_AREA if s < 1.0 else cv2.INTER_CUBIC)
    out = np.zeros((h, w, 3), np.uint8)
    y0 = (h - r.shape[0]) // 2
    x0 = (w - r.shape[1]) // 2
    out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    return out


def captions():
    try:
        # errors='replace' 不是偷懒：驱动脚本每 100 ms 级就往这个文件追加中文行，录制器每帧
        # 整读一次，读盘很容易正好切在一个多字节汉字的中间，严格解码会抛 UnicodeDecodeError
        # 把整个录制进程打死（实测 240 s 的片只留下 12 s）。宁可末行出一个替换符。
        with open(A.caption, encoding='utf-8', errors='replace') as f:
            ls = [x.rstrip('\n') for x in f.readlines() if x.strip()]
    except OSError:
        ls = []
    if ls:
        ls[-1] = ls[-1].rstrip('\ufffd')   # 末行可能被切在半字上，替换符不画进画面
    return [x for x in ls[-9:] if x.strip()]


def szs(z):
    """点位 ID 缩写字段宽度：SZ-A-west-instant -> A-west-instant。"""
    return z.replace('SZ-', '')


def zshort(z):
    """面板字段用：再去掉 -instant/-segment 后缀（模式已有单独一列，重复占宽）。"""
    return szs(z).replace('-instant', '').replace('-segment', '')


def panel_speed(r, im, y):
    """任务四面板：逐点位流量统计 + 两种测速模式的读数 + 告警五要素。"""
    x0 = MAIN + 12
    y = R.text(im, x0, y, '测速路段实时状态', 17, (255, 255, 255))
    y = R.text(im, x0, y, '在册点位 %d｜抓拍 %d｜区间通行 %d｜告警 %d' % (
        len(r.s_stats), r.n_scap, r.n_sseg, r.n_salt), 14, (190, 220, 190))
    if not r.s_stats:
        y = R.text(im, x0, y, '等待 /speed/stats 首帧…（检测节点每 10 s 发一轮）',
                   14, (150, 150, 150))
    else:
        y += 5
        win = next(iter(r.s_stats.values())).window_s
        y = R.text(im, x0, y, '条目4 流量统计（窗口 %.0fs 内均值 + 累计）' % win,
                   15, (255, 255, 255))
        y = R.text(im, x0, y, '点位 模式 阈值 流量 均速 峰值 超速 累计', 13, (200, 200, 200))
        for zid in sorted(r.s_stats):
            m = r.s_stats[zid]
            y = R.text(im, x0, y, '%-11s %s %4.1f %4d %5.2f %5.2f %4d %4d/%d%s' % (
                zshort(zid)[:11], '区间' if m.mode == 1 else '瞬时', m.limit_kmh,
                m.flow, m.avg_kmh, m.max_kmh, m.over_count, m.total_flow,
                m.total_over,
                (' 待%d' % m.pending_segment) if m.mode == 1 else ''),
                13, (235, 235, 235))
    if r.s_caps:
        y += 5
        y = R.text(im, x0, y, '条目2 定点瞬时（雷达=波束投影读数 / 位姿=真值对照）',
                   15, (255, 255, 255))
        y = R.text(im, x0, y, '点位 车辆 车牌 雷达 位姿 夹角 阈值 判定', 13, (200, 200, 200))
        for m in r.s_caps[-5:]:
            y = R.text(im, x0, y, '%-11s %-6s %-9s %5.2f %5.2f %5.1f %5.1f %s' % (
                zshort(m.zone_id)[:11], m.vehicle_id, m.plate_number or '-',
                m.speed_kmh, m.speed_pose_kmh, m.beam_angle_deg, m.limit_kmh,
                '超速' if m.over else '正常'),
                13, (255, 130, 110) if m.over else (130, 220, 140))
    if r.s_segs:
        y += 5
        y = R.text(im, x0, y, '条目2 区间平均（均速 = 断面长度 / 行程时间）',
                   15, (255, 255, 255))
        y = R.text(im, x0, y, '点位 车辆 车牌 长度 时长 均速 阈值 判定', 13, (200, 200, 200))
        for m in r.s_segs[-4:]:
            y = R.text(im, x0, y, '%-11s %-6s %-9s %4.1f %6.2f %5.2f %5.1f %s' % (
                zshort(m.zone_id)[:11], m.vehicle_id, m.plate_number or '-',
                m.length_m, m.travel_time_s, m.avg_kmh, m.limit_kmh,
                '超速' if m.over else '正常'),
                13, (255, 130, 110) if m.over else (130, 220, 140))
    if r.s_als:
        y += 5
        y = R.text(im, x0, y, '条目3 告警五要素（车号/车牌/速度/时刻/位置）',
                   15, (255, 130, 110))
        for m in r.s_als[-3:]:
            y = R.text(im, x0, y, '%s @%s %s %s' % (
                '区间' if m.kind == 1 else '瞬时', szs(m.zone_id)[:16],
                m.vehicle_id, m.plate_number or '-'), 13, (255, 170, 140))
            y = R.text(im, x0, y, '  速度%.2f 阈%.2f 超%.2f t=%.1f (%.2f,%.2f)' % (
                m.speed_kmh, m.limit_kmh, m.over_kmh, m.ts,
                m.pose.position.x, m.pose.position.y), 13, (255, 170, 140))
    return y


def pk_dur(a, b):
    """ParkingPass 里"还没到"的那一段是 -1，任何一侧是 -1 就返回 -1。直接相减会算出
    -13.4 s 摆在面板上，看着像提前抬杆 —— 与 parking_recorder_node、:8090 看板同一套规矩。"""
    if a < 0.0 or b < 0.0:
        return -1.0
    return b - a


def pk_sec(v):
    return '—' if v < 0.0 else '%.2f' % v


def wrap(s, n=48):
    """按显示宽度折行（半角算 1、CJK 算 2），面板一行只放得下约 48 个半角位。
    摆渡车那行进度串很长，尾部恰好是"第几轮 + 当前在做什么"，不折行就会被画面裁掉。"""
    out, cur, w = [], '', 0
    for c in s:
        k = 1 if ord(c) < 0x2E80 else 2
        if w + k > n and cur:
            out.append(cur)
            cur, w = c, k
        else:
            cur += c
            w += k
    if cur:
        out.append(cur)
    return out


def panel_parking(r, im, y):
    """任务五面板：闸机动力学（指令角 vs 实测角）→ 通行记录三段耗时 → 动作日志 → 摆渡车进度。"""
    x0 = MAIN + 12
    y = R.text(im, x0, y, '停车场闸机实时状态', 17, (255, 255, 255))
    y = R.text(im, x0, y, '闸机 %d 台｜通行 %d 条｜动作 %d 条｜累计抬 %d 落 %d' % (
        len(r.gates), r.n_pass, r.n_act,
        sum(int(m.raise_count) for m in r.gates.values()),
        sum(int(m.lower_count) for m in r.gates.values())), 14, (190, 220, 190))
    if not r.gates:
        y = R.text(im, x0, y, '等待 /parking/gate_state 首帧…（管控节点转发插件实测值）',
                   14, (150, 150, 150))
    else:
        y += 5
        y = R.text(im, x0, y, '条目3 闸机动力学（单位：度 / 牛米 / 秒）', 15, (255, 255, 255))
        y = R.text(im, x0, y, '闸机 状态 令源 目标 实测 误差 力矩', 13, (200, 200, 200))
        d = ImageDraw.Draw(im)
        for gid in sorted(r.gates):
            m = r.gates[gid]
            d.rectangle([x0, y + 4, x0 + 12, y + 16],
                        fill=BARRIER_SCOL.get(int(m.state), (180, 180, 180)))
            y = R.text(im, x0 + 18, y, '%-5s %s %s %6.1f %6.1f %+6.2f %6.1f' % (
                gid.replace('gate_', '')[:5], BARRIER_STATE.get(int(m.state), str(m.state)),
                BARRIER_SRC.get(int(m.source), str(m.source)),
                math.degrees(m.target_rad), math.degrees(m.angle_rad),
                math.degrees(m.angle_error_rad), m.effort_nm), 13, (235, 235, 235))
            y = R.text(im, x0 + 18, y, '  地感%s%s｜%s 本次%.2fs｜抬%d 落%d' % (
                '占用 ' if m.loop_occupied else '空闲', m.vehicle_id or '-',
                '到位' if m.at_target else '未到', m.action_elapsed_s,
                m.raise_count, m.lower_count), 13, (170, 200, 230))
            ftxt = BARRIER_FAULT.get(int(m.fault), '')
            if ftxt:
                y = R.text(im, x0 + 18, y, '  故障：%s（线圈 %s）' % (ftxt, m.loop_id),
                           13, (200, 120, 255))
    if r.passes:
        y += 5
        y = R.text(im, x0, y, '条目1/2/4 通行记录（核验/过车/落杆 三段耗时，秒）',
                   15, (255, 255, 255))
        y = R.text(im, x0, y, '编号 方向 车牌 类型 判定 核验 过车 落杆', 13, (200, 200, 200))
        for m in r.passes:
            ok = m.result == ParkingPass.RESULT_ALLOWED
            y = R.text(im, x0, y, '%-7s %s %-8s %s %s %s %s %s' % (
                m.pass_id.split('-gate')[0][:7], PASS_DIR.get(int(m.direction), '?'),
                (m.plate_number or '识别为空')[:8], PASS_KIND.get(int(m.kind), '?'),
                PASS_RESULT.get(int(m.result), '?'),
                pk_sec(pk_dur(m.arrive_s, m.raise_s)),
                pk_sec(pk_dur(m.raise_s, m.clear_s)),
                pk_sec(pk_dur(m.clear_s, m.lower_s))),
                13, (130, 220, 140) if ok else (255, 130, 110))
            if not ok and m.reason:
                for seg in wrap(m.reason, 44)[:2]:
                    y = R.text(im, x0, y, '  ' + seg, 13, (255, 170, 140))
        m = r.passes[-1]
        y = R.text(im, x0, y, '  最新 %s｜%s｜名单%s｜临时卡 %s' % (
            m.gate_id, '已识别' if m.recognized else '未识别',
            WL_NAME.get(int(m.whitelist_hit), '?'), m.ticket_id or '无'),
            13, (170, 200, 230))
        y = R.text(im, x0, y, '  核验图 %s｜到场 t=%.1f 抬杆 t=%.1f' % (
            os.path.basename(m.image_path) or '无', m.arrive_s, m.raise_s),
            13, (170, 200, 230))
    if r.acts:
        y += 5
        y = R.text(im, x0, y, '条目4/5 闸机动作日志（新在前）', 15, (255, 255, 255))
        y = R.text(im, x0, y, '仿真秒 闸机 动作 令源 到位 耗时', 13, (200, 200, 200))
        for m in reversed(r.acts):
            bad = (not m.reached) and int(m.fault) != BarrierState.FAULT_NONE
            y = R.text(im, x0, y, '%7.1f %-4s %s %s %s %5.2f' % (
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                m.gate_id.replace('gate_', '')[:4],
                ACT_NAME.get(int(m.action), str(m.action)),
                BARRIER_SRC.get(int(m.source), '?'), '是' if m.reached else '否',
                m.elapsed_s), 13, (255, 130, 110) if bad else (235, 235, 235))
            if int(m.action) in ACT_NOTE_KEYS and m.note:
                for seg in wrap('%s｜%s %s' % (
                    m.note, m.plate_number or m.vehicle_id or '-',
                    BARRIER_FAULT.get(int(m.fault), '')), 44)[:2]:
                    y = R.text(im, x0, y, '  ' + seg, 13, (200, 120, 255))
    y += 5
    y = R.text(im, x0, y, 'car_6 摆渡车进度（/parking/shuttle_status）', 15, (255, 255, 255))
    if r.shut:
        for seg in wrap(r.shut)[:3]:
            y = R.text(im, x0, y, seg, 13, (250, 200, 120))
    else:
        y = R.text(im, x0, y, '等待摆渡车首帧…', 14, (150, 150, 150))
    return y


def panel(r, im, y):
    if A.mode == 'speed':
        return panel_speed(r, im, y)
    if A.mode == 'parking':
        return panel_parking(r, im, y)
    x0 = MAIN + 12
    if A.mode in ('tl', 'both'):
        y = R.text(im, x0, y, '信号灯实时状态（6 组）', 17, (255, 255, 255))
        y += 4
        for g in GRPS:
            m = r.det.get(g)
            if m is None:
                y = R.text(im, x0, y, '%s 等待首帧…' % g, 15, (150, 150, 150))
                continue
            d = ImageDraw.Draw(im)
            d.rectangle([x0, y + 4, x0 + 12, y + 16],
                        fill=PH_COLOR.get(m.vehicle_phase, (180, 180, 180)))
            d.rectangle([x0 + 16, y + 4, x0 + 28, y + 16],
                        fill=PH_COLOR.get(m.pedestrian_phase, (180, 180, 180)))
            y = R.text(im, x0 + 34, y, '%s %s/%s 余%4.1fs' % (
                g, m.vehicle_phase, m.pedestrian_phase, m.phase_remaining),
                15, (235, 235, 235))
            y = R.text(im, x0 + 34, y, '绿%4.1f 黄%3.1f 红%4.1f  %s' % (
                m.green_duration, m.yellow_duration, m.red_duration,
                m.control_mode), 14, (170, 200, 230))
            y += 2
        if any(g in r.st for g in GRPS):
            y += 6
            y = R.text(im, x0, y, '统计（排队/累计通过/均等待/系数）', 16, (255, 255, 255))
            for g in GRPS:
                m = r.st.get(g)
                if m is None:
                    continue
                y = R.text(im, x0, y, '%-6s %2d %4d %5.1fs %4.2f' % (
                    g, m.queue_length, m.total_vehicles, m.avg_wait_time,
                    m.adaptation_factor), 14, (190, 220, 190))
        if r.adap:
            y += 6
            y = R.text(im, x0, y, '自适应算法节点输出', 16, (255, 255, 255))
            try:
                j = json.loads(r.adap)
                for k, v in list(j.items())[:8]:
                    t = k + '=' + (v if isinstance(v, str) else json.dumps(v, ensure_ascii=False))
                    y = R.text(im, x0, y, t[:44], 13, (170, 200, 230))
            except Exception:
                y = R.text(im, x0, y, r.adap[:60], 13, (170, 200, 230))
    if A.mode in ('plate', 'both'):
        y += 6
        y = R.text(im, x0, y, '卡口抓拍 %d 次 / 识别 %d 条 / 逐字命中 %d' % (
            r.n_cap, r.n_res, r.n_hit), 16, (255, 255, 255))
        rate = 100.0 * r.n_hit / r.n_res if r.n_res else 0.0
        y = R.text(im, x0, y, '识别成功率 %.1f%%（有真值样本内）' % rate, 15, (190, 220, 190))
        if r.algo:      # 来源计数证明结果确实由外部算法节点产出，而非内置识别器
            y = R.text(im, x0, y, '算法来源：' + '  '.join(
                '%s×%d' % (k or '-', v) for k, v in list(r.algo.items())[:4]),
                14, (250, 200, 120))
        y = R.text(im, x0, y, '卡口      车牌号      置信 车速 判定', 14, (200, 200, 200))
        for c in r.plates[-10:]:
            ok = '命中' if c[5] else ('空号' if not c[2] else '不符')
            col = (120, 220, 130) if c[5] else (120, 120, 220)
            y = R.text(im, x0, y, '%-9s %-10s %.2f %4.1f %s' % (
                c[1].replace('gate_', '')[:9], c[2] or '-', c[3], c[4], ok), 14, col)
        if r.alerts:
            y += 6
            y = R.text(im, x0, y, '黑名单告警', 16, (90, 90, 255))
            for lv, pn, gid, cid, why in r.alerts[-4:]:
                y = R.text(im, x0, y, 'L%d %s @%s %s' % (
                    lv, pn, gid.replace('gate_', '')[:8], why)[:42], 13, (90, 90, 255))
    return y


def main():
    rclpy.init()
    r = Rec()
    threading.Thread(target=lambda: rclpy.spin(r), daemon=True).start()
    if any(k != 'ros' for k, _ in SRC.values()):
        threading.Thread(target=puller, args=(r,), daemon=True).start()
    t0 = time.time()
    # 只等相机首帧（面板缺几秒会自己补上，等它会把整段字幕时间轴推歪）
    # 比的是"路数"不是"格子数"：同一画面写两遍（全景+特写）时 r.img 只有一个键，
    # 拿 len(VIEWS) 当目标是永远等不满的，会白等满 20 s 再误报一句"没等到首帧"。
    while time.time() - t0 < 20 and len(r.img) < len(SRC):
        time.sleep(0.3)
    if len(r.img) < len(SRC):
        print('!! %ds 内没等到相机首帧，画面区会显示提示而不是黑屏：%s' % (
            time.time() - t0, [KEY[t] for t in VIEWS if KEY[t] not in r.img]), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(A.out)), exist_ok=True)
    vw = cv2.VideoWriter(A.out, cv2.VideoWriter_fourcc(*'mp4v'), A.fps, (W, H))
    if not vw.isOpened():
        raise SystemExit('VideoWriter 打不开：' + A.out)
    n = 0
    limit = int(A.dur * A.fps)
    slot = (H - HEAD) // max(1, len(VIEWS))
    while n < limit and rclpy.ok():
        tick = time.time()
        canvas = np.zeros((H, W, 3), np.uint8)
        for i, t in enumerate(VIEWS):
            canvas[HEAD + i * slot:HEAD + (i + 1) * slot, 0:MAIN] = \
                fit(crop_to(r.img.get(KEY[t]), BOX.get(t)), MAIN, slot)
        canvas[:, MAIN:] = (24, 26, 30)
        canvas[0:HEAD, :] = (16, 18, 22)
        im = Image.fromarray(canvas)
        ImageDraw.Draw(im).rectangle([MAIN, HEAD, MAIN + 2, H], fill=(70, 70, 70))
        for i, t in enumerate(VIEWS):
            if KEY[t] not in r.img:      # 别把"没收到帧"演成"场景是黑的"
                R.text(im, 20, HEAD + i * slot + slot // 2 - 10,
                       '等待相机画面… %s' % KEY[t], 18, (255, 120, 120))
        R.text(im, 12, 6, A.title[:96], 20, (255, 255, 255))
        el = n / A.fps
        clock = 'sim %7.1fs' % r.simt if r.simt else 'sim  --'
        R.text(im, MAIN + 300, 8, '%4.0f/%.0fs' % (el, A.dur), 15, (200, 200, 200))
        R.text(im, MAIN + 12, 8, clock, 15, (200, 200, 200))
        panel(r, im, HEAD + 10)
        cs = captions()
        if cs:
            box_h = 12 + 20 * len(cs)
            ImageDraw.Draw(im).rectangle([0, H - box_h, MAIN, H], fill=(0, 0, 0))
            yy = H - box_h + 4
            for line in cs:
                R.text(im, 10, yy, line[:118], 16,
                       (90, 90, 255) if line.startswith('!!') else (255, 255, 180))
                yy += 20
        vw.write(cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR))
        n += 1
        time.sleep(max(0.0, 1.0 / A.fps - (time.time() - tick)))
    vw.release()
    # flush 必须在 os._exit 之前：stdout 被驱动脚本重定向到文件时是块缓冲（不是终端的行缓冲），
    # 而 os._exit 不走正常退出、不刷新 stdio —— 这行"已写入 N 帧"就整条留在缓冲区里没了。
    # vid_common.sh 的 vid_wait() 正是 tail /tmp/vid_rec.log 拿这句当收片证据的，
    # 少了它，读片的人只能看到"驱动脚本正常跑完"，看不到帧数。
    print('已写入 %s（%d 帧 / %.1f s，实际耗时 %.0f s）' % (A.out, n, n / A.fps, time.time() - t0),
          flush=True)
    os._exit(0)                                       # spin 线程还在 rclpy 里，硬退避免 core dump


if __name__ == '__main__':
    main()
