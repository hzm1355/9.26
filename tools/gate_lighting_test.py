#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""光照预设逐个过卡口：条目 3 的正式取证——逆光/弱光/夜间各自真拍一张并识别。

每档只做"切预设 -> 确认到位 -> 等下一张新抓拍 -> 量亮度 + 看识别"这一件事。
亮度分两栏：全图均值证明预设确实改到了相机那条渲染线；牌面均值决定这一档
算不算"拍得下来"——夜里全图必然黑，牌面黑才是问题，所以夜间档要看的是补光灯
有没有把牌面打亮（牌面均值得明显高于全图均值）。识别结论直接进表，
四档跑完就是条目 3 要的对照证据。

用法：python3 gate_lighting_test.py [--both] [预设...]   默认 day backlit lowlight night
      --both  每档等到两个卡口各出一张（消除"这一档恰好拍到南环"的取样偏差）
      位置参数里的纯数字是每档最长等待秒数
"""
import os
import sys
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from v2x_msgs.msg import PlateCapture, PlateResult

PLATES = ['car_1|rear|沪A12345', 'car_3|rear|京B88990', 'car_2|rear|粤Z1A2B3',
          'car_4|rear|苏E66F88', 'car_5|rear|浙A66666']
CN = {'day': '白天', 'backlit': '逆光', 'lowlight': '弱光', 'night': '夜间'}


class LightingTest(Node):
    def __init__(self, presets, per_preset_s, both_gates=False):
        super().__init__('gate_lighting_test')
        self.presets = presets
        self.per = per_preset_s
        # both_gates：每档等到**两个卡口各一张**才算过。只等第一张时，样本会随机
        # 落在南环或北环，而两环的取景角与补光余量本来就不同——受光对照第一轮
        # 就是这么把"北环弱光丢牌"和"南环弱光正常"混进同一档的。
        self.both = both_gates
        self.gates = ['gate_j3_north', 'gate_j2_south']
        self.got = set()
        self.images = {}
        self.rows = []
        self.ack = None
        self.done = False
        self.preset = None
        self.baseline = set()
        self.armed = False

        self.reqPub = self.create_publisher(String, '/plate/request', 10)
        self.lightPub = self.create_publisher(String, '/gate/lighting', 10)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_cap, 50)
        self.create_subscription(PlateResult, '/plate/result', self.on_res, 50)
        self.create_subscription(String, '/gate/lighting/current', self.on_ack, 10)

    def on_cap(self, m):
        self.images[m.capture_id] = m.image_path

    def on_ack(self, m):
        self.ack = m.data

    def on_res(self, r):
        # 只认"切换之后新产生"的抓拍。两个坑都在这道门槛上：
        #   · 识别器只要 35 ms，结果往往比这里发现 capture_id 还快，所以判据用
        #     "不在切换前的基线集合里"，而不是等 target 被赋值；
        #   · /plate/capture 与 /plate/result 都带历史（新订阅者会先收到上一轮的
        #     旧消息），第一档因此会量到**切换前**的帧——实测把夜间余留的暗帧
        #     当成"白天"档的样本，均值只有 4.8。armed 在收到本档回读、算完基线
        #     之后才置位，旧消息一律不进统计。
        if self.done or not self.armed or r.capture_id in self.baseline:
            return
        if self.both:
            if r.gate_id in self.got:
                return                       # 本档这个卡口已经量过了
            self.got.add(r.gate_id)
            self.done = set(self.got) >= set(self.gates)
        else:
            self.done = True
        self.rows.append((self.preset, r.gate_id, r.capture_id,
                          *self.stats(r)))

    def stats(self, r):
        path = self.images.get(r.capture_id, '')
        img = cv2.imread(path) if path else None
        if img is None:
            bright = uniq = roi = hi = -1.0
        else:
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            bright = float(g.mean())
            # 唯一灰度级（按 8 位量化到 32 档）：整幅同色说明这个预设把画面压成纯色
            uniq = float(len(np.unique(g >> 3)))
            # 整幅最亮：夜间档即使识别彻底失败（没有 bbox，牌面均值就量不出来），
            # 这一个数还能说明"画面里到底有没有被照亮的东西"。
            hi = float(g.max())
            # 牌面框内的均值：夜间档全靠这个判断补光灯到没到位。
            # 全图均值在夜里必然是低的（取景里大部分是没照的路段），
            # 牌面区亮了才是"补光有效"，牌面区也黑就是"这档根本拍不到"。
            x, y = int(r.bbox_x), int(r.bbox_y)
            w, h = int(r.bbox_w), int(r.bbox_h)
            if w > 0 and h > 0:
                box = g[max(y, 0):min(y + h, g.shape[0]),
                        max(x, 0):min(x + w, g.shape[1])]
                roi = float(box.mean()) if box.size else -1.0
            else:
                roi = -1.0
        return (bright, uniq, hi, roi, r.bbox_w * r.bbox_h, r.plate_number,
                r.truth_number, r.matched, r.confidence)

    def spin_until(self, pred, timeout):
        t0 = time.time()
        while rclpy.ok() and not pred() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return pred()

    def run(self):
        t0 = time.time()
        while self.reqPub.get_subscription_count() == 0 and time.time() - t0 < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        # 识别器活着才有表可统计。要 spin 着等而不是问一次：刚重启的节点在 DDS 里
        # 还没被发现到，`count_publishers` 会瞬时读到 0（第一轮就被这样误杀过一次，
        # 而 6 秒后它就在正常出结果了）。
        if not self.spin_until(lambda: self.count_publishers('/plate/result') > 0, 15):
            print('!! 15 s 内没发现 /plate/result 的发布者：识别器没跑或已崩。'
                  '先 bash tools/ensure_recognizer.sh 再来')
            return 1
        for p in PLATES:
            self.reqPub.publish(String(data=p))
            time.sleep(0.2)
        print('已下发 %d 个车牌' % len(PLATES))

        for name in self.presets:
            self.preset = name
            self.done = False
            self.armed = False
            self.got = set()
            self.ack = None
            for _ in range(5):
                self.lightPub.publish(String(data=name))
                rclpy.spin_once(self, timeout_sec=0.15)
            if not self.spin_until(lambda: self.ack == name, 5):
                print('!! %s 没有收到 /gate/lighting/current 回读' % name)

            # 切换之后才算数：切换前已在途的抓拍全部记进基线排除掉
            self.baseline = set(self.images)
            self.armed = True
            t1 = time.time()
            while not self.done and time.time() - t1 < self.per:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not self.done:
                miss = [g for g in self.gates if g not in self.got] if self.both else self.gates
                print('%s 档在 %d s 内还缺 %s 的新抓拍' % (name, self.per, '/'.join(miss)))

        print('\n%-8s %-15s %-9s %-7s %-7s %-9s %-8s %-10s %-10s %s' % (
            '预设', '卡口', '全图均值', '灰度级', '最亮', '牌面均值', '牌面px',
            '识别', '真值', '命中/置信'))
        for pr, gate, cid, b, u, hi, roi, px, got, truth, hit, conf in self.rows:
            print('%-8s %-15s %-9.1f %-7.0f %-7.0f %-9.1f %-8d %-10s %-10s %s %.2f (%s)' % (
                CN.get(pr, pr), gate, b, u, hi, roi, px, got or '(空)', truth or '(无)',
                '命中' if hit else '未命中', conf, cid))
        ok = sum(1 for r in self.rows if r[10])
        print('共 %d 张抓拍，逐字命中 %d 张' % (len(self.rows), ok))
        return 0


def main():
    argv = sys.argv[1:]
    both = '--both' in argv
    args = [a for a in argv if not a[0].isdigit() and a != '--both']
    secs = next((int(a) for a in argv if a.isdigit()), 150)
    rclpy.init()
    node = LightingTest(args or ['day', 'backlit', 'lowlight', 'night'], secs, both)
    rc = node.run()
    # 收尾必须把档位拧回 day 并确认：只发一次 day 曾在实测里整轮丢失，让下一条目
    # 在夜间档起跑（识别率、画面全被污染）。这里复用同一套"连发 + 等回读"。
    node.armed = False
    for _ in range(5):
        node.lightPub.publish(String(data='day'))
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.spin_until(lambda: node.ack == 'day', 8):
        for _ in range(5):
            node.lightPub.publish(String(data='day'))
            rclpy.spin_once(node, timeout_sec=0.2)
        node.spin_until(lambda: node.ack == 'day', 8)
    print('收尾档位回读 = %s' % (node.ack,))
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
