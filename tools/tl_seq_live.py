#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线量灯色时序合规性（任务二 条目1 的量化判据，直接读话题，不需要录制文件）。

条目1 要"灯色切换具备真实时序特性"，肉眼数时间轴不可判，这里给三条硬判据：
  ① 机动车相序只能是 GREEN->YELLOW->RED->GREEN，非法跳变计数必须为 0；
  ② 黄灯每段时长稳定（预设里黄灯固定，不随自适应系数缩放），出现 0 秒段即"黄灯被吞"；
  ③ 行人灯是独立子相位机 WALK->FLASHING->DONT_WALK，且 WALK 期间同组机动车不得为 GREEN。

时间基准：/clock 只有约 10 Hz 且是 BEST_EFFORT，单条边沿直接取"最后一次收到的仿真时刻"
会偶发少算约 1 s（实测把 3.0 s 黄灯量成 2.0 s）。所以边沿一律用单调墙钟打点，
再用整段采样实测的实时倍率（仿真推进/墙钟推进）换算回仿真秒。

用法：python3 tl_seq_live.py [采样秒=45]
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
NEXT_V = {('GREEN', 'YELLOW'), ('YELLOW', 'RED'), ('RED', 'GREEN')}
NEXT_P = {('WALK', 'FLASHING'), ('FLASHING', 'DONT_WALK'), ('DONT_WALK', 'WALK')}
CLOCK_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)


class T(Node):
    def __init__(self, dur):
        super().__init__('tl_seq_live')
        self.dur = dur
        self.t0 = time.monotonic()
        self.sim_t = 0.0
        self.sim_span = [None, 0.0, 0.0]      # [起点仿真时刻, 末点仿真时刻, 末点墙钟]
        self.cur, self.since, self.seeded = {}, {}, set()
        self.ev, self.d = {}, {}
        # 每组的 (墙钟时刻, 车灯, 行人) 快照序列，用于积分"WALK 期间机动车为绿"的时长
        self.snap = {g: [] for g in GROUPS}
        self.create_subscription(
            Clock, '/clock', self.on_clock, CLOCK_QOS)
        for g in GROUPS:
            self.ev[(g, 'V')], self.ev[(g, 'P')] = [], []
            self.d[(g, 'V')], self.d[(g, 'P')] = [], []
            self.create_subscription(String, f'/traffic_light/{g}/state',
                                     lambda m, g=g: self.on(m, g, 'V'), 10)
            self.create_subscription(String, f'/traffic_light/{g}/pedestrian',
                                     lambda m, g=g: self.on(m, g, 'P'), 10)
        self.create_timer(0.5, self.done)

    def on_clock(self, m):
        s = m.clock.sec + m.clock.nanosec * 1e-9
        self.sim_t = s
        if self.sim_span[0] is None:
            self.sim_span[0] = s
        self.sim_span[1], self.sim_span[2] = s, time.monotonic()

    def on(self, msg, g, kind):
        if not msg.data:
            return
        k = (g, kind)
        if self.cur.get(k) == msg.data:
            return
        t = time.monotonic() - self.t0            # 单调墙钟，不受 /clock 滞后影响
        prev = self.cur.get(k)
        # 首个观测到的相位是采样窗口截到的残段，起点不是真实相变时刻，不计入时长统计
        if prev is None:
            pass
        elif k not in self.seeded:
            self.seeded.add(k)
        else:
            self.d[k].append((prev, t - self.since[k]))
        self.ev[k].append((prev if prev is not None else '-', msg.data))
        self.cur[k] = msg.data
        self.since[k] = t
        self.snap[g].append((t, self.cur.get((g, 'V'), ''), self.cur.get((g, 'P'), '')))

    def conflict(self, g, f):
        s, out = self.snap[g], 0.0
        for (t, v, p), (t2, _, _) in zip(s, s[1:]):
            if v == 'GREEN' and p == 'WALK':
                out += (t2 - t) * f
        return out

    def done(self):
        if time.monotonic() - self.t0 < self.dur:
            return
        f = 1.0
        if self.sim_span[0] is not None and self.sim_span[2] > self.t0:
            span_w = self.sim_span[2] - self.t0
            span_s = self.sim_span[1] - self.sim_span[0]
            if span_w > 5:
                f = span_s / span_w
        print('组别      车相变 非法  黄灯各段时长(仿真秒)  行人相变 人非法  WALK撞绿(s)')
        tv = tb = tp = pb = 0
        yset, cf = [], 0.0
        for g in GROUPS:
            ve, pe = self.ev[(g, 'V')], self.ev[(g, 'P')]
            vbad = sum(1 for a, b in ve if a != '-' and (a, b) not in NEXT_V)
            pbad = sum(1 for a, b in pe if a != '-' and (a, b) not in NEXT_P)
            yell = [d * f for ph, d in self.d[(g, 'V')] if ph == 'YELLOW']
            c = self.conflict(g, f)
            yset += yell
            tv += len(ve) - 1
            tb += vbad
            tp += len(pe) - 1
            pb += pbad
            cf += c
            print('%-8s %6d %4d  %-21s %8d %6d %11.1f' % (
                g, len(ve) - 1, vbad,
                '/'.join('%.1f' % y for y in yell[:4]) or '-',
                len(pe) - 1, pbad, c))
        print('合计      %6d %4d                      %8d %6d %11.1f' % (tv, tb, tp, pb, cf))
        if yset:
            print('黄灯时长 min %.2f / max %.2f s，样本 %d 段；实时倍率 %.3f（墙钟×倍率=仿真秒）' % (
                min(yset), max(yset), len(yset), f))
        ok = (tb == 0 and pb == 0 and cf == 0.0 and yset and max(yset) - min(yset) < 0.4)
        print('判据：①机动车相序非法跳变 %d 次 ②黄灯每段稳定 ③行人子相位合法且 WALK 期间无机动车绿放行 -> %s' % (
            tb, '全部满足' if ok else '存在不满足项，见上表'))
        sys.stdout.flush()
        # 打印完直接退出：timer 回调里 shutdown 后 DDS 线程仍会挂着不返回，卡到 timeout 会被判成失败
        os._exit(0)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    rclpy.init()
    n = T(dur)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
