#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""灯色真实时序取证：按仿真时钟打印各信号组 机动车/行人 灯态跳变时间轴与相位持续。

时间基准：/clock 约 10 Hz 且 BEST_EFFORT，直接拿"最后一次收到的仿真时刻"给边沿计时会偶发少算
约 1 s（曾把 3.0 s 黄灯量成 2.0 s）。所以相变时刻取仿真时钟（用于对表），相位持续用单调墙钟差
再乘整段实测的实时倍率换算回仿真秒。

用法：python3 tl_timeline.py [采样秒=60]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

GROUPS = ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew']
CLOCK_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)


class T(Node):
    def __init__(self, dur):
        super().__init__('tl_timeline_probe')
        self.dur = dur
        self.t0 = time.monotonic()
        self.sim_t = 0.0
        self.sim_first = self.sim_last = self.wall_last = None
        self.cur, self.since = {}, {}
        self.ev = []          # [(打印用的仿真时刻, 行文本, 待换算的墙钟时长或 None)]
        self.create_subscription(Clock, '/clock', self.on_clock, CLOCK_QOS)
        for g in GROUPS:
            self.create_subscription(String, f'/traffic_light/{g}/state',
                                     lambda m, g=g: self.on(m, g, 'V'), 10)
            self.create_subscription(String, f'/traffic_light/{g}/pedestrian',
                                     lambda m, g=g: self.on(m, g, 'P'), 10)
        self.create_timer(0.5, self.check_done)

    def on_clock(self, m):
        s = m.clock.sec + m.clock.nanosec * 1e-9
        self.sim_t = s
        if self.sim_first is None:
            self.sim_first = s
        self.sim_last, self.wall_last = s, time.monotonic()

    def check_done(self):
        if time.monotonic() - self.t0 >= self.dur:
            f = 1.0
            if self.sim_first is not None and self.wall_last is not None:
                w = self.wall_last - self.t0
                if w > 5:
                    f = (self.sim_last - self.sim_first) / w
            for t, line, wd in self.ev:
                print(line % (wd * f) if wd is not None else line + '  (首个观测，非完整相位)',
                      flush=True)
            print(f'--- 共 {len(self.ev)} 次灯态跳变，采样 {self.dur:.0f}s'
                  f'（仿真时钟 {self.sim_t:.1f}s，实时倍率 {f:.3f}）---', flush=True)
            rclpy.shutdown()

    def on(self, msg, g, kind):
        k = (g, kind)
        if self.cur.get(k) == msg.data:
            return
        t = self.sim_t if self.sim_t > 0 else time.monotonic() - self.t0
        tw = time.monotonic() - self.t0
        base = (f'{t:8.2f}s  {g:<7} {"机动车" if kind == "V" else "行人   "} '
                f'{self.cur.get(k, "-"):<10} -> {msg.data:<10}')
        self.ev.append((t, base + '  (上一相位持续 %.1fs)' if k in self.since else base,
                        tw - self.since[k] if k in self.since else None))
        self.cur[k] = msg.data
        self.since[k] = tw


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
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
