#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""条目2c（多路口联动配时）量化取证。

   量两个东西：
   ① 相位差：各组"起绿时刻"相对基准组（默认 J1_ns）的模周期差。
      管理器里 `lt = (now + offset) % cycle`，所以正 offset 让该组相位**提前**，
      实测符号为负（比基准组早起绿）才是配置语义，不要按"晚 3 s"去读。
   ② 反相程度：两两之间"同时为绿"的时长，占各自绿灯总时长的比例。
      complementary / anti_phase_of 的正确判据是这个比例≈0%，
      光看几个瞬时快照根本判不出来。

   为什么不用 `ros2 topic echo --once` 逐组采样：四次串行 echo 每次耗 1~3 s，
   打在表里的 t+2s/t+4s 不是同一时刻，量不出 3 s 级相位差。
   这里所有组在同一条仿真时钟轴上记录跳变，可直接算。

   用法：python3 tl_offset_audit.py [采样秒=90] [基准组=J1_ns]
"""
import itertools
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


class Rec(Node):
    def __init__(self, dur):
        super().__init__('tl_offset_audit')
        self.dur = dur
        self.t0 = time.monotonic()
        self.sim_t = 0.0
        self.cur = {}
        self.seen = set()
        self.green = {g: [] for g in GROUPS}     # 每段 GREEN 的 (start, end)
        self.opened = {}
        self.create_subscription(
            Clock, '/clock',
            lambda m: setattr(self, 'sim_t', m.clock.sec + m.clock.nanosec * 1e-9),
            CLOCK_QOS)
        for g in GROUPS:
            self.create_subscription(String, f'/traffic_light/{g}/state',
                                     lambda m, g=g: self.on(g, m.data), 10)
        self.create_timer(0.2, self.finish)

    def on(self, gid, phase):
        if gid not in self.seen:          # 首帧只用来建立基线
            self.seen.add(gid)
            self.cur[gid] = phase
            return
        if self.cur.get(gid) == phase:
            return
        self.cur[gid] = phase
        if phase == 'GREEN':
            self.opened[gid] = self.sim_t
        elif gid in self.opened:
            self.green[gid].append((self.opened.pop(gid), self.sim_t))

    def finish(self):
        if time.monotonic() - self.t0 < self.dur:
            return
        for g in GROUPS:                          # 收尾未闭合的绿段
            if g in self.opened:
                self.green[g].append((self.opened[g], self.sim_t))
        report(self.green, self.sim_t, REF)
        rclpy.shutdown()


def median(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else 0.0


def report(green, sim_t, ref):
    cyc = {}
    for g, segs in green.items():
        starts = [s for s, _ in segs]
        gaps = [b - a for a, b in zip(starts, starts[1:])]
        # 只取 >= 绿+黄+红 的最小间隔：段数少时首段可能被窗口截断
        cyc[g] = median([x for x in gaps if x >= 5.0] or gaps)
    print(f'  采样窗口：仿真时钟 {sim_t:.1f} s 基准组={ref}')
    print('  组别      GREEN 段数  周期中位(s)  平均绿长(s)  相对基准起绿相位差(s)')
    base = green.get(ref) or []
    for g in GROUPS:
        segs = green[g]
        if not segs or not base:
            print(f'  {g:<9} {len(segs):>6}    -            -             (无数据)')
            continue
        c = cyc[g] or 1.0
        d = _phase_delta(segs[0][0], base[0][0], c)
        avg_g = sum(e - s for s, e in segs) / len(segs)
        print(f'  {g:<9} {len(segs):>6}    {c:6.2f}      {avg_g:6.2f}        {d:+6.2f}'
              f'   （{"比基准早" if d < 0 else "比基准晚"} {abs(d):.2f}s 起绿）')

    print('\n  —— 两两"同时为绿"重叠（反相类的联动关系应≈0）——')
    print('    组别对                     绿A(s)  绿B(s)  同时为绿(s)  占比')
    for x, y in itertools.combinations(GROUPS, 2):
        gx = sum(e - s for s, e in green[x])
        gy = sum(e - s for s, e in green[y])
        ov = sum(max(0.0, min(e1, e2) - max(s1, s2))
                 for s1, e1 in green[x] for s2, e2 in green[y])
        pct = 100 * ov / max(min(gx, gy), 1e-9)
        print(f'    {x} vs {y:<12} {gx:6.1f}  {gy:6.1f}   {ov:8.2f}   {pct:5.1f}%')


def _phase_delta(t, t_ref, cycle):
    """把两组起绿时刻之差折算到 (-cycle/2, +cycle/2]，正=晚于基准，负=早于基准。"""
    d = (t - t_ref) % cycle
    if d > cycle / 2:
        d -= cycle
    return d


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    global REF
    REF = sys.argv[2] if len(sys.argv) > 2 else 'J1_ns'
    rclpy.init()
    n = Rec(dur)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


REF = 'J1_ns'
main()
