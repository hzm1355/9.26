#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务四测速模块取证（被动跟队版）：不动任何车，让车队按自己的环线巡航跑过基线点位。

为什么不用"摆一辆车单独开一趟"的受控版（tools/speed_ctrl_evidence.py）：
  tools/why_move_probe.py 实测，把车摆到车道上之后它会以 ~8 m/s 沿车头方向冲出去、
  撞上东西才停（z 全程不变、cmd_vel 发布者数=0、odom 与位姿微分一致 → 轮子确实在转），
  而同一点位从空中自由落体放下去反而滑一段就停死。也就是说"摆位"这个动作本身会把现场
  搞成不可控状态，受控趟的真值前提（起点静止、车道净空）立不住。
  车队正常巡航时速度由 signal_drive 的 cruise_speed=0.7 m/s 决定，是**已知真值**，
  还能用位姿断面计时独立复核，不需要摆位 —— 所以改成被动观测。

  （后续更新：那条"摆位会把车沿车头弹出 ~8 m/s"的结论只在那种连续跑了 8.8 h 的脏物理态
  里成立；干净时线重启后同一 /set_entity_state 操作实测回弹 0.00 m。受控单趟版本已经重新
  可用，见 tools/speed_live_pass.py，它才是"读数对不对"的主证据，本脚本负责流量统计。）

真值口径（与检测节点完全独立）：本脚本自己订阅 /model_states（~97 Hz），对每个点位的
两条断面做线性插值计时：
  * 瞬时点位：车道轴上 center 前后各 2 m 的通过时刻差 → 4 m 平均速度（节点报的是 0.30 s 窗中位）；
  * 区间点位：entry 圆前沿与 exit 圆前沿的通过时刻差 → 与节点 travel_time_s 直接对表。
判定用"告警位是否等于(真值>阈值)"，不用静态预期 —— 车被红灯排队拖慢时真值本来就会掉到
阈值以下，那种情况下"不告警"才是对的。三段阈值把"阈值真的在起作用"钉死：
  A 基线（2.0/3.0/4.0 混布）→ 同一趟里低阈值点位告警、高阈值点位干净；
  B 全局 99.0 → 同车同速，一条都不许告；
  C 全局 1.0 → 同车同速，通过即告警。
"""
import csv
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats
from v2x_msgs.srv import SetSpeedZone

_T0 = time.monotonic()
OUTDIR = '/tmp/v2x_records'
# 每个阈值阶段的仿真时长（秒）。正式取证用 240（Ring A 一圈 109 s、Ring B 一圈 97 s，
# 240 s 保证每个点位至少被两圈车各通过一次）；PHASE_S=40 可以只验机器管线通不通。
PHASE_S = float(os.environ.get('PHASE_S', '240'))

# 与 speed_zones.yaml 基线同一套几何。axis 是通行方向所在轴，sign 是通行方向
# （+1 沿轴增大 / -1 沿轴减小）；a=断面中心（瞬时=center，区间=entry），b=exit。
ZONES = {
    'SZ-A-west-instant':  dict(mode='instant', axis='y', a=-5.0, r=1.6, sign=+1, base=2.0),
    'SZ-B-north-instant': dict(mode='instant', axis='x', a=9.0, r=1.6, sign=+1, base=4.0),
    'SZ-B-south-instant': dict(mode='instant', axis='x', a=9.0, r=1.6, sign=-1, base=2.0),
    'SZ-A-south-segment': dict(mode='segment', axis='x', a=-5.0, b=-16.0, r=1.6, sign=-1, base=2.0),
    'SZ-B-east-segment':  dict(mode='segment', axis='y', a=2.0, b=-12.0, r=1.6, sign=-1, base=3.0),
}
for _z, _d in ZONES.items():
    if _d['mode'] == 'instant':
        _d['u1'] = _d['a'] - 2.0 * _d['sign']
        _d['u2'] = _d['a'] + 2.0 * _d['sign']
    else:
        # 圆前沿：车心刚进圆的那个轴坐标 = 断面中心 - r*通行方向（与节点 travel_time 同口径）
        _d['u1'] = _d['a'] - _d['r'] * _d['sign']
        _d['u2'] = _d['b'] - _d['r'] * _d['sign']
    _d['span'] = abs(_d['u2'] - _d['u1'])


def median(xs):
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class Fleet(Node):
    def __init__(self):
        super().__init__('speed_fleet_evidence')
        self.sim_t = None
        self.latest = {}                # name -> (x, y)，最近一帧位姿
        self.tick = {}                  # name -> (时钟格时刻, x, y)
        self.speeds = {}                # name -> [(t, 速度 m/s)]，只留最近 60 s
        self.cross = {}                 # zone_id -> name -> [(断面序号 1/2, 时刻)]
        self.caps, self.segs, self.als = [], [], []
        self.stats = {}
        self.stats_round = 0.0
        self.phases = []

        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_subscription(SpeedCapture, '/speed/capture', self.on_cap, 50)
        self.create_subscription(SpeedSegment, '/speed/segment', self.on_seg, 50)
        self.create_subscription(SpeedAlert, '/speed/alert', self.on_al, 50)
        self.create_subscription(SpeedStats, '/speed/stats', self.on_stats, 20)

        self.cli_zone = self.create_client(SetSpeedZone, '/speed/zones')
        self.cli_exp = self.create_client(Trigger, '/speed/export')

    # ------------------------------------------------------------------ 采集
    def on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_t is not None and now > self.sim_t:
            # 速度与断面穿越都按 /clock 格来差分：位姿流 ~97 Hz 而时钟 ~10 Hz。
            # 若拿"相邻两帧位姿之差"去除"一个时钟格"，分子只有 1 ms 的位移、分母是
            # 100 ms，速度会被缩成 1/10（预检就死在这儿：巡航 2.52 km/h 报成 0.25 km/h）；
            # 若只在相邻两帧上做穿越判定，一次穿越落在 9 帧里的概率只有 ~1/9，
            # 独立真值会大面积缺失（同一枚 /clock 量化坑，见 memory「仿真速度差分要 /clock 驱动」）。
            for n, (x, y) in self.latest.items():
                q = self.tick.get(n)
                if q is not None and now > q[0]:
                    dt = now - q[0]
                    lst = self.speeds.setdefault(n, [])
                    lst.append((now, math.hypot(x - q[1], y - q[2]) / dt))
                    while lst and now - lst[0][0] > 60.0:
                        lst.pop(0)
                    if dt <= 1.0:         # 丢帧/复位这一格不做任何计时
                        for zid, z in ZONES.items():
                            u0 = q[1] if z['axis'] == 'x' else q[2]
                            u1 = x if z['axis'] == 'x' else y
                            for which, cv in ((1, z['u1']), (2, z['u2'])):
                                hit = (u0 <= cv < u1) if z['sign'] > 0 else (u1 <= cv < u0)
                                if hit:
                                    f = 0.0 if abs(u1 - u0) < 1e-9 else (cv - u0) / (u1 - u0)
                                    self.cross.setdefault(zid, {}).setdefault(n, []).append(
                                        (which, q[0] + f * dt))
                self.tick[n] = (now, x, y)
        self.sim_t = now

    def t(self):
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - _T0 > 30:
                raise SystemExit('!! 30 s 内没收到 /clock，仿真没在跑')
        return self.sim_t

    def on_states(self, msg):
        # 只缓存最新位姿；一切计时都在 on_clock 上做（见上）
        for n, p in zip(msg.name, msg.pose):
            if n.startswith('car_'):
                self.latest[n] = (p.position.x, p.position.y)

    def on_cap(self, m):
        self.caps.append(m)

    def on_seg(self, m):
        self.segs.append(m)

    def on_al(self, m):
        self.als.append(m)

    def on_stats(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st > self.stats_round:
            self.stats_round = st
            self.stats = {}
        self.stats[m.zone_id] = m

    # ------------------------------------------------------------------ 工具
    def spin_wait(self, dur):
        t0 = self.t()
        while rclpy.ok() and self.t() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, cli, req, who, tries=3):
        t0 = time.monotonic()
        while not cli.wait_for_service(timeout_sec=3.0):
            if not rclpy.ok():
                raise SystemExit('!! %s 服务没了' % who)
            if time.monotonic() - t0 > 60.0:
                raise SystemExit('!! %s 60 s 内没发现服务，节点没起？' % who)
        for i in range(tries):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            r = fut.result()
            if r is not None:
                return r
            print('  !! %s 30 s 无回执，重试（第 %d 次）' % (who, i + 1))
        raise SystemExit('!! %s 连续 %d 次无回执，现场不能继续' % (who, tries))

    def zone(self, **kw):
        req = SetSpeedZone.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        r = self.call(self.cli_zone, req, '/speed/zones')
        print('  [zones] %s' % r.message.splitlines()[0])
        if not r.success:
            raise SystemExit('!! /speed/zones 失败，现场不能继续')
        return r

    # ------------------------------------------------------------------ 真值
    def truth(self, zid, name, t_ev):
        """事件时刻所在那一次通行：断面 1 在前、断面 2 在后 → (真值 km/h, 独立计时 s)。"""
        z = ZONES.get(zid)
        if z is None:
            return None, None
        t1 = t2 = None
        for which, tc in self.cross.get(zid, {}).get(name, []):
            if which == 1 and tc <= t_ev + 1.5:
                t1 = tc
            elif which == 2 and tc >= t_ev - 1.5 and t1 is not None:
                t2 = tc
                break
        if t1 is None or t2 is None or t2 <= t1:
            return None, None
        return z['span'] / (t2 - t1) * 3.6, t2 - t1

    def car_median_kmh(self, name):
        lst = [v for _, v in self.speeds.get(name, [])][-300:]
        return median(lst) * 3.6 if lst else None

    # ------------------------------------------------------------------ 一个观测阶段
    def observe(self, label, hint):
        n0 = (len(self.caps), len(self.segs), len(self.als))
        t0 = self.t()
        print('\n== %s ==' % label)
        self.spin_wait(PHASE_S)
        n1 = (len(self.caps), len(self.segs), len(self.als))
        caps, segs, als = self.caps[n0[0]:n1[0]], self.segs[n0[1]:n1[1]], self.als[n0[2]:n1[2]]
        rows = []
        print('  --- 瞬时抓拍 %d 条（真值=位姿在 center 前后各 2 m 上的 4 m 平均）' % len(caps))
        for m in caps:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            tv, tt = self.truth(m.zone_id, m.vehicle_id, st)
            ok = '告警' if m.over else '正常'
            verdict = self.verdict(tv, m.over, m.limit_kmh)
            print('    %-20s %s 牌=%-8s 雷达%6.2f 位姿%6.2f 峰%6.2f 夹角%5.1f° 阈值%6.2f %s'
                  ' | 独立真值 %-8s %s'
                  % (m.zone_id, m.vehicle_id, m.plate_number or '-', m.speed_kmh,
                     m.speed_pose_kmh, m.speed_peak_kmh, m.beam_angle_deg, m.limit_kmh, ok,
                     ('%.2f' % tv) if tv else '缺', verdict))
            rows.append({'kind': 'capture', 'zone_id': m.zone_id, 'vehicle_id': m.vehicle_id,
                         'plate_number': m.plate_number, 'speed_kmh': round(m.speed_kmh, 3),
                         'speed_pose_kmh': round(m.speed_pose_kmh, 3),
                         'speed_peak_kmh': round(m.speed_peak_kmh, 3),
                         'beam_angle_deg': round(m.beam_angle_deg, 2),
                         'limit_kmh': round(m.limit_kmh, 3), 'over': m.over,
                         'truth_kmh': round(tv, 3) if tv else None,
                         'truth_travel_s': round(tt, 3) if tt else None,
                         'hint': hint.get(m.zone_id), 'verdict': verdict,
                         'sim_ts': round(st, 3)})
        print('  --- 区间结果 %d 条（真值=独立断面计时的 行程/时长）' % len(segs))
        for m in segs:
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            tv, tt = self.truth(m.zone_id, m.vehicle_id, st)
            verdict = self.verdict(tv, m.over, m.limit_kmh)
            print('    %-20s %s 牌=%-8s L=%5.1f 节点t=%6.2f 均速%6.2f 峰%6.2f 阈值%6.2f %s'
                  ' | 独立真值 %-8s（t=%.2f s，差 %+.2f s）%s'
                  % (m.zone_id, m.vehicle_id, m.plate_number or '-', m.length_m,
                     m.travel_time_s, m.avg_kmh, m.max_kmh, m.limit_kmh,
                     ('告警' if m.over else '正常'),
                     ('%.2f' % tv) if tv else '缺', tt or 0.0,
                     (tt - m.travel_time_s) if tt else 0.0, verdict))
            rows.append({'kind': 'segment', 'zone_id': m.zone_id, 'vehicle_id': m.vehicle_id,
                         'plate_number': m.plate_number, 'length_m': round(m.length_m, 3),
                         'travel_time_s': round(m.travel_time_s, 3),
                         'avg_kmh': round(m.avg_kmh, 3), 'max_kmh': round(m.max_kmh, 3),
                         'limit_kmh': round(m.limit_kmh, 3), 'over': m.over,
                         'truth_kmh': round(tv, 3) if tv else None,
                         'truth_travel_s': round(tt, 3) if tt else None,
                         'hint': hint.get(m.zone_id), 'verdict': verdict,
                         'sim_ts': round(st, 3)})
        per = {}
        for a in als:
            per[a.zone_id] = per.get(a.zone_id, 0) + 1
        print('  --- 告警 %d 条：%s' % (len(als),
              ', '.join('%s×%d' % kv for kv in sorted(per.items())) or '无'))
        for a in als[:6]:
            print('      %s %s %s 牌=%s 速度%.2f 阈值%.2f 超%.2f 位置(%.2f,%.2f) 时刻%.2f'
                  % (a.kind, a.zone_id, a.vehicle_id, a.plate_number or '-', a.speed_kmh,
                     a.limit_kmh, a.over_kmh, a.pose.position.x, a.pose.position.y, a.ts))
        by = {}
        for r in rows:
            by.setdefault(r['zone_id'], []).append(r)
        print('  --- 逐点位小结（真值中位 / 告警数 / 判定）')
        for zid in sorted(ZONES):
            rs = by.get(zid, [])
            tvs = [r['truth_kmh'] for r in rs if r['truth_kmh']]
            nov = sum(1 for r in rs if r['over'])
            bad = sum(1 for r in rs if r['verdict'] == 'FAIL')
            print('    %-20s 事件%3d 告警%3d 真值中位 %-8s 基线阈值%5.2f 判定不符%d'
                  % (zid, len(rs), nov,
                     ('%.2f' % median(tvs)) if tvs else '缺', ZONES[zid]['base'], bad))
        fails = sum(1 for r in rows if r['verdict'] == 'FAIL')
        noref = sum(1 for r in rows if r['verdict'] == '无真值')
        print('  本阶段：事件 %d 条，判定不符 %d 条，无独立真值可核 %d 条'
              % (len(rows), fails, noref))
        self.phases.append({'label': label, 'sim_from': round(t0, 1),
                            'sim_to': round(self.t(), 1), 'n_capture': len(caps),
                            'n_segment': len(segs), 'n_alert': len(als),
                            'n_fail': fails, 'n_no_ref': noref,
                            'alerts_by_zone': per, 'rows': rows})
        return fails

    @staticmethod
    def verdict(tv, over, limit):
        if tv is None:
            return '无真值'
        want = tv > limit
        return 'PASS' if want == bool(over) else 'FAIL'


def show_csv(nd):
    ids = set(ZONES)
    for name in ('speed_captures.csv', 'speed_segments.csv', 'speed_alerts.csv'):
        path = os.path.join(OUTDIR, name)
        print('  --- %s' % name)
        if not os.path.exists(path):
            print('      (文件不存在)')
            continue
        with open(path, encoding='utf-8-sig', newline='') as f:
            n = 0
            for r in csv.DictReader(f):
                if r.get('zone_id') not in ids:
                    continue
                n += 1
                if n <= 3:
                    keys = ('capture_id', 'segment_id', 'ref_id', 'zone_id', 'vehicle_id',
                            'plate_number', 'speed_kmh', 'speed_pose_kmh', 'avg_kmh',
                            'travel_time_s', 'length_m', 'limit_kmh', 'over_kmh', 'over',
                            'x', 'y', 'ts')
                    print('      ' + '  '.join('%s=%s' % (k, r[k])
                          for k in keys if k in r and r[k] != ''))
            print('      基线点位行 %d 条' % n)


def main():
    rclpy.init()
    nd = Fleet()
    nd.t()
    print('仿真起于 %.1f s，已见车辆 %d 辆' % (nd.sim_t, len(nd.latest)))

    # 预检：车队必须在真跑（至少两辆车中位速度落在 0.5~5 km/h），否则观测没有意义。
    # 要轮询不能只看一眼：总控刚把 signal_drive 拉起来时 DDS 发现 + 第一轮 cmd_vel 还没
    # 落到车上，只等 20 s 会把"整队 0.25 km/h"当成车队没跑起来而误退出。
    for warm in range(1, 7):
        nd.spin_wait(20.0)
        print('--- 预检（第 %d 次采样）：各车最近 60 s 中位速度（巡航指令 0.70 m/s = 2.52 km/h）---' % warm)
        alive = 0
        for n in sorted(nd.speeds):
            v = nd.car_median_kmh(n)
            print('    %-6s %s' % (n, ('中位 %5.2f km/h' % v) if v else '无速度样本'))
            if v and 0.5 <= v <= 5.0:
                alive += 1
        if alive >= 2:
            break
    else:
        raise SystemExit('!! 预热 120 s 后只有 %d 辆车在正常巡航，车队没跑起来'
                         '（先跑 reset_fleet.sh 并查 /tmp/signal_drive_node.log）' % alive)

    nd.zone(op=SetSpeedZone.Request.OP_CLEAR)
    nd.zone(op=SetSpeedZone.Request.OP_LIST)

    hint_base = {z: ('over' if d['base'] <= 2.0 else 'clean') for z, d in ZONES.items()}
    f1 = nd.observe('阶段 A 基线阈值（2.0 应告警 / 3.0、4.0 应干净）', hint_base)
    nd.zone(op=SetSpeedZone.Request.OP_SET_GLOBAL_LIMIT, limit_kmh=99.0)
    f2 = nd.observe('阶段 B 全局阈值 99.0（同车同速，一条都不许告）',
                    {z: 'clean' for z in ZONES})
    nd.zone(op=SetSpeedZone.Request.OP_SET_GLOBAL_LIMIT, limit_kmh=1.0)
    f3 = nd.observe('阶段 C 全局阈值 1.0（同车同速，通过即告警）',
                    {z: 'over' for z in ZONES})

    # 复位先于打印/导出：上一版把 OP_CLEAR 放在最后，结果 stats 行里一处字段类型写错
    # （vehicles 是 uint32 计数，不是数组）就整段崩在导出之前，现场停在"全局阈值 1.0"
    # 上、JSON 也没落盘。顺序换过来，后面再怎么崩都不影响现场。
    nd.zone(op=SetSpeedZone.Request.OP_CLEAR)

    print('\n--- /speed/stats 全表（最新一轮，窗口 120 s 的末帧）---')
    for zid, m in sorted(nd.stats.items()):
        print('  %-20s mode=%d 阈值%6.2f 窗口%.0f s 流量%3d 车辆%d 均速%6.2f 峰%6.2f '
              '低速%6.2f 超速%d 占比%.2f 累计流量%d 累计超速%d'
              % (zid, m.mode, m.limit_kmh, m.window_s, m.flow, m.vehicles,
                 m.avg_kmh, m.max_kmh, m.min_kmh, m.over_count, m.over_ratio,
                 m.total_flow, m.total_over))
    r = nd.call(nd.cli_exp, Trigger.Request(), '/speed/export')
    print('\n[/speed/export] %s' % r.message)
    show_csv(nd)

    os.makedirs(OUTDIR, exist_ok=True)
    path = os.path.join(OUTDIR, 'speed_fleet_evidence.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'sim_start': nd.phases[0]['sim_from'],
                   'zones': ZONES, 'phases': nd.phases,
                   'fail_total': f1 + f2 + f3}, f, ensure_ascii=False, indent=1)
    print('\nJSON → %s（判定不符合计 %d 条）' % (path, f1 + f2 + f3))
    rclpy.shutdown()
    return 1 if (f1 + f2 + f3) else 0


if __name__ == '__main__':
    sys.exit(main())
