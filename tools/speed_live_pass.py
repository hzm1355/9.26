#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务四条目 1/2/3 + #60 护栏的**零瞬移受控取证**。

为什么不再"摆一辆车到起点然后开过去"：#61 实测每次 `/set_entity_state` 都会给车一个沿
车头方向 ~8 m/s 的穿透回弹初速（旧脏物理态里弹 17~20 m），摆位本身就是复位风暴的源头；
而弹射段的位移不是"行驶"，速度真值当场不存在。所以这一版全程只发 `cmd_vel`：
**读取 car_4 的实际出生位姿 → 把点位下发到它真正所在的车道与航向上 → 让它自己开过去**，
回起点靠"先摘掉点位再掉头北行"，不靠瞬移。

走廊：Ring B 东腿 x=16.8 的直道（北路 y=4.6 与南路 y=-14.6 之间），car_4 出生在
(16.80,-4.99)、航向 -90°（车头朝南）。圆心全部压在车道线上、半径 1.0 m：

    入口断面  entry (16.8,-6.6)  —— 车从 -5.0 南行，y=-5.6 进圈（区间开始计时）
    瞬时点位  center(16.8,-9.1)  —— y=-8.1 进圈，y=-10.1 出圈时按"最近点"出抓拍
    出口断面  exit  (16.8,-11.6) —— y=-10.6 进圈时出区间均速（L=5.0 m）
    停车线    y=-12.8            —— 离南路口还留 1.8 m，不进路口

一趟南向同时产出"瞬时抓拍 + 区间均速"。三趟：

  第 1 趟 阈值 1.5 km/h（真值 ~2.5）→ 两条事件都必须 over=True，各带一条告警。
         阈值不写成 2.0：雷达噪声 σ=0.3 km/h，2.0 距真值只有 1.7σ，实测一抽 -0.53
         就把"必告警"翻成 over=False —— 那是噪声不是缺陷，留 3σ 以上余量才站得住。
  第 2 趟 阈值 99.0 km/h（同一条道、同一个速度、同一个方向）→ 两条事件都必须 over=False
         且零告警。"界面告警是阈值驱动的"于是落在同一份数据里，自证。
  第 3 趟 阈值 1.5，#60 跳变护栏专项，A/B 两段：
    A 段开到 y=-9.6（区间在途 + 瞬时 Encounter 在途）→ 瞬移 8 m 到 y=-1.6。
      节点侧 `hypot(窗口位移) > 5 m` 命中"复位/瞬移"分支：在途瞬时与区间全部作废，
      日志 '本次通行作废' 计数 +1，A 段不许出任何事件（改造前这里会冒出几十 km/h
      的假告警和一个假峰值）。回弹滑行若连续几帧超 12 km/h，还会累加 '超物理上限'。
    B 段从 y=-1.6 重新起一趟：y=-7.0 时再瞬移 1.2 m（仍在入口圆内、还没进瞬时圈）。
      小跳变**不该**命中 >5 m 的作废分支；而节点用的是"子区间速度取中位数"，
      单格跳变（0.1 s 里 1.2 m = 43 km/h）是最大那一格，取中位就把它筛掉了 ——
      所以 B 段必须照常出抓拍与区间，且读数仍是真实巡航速度，全程不出现 >12 km/h 的数。

真值由本脚本自己算，不复用节点的差分：`/model_states` ~97 Hz 全频缓存，
瞬时真值 = 几何最近点前后各 ±0.35 s 的弦长/时长；区间真值 = 高频轨迹线性插值出的两次
越断面时刻之差（节点的边沿取在 10 Hz 的 /clock 格上，±0.05 s；这里 ±0.01 s）。
时长一律换算到仿真秒：gz 的 RTF 不是严格 1.0（实测 ~0.97），从同一趟轨迹的
(墙钟, 仿真钟) 两端回归出来，免得把 3% 的系统偏差当成读数误差。

前置（tools/restart_clean_stack.sh 已经做好）：gzserver 刚重启、signal_drive 不在场、
car_4 停在 RingB 东腿这条道上、五副车牌已重贴。开局硬性核对这几项，缺一项就停跑 ——
"零事件"和"没有车超速"在探针里长得一样。车若停在走廊南端（上一趟的终点），脚本会自己
开回起点，不需要摆位。

用法：python3 speed_live_pass.py [输出json=/tmp/v2x_records/speed_live_pass.json]
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, Vector3
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats
from v2x_msgs.srv import SetSpeedZone

CAR = 'car_4'
X_LANE = 16.8
Z_SPAWN = 6.70
MS2KMH = 3.6

INST_ID = 'SZ-LIVE-instant'
SEG_ID = 'SZ-LIVE-segment'
INST_Y = -9.1
ENTRY_Y = -6.6
EXIT_Y = -11.6
RADIUS = 1.0
RADAR_X = 19.4                 # 雷达杆在车道东侧 2.6 m，波束轴 beam=-90 与通行方向重合
Y_STOP = -12.8
Y_NORTH_BACK = -4.6           # 回程终点（留 0.4 m 余量给入口圆外缘 -5.6）
V_CMD = 0.70                  # 2.52 km/h，车队巡航同款
LIMIT_OVER = 1.5              # 低于真值 -> 必告警
LIMIT_CLEAN = 99.0            # 高于任何读数 -> 必干净
BEAM_AUTO = 999.0             # SetSpeedZone 的"按通行方向自动取"哨兵；字段缺省是 0.0，
#                               而 0.0 是一个真角度（+x 轴）—— 省略它南向车会被判成
#                               "横向离开波束"，radial 恒被 max(0,·) 夹成 0。

JUMP_A_Y = -9.6               # A 段注入 8 m 瞬移的位置（两个判定都还没出结果）
JUMP_A_M = 8.0
JUMP_B_Y = -7.0               # B 段注入 1.2 m 瞬移的位置（区间已开、瞬时未进圈）
JUMP_B_M = 1.2

K_Y = 0.10                    # 横向误差 -> 角速度
K_H = 0.90                    # 航向误差 -> 角速度
A_CAP = 0.35
TURN_RATE = 0.45
YAW_S = -math.pi / 2          # 车头朝南

GUARD_LOG = '/tmp/speed_detect_node.log'
MARKER_FRAME = '超物理上限'    # 帧级护栏（speed_detect_node.py 的 warn）
MARKER_DROP = '本次通行作废'   # 在途通行作废
MAX_SANE = 12.0               # 与 speed_zones.yaml 的 max_sane_kmh 同值

YAML_EAST_SEG = 'SZ-B-east-segment'   # YAML 基线里跨过本走廊的点位，先摘掉


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Runner(Node):

    def __init__(self, outjson):
        super().__init__('speed_live_pass')
        self.outjson = outjson
        self.latest = {}          # name -> (x, y, yaw, z)
        self.sim_t = 0.0
        self.trace = []           # 本趟 car_4 高频轨迹 (wall, sim, x, y)
        self.recording = False
        self.caps, self.segs, self.als = [], [], []
        self.stats = {}           # zone_id -> 最近一条统计帧
        self.holding = True       # True 时 20 Hz 灌零速锁住五辆车
        self.plates = {}
        self.results = []
        self._seg_path = None
        self._seg_jump = None

        self.hold = {}
        for n in ('car_1', 'car_2', 'car_3', 'car_4', 'car_5'):
            self.hold[n] = self.create_publisher(Twist, '/%s/cmd_vel' % n, 10)
        self.create_timer(0.05, self.on_hold)
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_subscription(SpeedCapture, '/speed/capture', self.on_cap, 50)
        self.create_subscription(SpeedSegment, '/speed/segment', self.on_seg, 50)
        self.create_subscription(SpeedAlert, '/speed/alert', self.on_al, 50)
        self.create_subscription(SpeedStats, '/speed/stats', self.on_stats, 20)
        self.create_subscription(String, '/plate/number', self.on_plate, 10)

        self.cli_zone = self.create_client(SetSpeedZone, '/speed/zones')
        self.cli_put = self.create_client(SetEntityState, '/set_entity_state')
        self.cli_exp = self.create_client(Trigger, '/speed/export')

    # ------------------------------------------------------------------ 采集
    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_states(self, msg):
        w = time.monotonic()
        for n, p in zip(msg.name, msg.pose):
            if not n.startswith('car_'):
                continue
            self.latest[n] = (p.position.x, p.position.y,
                              yaw_of(p.orientation), p.position.z)
            if n == CAR and self.recording:
                self.trace.append((w, self.sim_t, p.position.x, p.position.y))

    def on_cap(self, m):
        self.caps.append(m)

    def on_seg(self, m):
        self.segs.append(m)

    def on_al(self, m):
        self.als.append(m)

    def on_stats(self, m):
        # 每个点位只留最后一帧：OP_REMOVE 之后节点不再发它，按轮次整表换新会把它抖没。
        self.stats[m.zone_id] = m

    def on_plate(self, m):
        parts = m.data.split('|')
        if len(parts) >= 2:
            self.plates[parts[0]] = parts[1]

    def on_hold(self):
        """20 Hz 灌零速：gz 的 diff_drive 在 cmd_vel_timeout 0.3 s 之后轮子是自由的，
        别车被顶一下就会飘进走廊撞上测试车，所以除了正在开的那辆全程锁住。"""
        for n, pub in self.hold.items():
            if n == CAR and not self.holding:
                continue
            pub.publish(Twist())

    # ------------------------------------------------------------------ 工具
    def spin_wait(self, dur):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, cli, req, who, tries=2):
        t0 = time.monotonic()
        while not cli.wait_for_service(timeout_sec=3.0):
            if not rclpy.ok():
                raise SystemExit('!! %s 服务没了' % who)
            print('  ... 等 %s 服务发现（%.0f s）' % (who, time.monotonic() - t0))
            if time.monotonic() - t0 > 60.0:
                raise SystemExit('!! %s 60 s 内没发现' % who)
        for i in range(tries):
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            r = fut.result()
            if r is not None:
                return r
            print('  !! %s 30 s 无回执，重试（第 %d 次）' % (who, i + 1))
        raise SystemExit('!! %s 连续 %d 次无回执' % (who, tries))

    def zone(self, **kw):
        req = SetSpeedZone.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        r = self.call(self.cli_zone, req, '/speed/zones')
        print('  [zones] %s' % r.message)
        if not r.success:
            raise SystemExit('!! /speed/zones op=%d 失败：%s' % (kw.get('op'), r.message))
        return r

    def zone_soft(self, **kw):
        """允许失败的 op（例如摘一个可能不存在的 YAML 点位）。"""
        req = SetSpeedZone.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        r = self.call(self.cli_zone, req, '/speed/zones')
        print('  [zones] %s' % r.message)
        return r

    def guard_count(self, marker):
        try:
            with open(GUARD_LOG, encoding='utf-8', errors='replace') as f:
                return sum(1 for ln in f if marker in ln)
        except OSError:
            return -1

    # -------------------------------------------------------------- 车辆状态
    def pose(self):
        p = self.latest.get(CAR)
        if p is None:
            raise SystemExit('!! /model_states 里没有 %s' % CAR)
        return p

    def still(self, dur=3.0, thresh=0.25, name=CAR):
        a = self.latest.get(name)
        self.spin_wait(dur)
        b = self.latest.get(name)
        if a is None or b is None:
            return False
        return math.hypot(b[0] - a[0], b[1] - a[1]) < thresh

    def corridor_clear(self):
        bad = []
        for n, p in self.latest.items():
            if n == CAR or not n.startswith('car_'):
                continue
            if abs(p[0] - X_LANE) < 3.0 and -14.5 < p[1] < -3.5:
                bad.append('%s(%.1f,%.1f)' % (n, p[0], p[1]))
        return bad

    # ------------------------------------------------------------------ 驾驶
    def cmd(self, v, w):
        self.hold[CAR].publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))

    def turn_to(self, yaw_des, budget=45.0):
        self.holding = False
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < budget:
            rclpy.spin_once(self, timeout_sec=0.02)
            err = wrap(yaw_des - self.pose()[2])
            if abs(err) < 0.06:
                break
            self.cmd(0.0, TURN_RATE * (1.0 if err > 0 else -1.0))
        self.cmd(0.0, 0.0)
        self.holding = True
        self.spin_wait(1.2)
        print('  转向：航向 %.1f°（目标 %.1f°）' % (
            math.degrees(self.pose()[2]), math.degrees(yaw_des)))

    def drive_to(self, y_stop, yaw_des, budget=90.0, v=V_CMD, hook=None):
        """定速 + 横向/航向闭环，把车沿 x=X_LANE 开到 y_stop（越过即停）。
        车头与行进方向相反时自动倒车（SDF 的 diff_drive 没设速度上限，允许负值）。
        hook(y) 每拍调用一次，返回 True 表示本趟提前结束。"""
        self.holding = False
        southbound = y_stop < self.pose()[1]
        travel = -math.pi / 2 if southbound else math.pi / 2
        vr = v if abs(wrap(yaw_des - travel)) < math.pi / 2 else -v
        t0 = time.monotonic()
        hit = False
        while rclpy.ok() and time.monotonic() - t0 < budget:
            rclpy.spin_once(self, timeout_sec=0.02)
            x, y, yaw, _z = self.pose()
            # hook 必须排在"到达"判定之前：注入点常等于本段的目标 y，
            # 先判到达就永远进不了 hook（上一轮 A 段的瞬移就是这么没打出去的）。
            if hook is not None and hook(y):
                break
            if (southbound and y <= y_stop) or (not southbound and y >= y_stop):
                hit = True
                break
            w = K_Y * (X_LANE - x) + K_H * wrap(yaw_des - yaw)
            self.cmd(vr, max(-A_CAP, min(A_CAP, w)))
        self.cmd(0.0, 0.0)
        self.holding = True
        self.spin_wait(1.5)
        if not hit and hook is None:
            print('  !! %.0f s 没开到 y=%.1f（当前 y=%.2f）' % (budget, y_stop,
                                                                self.pose()[1]))
        return hit

    def return_north(self, target=Y_NORTH_BACK):
        """回程不用瞬移：摘掉点位（免得北行时误触发反向边沿）-> 掉头朝北 ->
        回到起点 -> 掉头朝南。"""
        self.zone_soft(op=SetSpeedZone.Request.OP_REMOVE, zone_id=INST_ID)
        self.zone_soft(op=SetSpeedZone.Request.OP_REMOVE, zone_id=SEG_ID)
        self.turn_to(math.pi / 2)
        self.drive_to(target, math.pi / 2, budget=70.0)
        self.turn_to(YAW_S)

    # ------------------------------------------------------------------ 真值
    def rtf(self):
        tr = self.trace
        if len(tr) < 30 or tr[-1][0] - tr[0][0] < 2.0:
            return 1.0
        dw = tr[-1][0] - tr[0][0]
        ds = tr[-1][1] - tr[0][1]
        return ds / dw if dw > 0 and ds > 0 else 1.0

    def chord_kmh(self, wall_t, half=0.35):
        """wall_t 前后各 ±half 秒的弦长速度（换算到仿真秒口径）。"""
        a = b = None
        for s in self.trace:
            if a is None and s[0] >= wall_t - half:
                a = s
            if s[0] <= wall_t + half:
                b = s
        if a is None or b is None or b[0] - a[0] < 0.2:
            return None
        dist = math.hypot(b[2] - a[2], b[3] - a[3])
        dt = (b[0] - a[0]) * self.rtf()
        return dist / dt * MS2KMH if dt > 1e-6 else None

    def cross_wall(self, y_line):
        """高频轨迹里线性插值出 y 第一次向下越过 y_line 的墙钟时刻。"""
        tr = self.trace
        for i in range(1, len(tr)):
            y0, y1 = tr[i - 1][3], tr[i][3]
            if y0 > y_line >= y1:
                if abs(y1 - y0) < 1e-9:
                    return tr[i][0]
                f = (y_line - y0) / (y1 - y0)
                return tr[i - 1][0] + f * (tr[i][0] - tr[i - 1][0])
        return None

    def path_len(self, w0, w1):
        """区间内实走折线长度，按"相邻两拍的距离"分段累计：>0.5 m 的那几段是本脚本自己
        注入的瞬移（护栏趟才有），单独计成 jump —— 不拆开的话"实走 7.46 m / 断面间距
        5.0 m"会被读成车在斜穿，其实多出来的 1.2 m 是一次瞬移的落点差。"""
        drive = jump = 0.0
        prev = None
        for s in self.trace:
            if s[0] < w0 or s[0] > w1:
                continue
            if prev is not None:
                d = math.hypot(s[2] - prev[0], s[3] - prev[1])
                if d > 0.5:
                    jump += d
                else:
                    drive += d
            prev = (s[2], s[3])
        return round(drive, 3), round(jump, 3)

    def truth_instant(self):
        """几何最近点（到瞬时圆心距离最小的那一拍）前后 ±0.35 s 的弦速。"""
        if len(self.trace) < 20:
            return None
        best, d = None, 1e9
        for s in self.trace:
            dd = math.hypot(s[2] - X_LANE, s[3] - INST_Y)
            if dd < d:
                d, best = dd, s
        if best is None or d > RADIUS:
            return None
        return self.chord_kmh(best[0])

    def truth_segment(self):
        """真值口径与节点一致（直线 L=5.0 m / 两次进断面时刻差），差别只在边沿时刻：
        节点取在 10 Hz 的 /clock 格上（±0.05 s），这里用 97 Hz 轨迹线性插值（±0.01 s）。
        另附实走折线长度，用来证明车没有斜穿走廊。"""
        w0 = self.cross_wall(ENTRY_Y + RADIUS)
        w1 = self.cross_wall(EXIT_Y + RADIUS)
        if w0 is None or w1 is None or w1 - w0 < 0.2:
            return None, None
        dt = (w1 - w0) * self.rtf()
        drive, jump = self.path_len(w0, w1)
        self._seg_jump = jump
        return abs(EXIT_Y - ENTRY_Y) / dt * MS2KMH, drive

    # ------------------------------------------------------------------ 点位
    def publish_zones(self, limit):
        """把点位下发到车真正所在的车道（x=X_LANE）与通行方向（南向 beam=-90）上。
        OP_ADD 会顺带 reset_state()：清掉上一趟的在途累积与统计，并按当前位置重建
        圈内外标志 —— 此刻车静止在 y=-4.6~-5.0，距入口圆心 1.6~2.0 m（>半径 1.0），
        所以重建后不会把"起点已在圈内"当成已经越过断面。"""
        p = self.pose()
        if abs(p[1] - ENTRY_Y) < RADIUS + 0.4:
            raise SystemExit('!! 起点 y=%.2f 离入口断面圆心太近（需 >%.1f m），'
                             'reset 后的种子会把"已在圈内"当成已经越过' % (
                                 p[1], RADIUS + 0.4))
        self.zone(op=SetSpeedZone.Request.OP_ADD, zone_id=INST_ID, mode=0,
                  limit_kmh=limit, radius=RADIUS,
                  center_x=X_LANE, center_y=INST_Y,
                  radar_x=RADAR_X, radar_y=INST_Y, beam_deg=-90.0,
                  road='RingB 东腿（受控单跑）')
        self.zone(op=SetSpeedZone.Request.OP_ADD, zone_id=SEG_ID, mode=1,
                  limit_kmh=limit, radius=RADIUS,
                  entry_x=X_LANE, entry_y=ENTRY_Y, exit_x=X_LANE, exit_y=EXIT_Y,
                  beam_deg=BEAM_AUTO,
                  road='RingB 东腿（受控单跑）')

    def events_since(self, n_cap, n_seg, n_al):
        return self.caps[n_cap:], self.segs[n_seg:], self.als[n_al:]

    def snap_stats(self, wait=True):
        """/speed/stats 是 10 s 周期发布的，事件刚落地时最近那一帧还是旧的（上一轮实测
        flow=0）—— 多等一格再取。"""
        if wait:
            self.spin_wait(12.0)
        return {z: dict(flow=m.flow, vehicles=m.vehicles, avg_kmh=round(m.avg_kmh, 3),
                        max_kmh=round(m.max_kmh, 3), min_kmh=round(m.min_kmh, 3),
                        over_count=m.over_count, total_flow=m.total_flow,
                        total_over=m.total_over, window_s=m.window_s,
                        limit_kmh=round(m.limit_kmh, 3))
                for z, m in self.stats.items() if z in (INST_ID, SEG_ID)}

    def fmt_events(self, caps, segs, truth_inst, truth_seg):
        rows = []
        for c in caps:
            r = {'kind': 'instant', 'id': c.capture_id, 'zone': c.zone_id,
                 'vehicle': c.vehicle_id, 'plate': c.plate_number,
                 'radar_kmh': round(c.speed_kmh, 3),
                 'pose_kmh': round(c.speed_pose_kmh, 3),
                 'peak_kmh': round(c.speed_peak_kmh, 3),
                 'limit_kmh': round(c.limit_kmh, 3), 'over': bool(c.over),
                 'over_kmh': round(c.over_kmh, 3),
                 'beam_angle_deg': round(c.beam_angle_deg, 2),
                 'radar_range_m': round(c.radar_range_m, 2),
                 'noise_kmh': round(c.noise_kmh, 3),
                 'ts': c.header.stamp.sec + c.header.stamp.nanosec * 1e-9,
                 'pos': [round(c.pose.position.x, 2), round(c.pose.position.y, 2)],
                 'truth_kmh': None if truth_inst is None else round(truth_inst, 3)}
            rows.append(r)
            print('  瞬时 %s %s 牌=%s 雷达%.2f 位姿%.2f 峰%.2f 阈值%.2f over=%-5s '
                  '夹角%.1f° 斜距%.2fm 噪声%+.2f | 本脚本真值 %s（差 %s）' % (
                      r['zone'], r['vehicle'], r['plate'] or '-', r['radar_kmh'],
                      r['pose_kmh'], r['peak_kmh'], r['limit_kmh'], r['over'],
                      r['beam_angle_deg'], r['radar_range_m'], r['noise_kmh'],
                      '-' if truth_inst is None else '%.2f' % truth_inst,
                      '-' if truth_inst is None else '%+.2f' % (
                          r['radar_kmh'] - truth_inst)))
        for s in segs:
            r = {'kind': 'segment', 'id': s.segment_id, 'zone': s.zone_id,
                 'vehicle': s.vehicle_id, 'plate': s.plate_number,
                 'travel_time_s': round(s.travel_time_s, 3),
                 'length_m': round(s.length_m, 3), 'avg_kmh': round(s.avg_kmh, 3),
                 'limit_kmh': round(s.limit_kmh, 3), 'over': bool(s.over),
                 'over_kmh': round(s.over_kmh, 3), 'max_kmh': round(s.max_kmh, 3),
                 'entry_speed_kmh': round(s.entry_speed_kmh, 3),
                 'exit_speed_kmh': round(s.exit_speed_kmh, 3),
                 'truth_avg_kmh': None if truth_seg is None else round(truth_seg, 3),
                 'truth_path_len_m': self._seg_path,
                 'truth_jump_len_m': self._seg_jump}
            rows.append(r)
            print('  区间 %s %s 牌=%s L=%.1fm t=%.3fs 均速%.2f 峰%.2f 阈值%.2f over=%-5s'
                  ' | 高频插值真值 %s（差 %s，实走折线 %s m + 瞬移 %s m）' % (
                      r['zone'], r['vehicle'], r['plate'] or '-', r['length_m'],
                      r['travel_time_s'], r['avg_kmh'], r['max_kmh'], r['limit_kmh'],
                      r['over'], '-' if truth_seg is None else '%.2f' % truth_seg,
                      '-' if truth_seg is None else '%+.2f' % (r['avg_kmh'] - truth_seg),
                      r['truth_path_len_m'], r['truth_jump_len_m']))
        return rows

    def fmt_alerts(self, als):
        for a in als:
            print('  告警 kind=%d(%s) ref=%s %s 牌=%s 判定%.2f 阈值%.2f 超限%.2f '
                  '@%.2fs 位置(%.2f,%.2f) note=%r' % (
                      a.kind, '瞬时' if a.kind == 0 else '区间', a.ref_id, a.vehicle_id,
                      a.plate_number or '-', a.speed_kmh, a.limit_kmh, a.over_kmh,
                      a.ts, a.pose.position.x, a.pose.position.y, a.note))

    # ---------------------------------------------------------- 第 1/2 趟
    def pass_normal(self, label, limit):
        print('\n== %s（阈值 %.1f km/h，指令 %.2f m/s = %.2f km/h，起点 y=%.2f）==' % (
            label, limit, V_CMD, V_CMD * MS2KMH, self.pose()[1]))
        bad = self.corridor_clear()
        if bad:
            raise SystemExit('!! 走廊上有别的车：%s' % ', '.join(bad))
        if not self.still():
            raise SystemExit('!! 起步前 %s 没停稳，真值不成立' % CAR)
        self.publish_zones(limit)

        n = (len(self.caps), len(self.segs), len(self.als))
        self.trace = []
        self.recording = True
        self.drive_to(Y_STOP, YAW_S)
        self.recording = False
        self.spin_wait(2.0)

        caps, segs, als = self.events_since(*n)
        truth_inst = self.truth_instant()
        truth_seg, self._seg_path = self.truth_segment()
        rows = self.fmt_events(caps, segs, truth_inst, truth_seg)
        self.fmt_alerts(als)
        snap = self.snap_stats()
        print('  流量统计帧：%s' % json.dumps(snap, ensure_ascii=False))

        expect_over = truth_inst is not None and truth_inst > limit
        ok = (len(caps) == 1 and len(segs) == 1
              and len(als) == (2 if expect_over else 0)
              and bool(rows) and all(r['over'] == expect_over for r in rows))
        print('  判定：%s（期望 1 抓拍 + 1 区间 + %d 告警，over 标志全=%s）' % (
            '通过' if ok else '不通过', 2 if expect_over else 0, expect_over))
        self.results.append({
            'pass': label, 'limit_kmh': limit, 'cmd_mps': V_CMD,
            'rtf': round(self.rtf(), 4), 'n_capture': len(caps),
            'n_segment': len(segs), 'n_alert': len(als),
            'truth_instant_kmh': None if truth_inst is None else round(truth_inst, 3),
            'truth_segment_kmh': None if truth_seg is None else round(truth_seg, 3),
            'truth_path_len_m': self._seg_path,
            'expect_over': bool(expect_over), 'stats': snap,
            'events': rows, 'ok': ok})
        if len(self.results) < 3:
            self.return_north()
        return ok

    # ------------------------------------------------------ 第 3 趟：护栏专项
    def pass_guard(self, label, limit):
        print('\n== %s（阈值 %.1f，A 段注入 %.0f m、B 段注入 %.1f m 瞬移）==' % (
            label, limit, JUMP_A_M, JUMP_B_M))
        g0 = self.guard_count(MARKER_FRAME)
        d0 = self.guard_count(MARKER_DROP)

        # ---- A 段：在途时 8 m 瞬移，必须作废且不出任何事件
        if not self.still():
            raise SystemExit('!! A 段起步前没停稳')
        self.publish_zones(limit)
        n = (len(self.caps), len(self.segs), len(self.als))
        self.trace = []
        self.recording = True
        fired = [False]

        def hook_a(y):
            if y <= JUMP_A_Y and not fired[0]:
                fired[0] = True
                self.teleport(self.pose()[1] + JUMP_A_M)
                return True
            return False

        self.drive_to(JUMP_A_Y, YAW_S, hook=hook_a)
        self.recording = False
        drift_a = self.settle_after_jump()
        caps_a, segs_a, als_a = self.events_since(*n)
        print('\n  ---- A 段（8 m 瞬移，落点回弹 %.2f m）：抓拍 %d / 区间 %d / 告警 %d'
              ' ----' % (drift_a, len(caps_a), len(segs_a), len(als_a)))
        for c in caps_a:
            print('    不该出现的抓拍：%s 雷达%.2f over=%s' % (
                c.capture_id, c.speed_kmh, bool(c.over)))
        for s in segs_a:
            print('    不该出现的区间：%s 均速%.2f over=%s' % (
                s.segment_id, s.avg_kmh, bool(s.over)))
        self.fmt_alerts(als_a)
        self.return_north()

        # ---- B 段：重新起一趟，途中 1.2 m 小跳变，事件必须照常且读数真实
        if not self.still():
            raise SystemExit('!! B 段起步前没停稳')
        self.publish_zones(limit)
        n2 = (len(self.caps), len(self.segs), len(self.als))
        self.trace = []
        self.recording = True
        fired_b = [False]

        def hook_b(y):
            if y <= JUMP_B_Y and not fired_b[0]:
                fired_b[0] = True
                self.teleport(self.pose()[1] + JUMP_B_M)
            return False

        self.drive_to(Y_STOP, YAW_S, budget=120.0, hook=hook_b)
        self.recording = False
        self.spin_wait(2.0)
        caps_b, segs_b, als_b = self.events_since(*n2)
        truth_inst = self.truth_instant()
        truth_seg, self._seg_path = self.truth_segment()
        rows = self.fmt_events(caps_b, segs_b, truth_inst, truth_seg)
        self.fmt_alerts(als_b)
        g1 = self.guard_count(MARKER_FRAME)
        d1 = self.guard_count(MARKER_DROP)
        snap = self.snap_stats()

        seen = [r['radar_kmh'] if r['kind'] == 'instant' else r['avg_kmh'] for r in rows]
        seen += [c.speed_kmh for c in caps_a] + [s.avg_kmh for s in segs_a]
        peak = max(seen) if seen else 0.0
        # 主判据是"跳变没变成假告警"：作废计数必须 +1，全场最大读数必须低于物理上限。
        # A 段事件数为 0 是理想情况；回弹滑行若真的把车推着穿过断面，节点会按实测速度
        # 出事件（那不是假告警），所以这里只记录不否决。
        ok = (d1 - d0 >= 1 and len(caps_b) == 1 and len(segs_b) == 1
              and peak < MAX_SANE and fired[0])
        print('\n  护栏计数变化：帧级「%s」%+d 条、通行作废「%s」%+d 条；'
              '本趟出现的最大读数 %.2f km/h（物理上限 %.1f）' % (
                  MARKER_FRAME, g1 - g0, MARKER_DROP, d1 - d0, peak, MAX_SANE))
        print('  判定：%s（作废计数 +1 + B 段照常出事件 + 全程读数不超上限%s）' % (
            '通过' if ok else '不通过',
            '，A 段零事件' if not (caps_a or segs_a or als_a) else '，A 段有回弹事件'))
        self.results.append({
            'pass': label, 'limit_kmh': limit,
            'jump_a_m': JUMP_A_M, 'jump_b_m': JUMP_B_M,
            'a_events': {'capture': len(caps_a), 'segment': len(segs_a),
                         'alert': len(als_a)},
            'a_rebound_drift_m': round(drift_a, 3),
            'guard_frame_warn_delta': g1 - g0, 'guard_drop_warn_delta': d1 - d0,
            'b_events': {'capture': len(caps_b), 'segment': len(segs_b),
                         'alert': len(als_b)},
            'b_truth_instant_kmh': None if truth_inst is None else round(truth_inst, 3),
            'b_truth_segment_kmh': None if truth_seg is None else round(truth_seg, 3),
            'b_peak_kmh': round(peak, 3), 'stats': snap, 'events': rows, 'ok': ok})
        return ok

    def teleport(self, y):
        """只在本趟的护栏专项里用：把车沿 y 平移，z 与航向保持当前值。"""
        p = self.pose()
        req = SetEntityState.Request()
        req.state.name = CAR
        req.state.pose.position.x = X_LANE
        req.state.pose.position.y = y
        req.state.pose.position.z = p[3] if abs(p[3] - Z_SPAWN) < 1.0 else Z_SPAWN
        req.state.pose.orientation.z = math.sin(p[2] / 2.0)
        req.state.pose.orientation.w = math.cos(p[2] / 2.0)
        req.state.reference_frame = 'world'
        r = self.call(self.cli_put, req, '/set_entity_state')
        print('  [瞬移] %s y %.2f -> %.2f 服务接受=%s' % (
            CAR, p[1], y, bool(r and r.success)))
        self.spin_wait(1.0)

    def settle_after_jump(self):
        """量一下瞬移后的回弹滑行多远（#61 的实测项），并等它彻底静止再进下一段。"""
        p0 = self.pose()
        print('  [落点] 实测 (%.2f, %.2f, z=%.2f) 航向 %.1f°' % (
            p0[0], p0[1], p0[3], math.degrees(p0[2])))
        self.holding = True
        drift = 0.0
        for i in range(8):
            self.spin_wait(3.0)
            p = self.pose()
            drift = math.hypot(p[0] - p0[0], p[1] - p0[1])
            print('    %2d s：离落点 %.2f m（z=%.2f）' % ((i + 1) * 3, drift, p[3]))
            if self.still(3.0, 0.20):
                print('    已静止，末位 (%.2f, %.2f) 航向 %.1f°' % (
                    p[0], p[1], math.degrees(p[2])))
                return drift
        print('    !! 24 s 没静止，按当前值继续（护栏事件已计数，不受影响）')
        return drift

    # ------------------------------------------------------------------ 收尾
    def export(self):
        r = self.call(self.cli_exp, Trigger.Request(), '/speed/export')
        print('  [export] %s' % (r.message if r else '无回执'))

    def run(self):
        print('=== 前置核对 ===')
        self.spin_wait(2.0)
        t0 = time.monotonic()
        while not self.latest:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > 25:
                raise SystemExit('!! 25 s 内没收到 /model_states')
        if self.sim_t <= 0:
            raise SystemExit('!! 没收到 /clock，仿真时钟没在走')
        p = self.pose()
        print('  %s 当前 (%.2f, %.2f, z=%.2f) 航向 %.1f°' % (
            CAR, p[0], p[1], p[3], math.degrees(p[2])))
        if abs(p[0] - X_LANE) > 0.8 or not (-14.0 < p[1] < -3.0):
            raise SystemExit('!! %s 不在 RingB 东腿 (x=16.8, y∈[-14,-3]) 上 —— '
                             '本脚本全程不摆位，车得自己在这条道上；不在就先跑 '
                             'tools/restart_clean_stack.sh' % CAR)
        if abs(p[3] - Z_SPAWN) > 1.0:
            raise SystemExit('!! %s 离地高度 z=%.2f（期望 %.2f）—— 它挂在界外/半空，'
                             '物理态不干净' % (CAR, p[3], Z_SPAWN))
        r = self.zone(op=SetSpeedZone.Request.OP_LIST)
        print('  点位 %d 个：%s' % (len(r.zones), ' | '.join(r.zones)))
        if not self.still(4.0, 0.25):
            raise SystemExit('!! 开局 %s 就在动，物理态不干净' % CAR)
        self.zone_soft(op=SetSpeedZone.Request.OP_REMOVE, zone_id=YAML_EAST_SEG)
        # 起点必须在入口断面圆外至少 0.4 m，否则 reset_state 重建的种子会吃掉进圈边沿。
        # 不到位就自己开回去（这一段仍然只发 cmd_vel，不瞬移）。
        if self.pose()[1] < -5.2:
            print('  先把车开回起点（y=%.2f）' % Y_NORTH_BACK)
            self.return_north()
        if abs(wrap(self.pose()[2] - YAW_S)) > 0.2:
            self.turn_to(YAW_S)
        self.pass_normal('第 1 趟 低阈值（必告警）', LIMIT_OVER)
        self.pass_normal('第 2 趟 高阈值（必干净）', LIMIT_CLEAN)
        self.pass_guard('第 3 趟 跳变护栏', LIMIT_OVER)

        print('\n=== 恢复点位基线 + 导出 ===')
        self.zone(op=SetSpeedZone.Request.OP_CLEAR)
        self.export()
        print('\n=== 五车末态 ===')
        for n in sorted(self.latest):
            x, y, yaw, z = self.latest[n]
            print('  %-6s (%7.2f,%7.2f) yaw %6.1f° z %.2f' % (
                n, x, y, math.degrees(yaw), z))

        d = os.path.dirname(self.outjson)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.outjson, 'w', encoding='utf-8') as f:
            json.dump({'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
                       'car': CAR, 'lane_x': X_LANE, 'cmd_mps': V_CMD,
                       'limit_over': LIMIT_OVER, 'limit_clean': LIMIT_CLEAN,
                       'zones': {'instant_center': [X_LANE, INST_Y],
                                 'segment': [[X_LANE, ENTRY_Y], [X_LANE, EXIT_Y]],
                                 'radius': RADIUS, 'radar': [RADAR_X, INST_Y],
                                 'beam_deg': -90.0},
                       'guard': {'log': GUARD_LOG, 'frame_marker': MARKER_FRAME,
                                 'drop_marker': MARKER_DROP, 'max_sane_kmh': MAX_SANE},
                       'plates': self.plates, 'passes': self.results},
                      f, ensure_ascii=False, indent=2)
        print('\n[json] %s' % self.outjson)
        allok = len(self.results) == 3 and all(v.get('ok') for v in self.results)
        print('[判定] %s' % ('三趟全部通过' if allok else '有趟次不通过，见逐趟判定'))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records/speed_live_pass.json'
    rclpy.init()
    node = Runner(out)
    code = 0
    try:
        node.run()
    except SystemExit as exc:
        code = exc.code if isinstance(exc.code, int) else 1
        print('!! 停跑（exit=%s）：%s' % (code, exc if not isinstance(exc.code, int)
                                          else ''))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
