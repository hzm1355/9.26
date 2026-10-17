#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务四条目 2/3/4 的**受控单跑**取证：由本脚本亲自把一辆车以定速开过"瞬时点位 + 区间点位"，
用独立测得的高频位姿算出真值速度，和检测节点的读数逐条对照。

为什么要有这一份：现场取证窗口里车队驱动会偶发"复位风暴"（车被甩到界外、互相嵌入弹开），
一轮 600 s 能打出零事件，也能打出几十条被跳变护栏拦掉的帧 —— 那种窗口里"读数对不对"
根本没法核对，因为连真值都不存在。受控单跑把真值握在自己手里：
  * 瞬时通道：真值 = 最近点位前后各 1.5 m 的带内位移 / 带内时间（高频 /model_states 插值）；
  * 区间通道：真值 = entry→exit 的圆心距 20.0 m / 车辆圆心两次进入断面的时间差，
    与检测节点的 L/t 完全同口径，差别只剩它的边沿是在 10 Hz 时钟格上取的。
两趟同一速度、同一路线，只有阈值不同（2.0 → 必告警 / 99.0 → 必干净），
所以"界面告警是阈值驱动的"这一条在同一份数据里就自证了。

前置（外层 speed_ctrl_run.sh 已经做好）：signal_drive_node 必须先杀掉，
否则它和脚本抢同一个 /car_2/cmd_vel，还会把车复位走。

第 3 趟是**跳变护栏**专项（任务四 #60）：车过瞬时点位后人为 /set_entity_state 前移 8 m，
造出一个 8 m/0.2 s ≈ 144 km/h 的窗口弦速。改造前这会变成一条假超速告警和一个假峰值；
加了 max_sane_kmh 护栏后必须看到：护栏计数 +1、在途区间被作废、不产生任何超速告警。
其余四辆车全程锁轮停在 y=-14.6 的南环车道上 —— 之前几轮受控跑"摆完就被撞飞 16.5 m"
不是地面坡度（free_roll_probe.py 实测：同点位锁轮 15 s / 撤指令自由 25 s，两种车头朝向
位移都是 0.00 m），而是复位风暴里乱跑的车撞上测试车。所以除了杀掉驱动，
还要保证走廊上没有别的车。

用法：python3 speed_ctrl_evidence.py [输出目录=/tmp/v2x_records]
"""
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import SetEntityState
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats
from v2x_msgs.srv import SetSpeedZone

# ── 路线：Ring B 的北环东行车道 y=4.6（与 speed_zones.yaml 的 SZ-B-north-instant 同一条道，
#    所以 YAML 基线点位会在同一趟里一起出事件，正好当"配置点位也在生效"的旁证）
LANE_Y = 4.6
Z = 6.70                  # 车辆静止高度（place_cars.py 实测值，按 spawn 的 7.0 摆会自由落体）
START_X = -9.0            # 起点：入口断面外缘（-7.6）之前 1.4 m，让速度窗口先攒稳
END_X = 15.0              # 终点：出口断面（14.0）之后，保证"离开出口圆"的边沿出得来
V_CMD = 0.70              # 指令速度 m/s = 2.52 km/h（车队巡航同款）

INST_ID = 'SZ-CTRL-instant'
SEG_ID = 'SZ-CTRL-segment'
INST_X = 4.0              # 瞬时点位中心（车道上）
RADAR_Y = 2.6             # 雷达杆在车道南侧，波束轴沿 +x（beam=0）正对来车
ENTRY_X = -6.0
EXIT_X = 14.0
RADIUS = 1.6

# 其它四辆车的避让位：南环西行车道 y=-14.6 —— 沙盘里少数的平直沥青面，离北环走廊
# y=4.6 有 19 m。试过 y=-22（沙盘外斜面），摆完的车一路下滑到 y=-387，不能用作避让位。
PARK = {'car_1': (-15.0, -14.6), 'car_3': (-8.0, -14.6),
        'car_4': (2.0, -14.6), 'car_5': (10.0, -14.6)}

K_Y = 0.10                # 横向误差 -> 角速度
K_H = 0.80                # 航向 -> 角速度
A_CAP = 0.35

# 跳变护栏专项：注入的瞬移量与护栏告警的日志出处
JUMP_M = 8.0
JUMP_AT = 6.5             # 已过瞬时点位（2.5/5.5 两个真值断面），还没到出口断面 12.4
GUARD_LOG = '/tmp/speed_detect_node.log'
GUARD_MARKER = '超物理上限'


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Runner(Node):

    def __init__(self, outdir):
        super().__init__('speed_ctrl_evidence')
        self.outdir = outdir
        self.latest = {}            # name -> (x, y, yaw)
        self.vel = {}               # name -> 平面速度模 m/s（判"摆完有没有静止"）
        self.sim_t = None
        self.caps = []
        self.segs = []
        self.als = []
        self.stats = {}             # zone_id -> 最新一条统计（按 stamp 整表换新）
        self.stats_round = 0.0
        self.passes = []            # 每趟的对照结果

        self.pub_cmd = self.create_publisher(Twist, '/car_2/cmd_vel', 10)
        # **全程锁住另外四辆车**：它们停在 y=-14.6 的南环车道上，但只摆位不灌零速的话，
        # 一旦被别的东西顶一下就会飘进北环走廊撞测试车。gz 的 diff_drive 只在"收到指令"
        # 时才锁轮（cmd_vel_timeout 0.3 s 之后轮子是自由的），所以 20 Hz 持续灌零速 Twist。
        # （注：曾以为是"沙盘地面有坡度、断电自滚"，free_roll_probe.py 已推翻——
        #  同点位撤指令 25 s 位移 0.00 m。车不会自己滚，是别的车撞它。）
        self.hold = {}
        for n in ('car_1', 'car_2', 'car_3', 'car_4', 'car_5'):
            self.hold[n] = self.create_publisher(Twist, '/%s/cmd_vel' % n, 10)
        self.drive_car2 = False         # True 时 car_2 由驾驶环给指令，别用零速去顶
        self.create_timer(0.05, self.on_hold)
        self.create_timer(0.05, self.on_hold)
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        self.create_subscription(SpeedCapture, '/speed/capture', self.on_cap, 50)
        self.create_subscription(SpeedSegment, '/speed/segment', self.on_seg, 50)
        self.create_subscription(SpeedAlert, '/speed/alert', self.on_al, 50)
        self.create_subscription(SpeedStats, '/speed/stats', self.on_stats, 20)

        self.cli_zone = self.create_client(SetSpeedZone, '/speed/zones')
        self.cli_put = self.create_client(SetEntityState, '/set_entity_state')
        self.cli_exp = self.create_client(Trigger, '/speed/export')

    # ------------------------------------------------------------------ 采集
    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_states(self, msg):
        for n, p, t in zip(msg.name, msg.pose, msg.twist):
            if n.startswith('car_'):
                self.latest[n] = (p.position.x, p.position.y, yaw_of(p.orientation))
                self.vel[n] = math.hypot(t.linear.x, t.linear.y)

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

    def on_hold(self):
        """20 Hz 灌零速：除了正在被驾驶的 car_2，其余车全程刹住（见 __init__ 的锁轮说明）。"""
        for n, pub in self.hold.items():
            if n == 'car_2' and self.drive_car2:
                continue
            pub.publish(Twist())

    # ------------------------------------------------------------------ 工具
    def spin_wait_sim(self, dur):
        t0 = self.t()
        while rclpy.ok() and self.t() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.05)

    def t(self):
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - _T0 > 30:
                raise SystemExit('!! 30 s 内没收到 /clock，仿真时钟没在走')
        return self.sim_t

    def call(self, cli, req, who, tries=2):
        """先等服务**发现**再发请求：上一轮直接 call_async，第一条 OP_CLEAR 在 20 s 里
        没人接（同一时刻用 ros2 service call 手动打 OP_LIST 只花 1.7 s 就回包 —— 说明
        不是节点卡住，是新节点的 DDS 服务发现还没完成，请求发进了空气）。"""
        t0 = time.monotonic()
        while not cli.wait_for_service(timeout_sec=3.0):
            if not rclpy.ok():
                raise SystemExit('!! %s 服务没了（rclpy 已 shutdown）' % who)
            print('  ... 等 %s 服务发现（已 %.0f s）' % (who, time.monotonic() - t0))
            if time.monotonic() - t0 > 60.0:
                raise SystemExit('!! %s 60 s 内没发现，节点没起还是话题名不对？' % who)
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
        print('  [zones] %s' % r.message)
        if not r.success:
            raise SystemExit('!! /speed/zones 失败，现场不能继续')
        return r

    def place(self, name, x, y, yaw=0.0, want=True):
        """摆位并**回读确认**：/set_entity_state 的 success 只代表"命令被接受"，
        复位风暴里的车可能正好被物理引擎顶着，落位要自己验（响应体只有 bool success，
        没有 status_message —— 取它会把脚本掀掉）。
        容差 1.5 m：落点有 ~0.85–1.25 m 的系统性 +x 偏差（见 settle 的说明），
        按 0.35 m 判会把每辆车都判成"摆不上"，第三次取证就是栽在这儿的。"""
        for attempt in range(4):
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = x
            req.state.pose.position.y = y
            req.state.pose.position.z = Z
            req.state.pose.orientation.z = math.sin(yaw / 2.0)
            req.state.pose.orientation.w = math.cos(yaw / 2.0)
            req.state.reference_frame = 'world'
            r = self.call(self.cli_put, req, '/set_entity_state')
            if not (r and r.success):
                print('  !! 摆放 %s 服务拒绝（第 %d 次）' % (name, attempt + 1))
            self.spin_wait_wall(0.8)
            p = self.latest.get(name)
            if p is None:
                continue
            err = math.hypot(p[0] - x, p[1] - y)
            if err < 1.5:
                return True
            if attempt == 2:
                print('  !! %s 落位失败：目标(%.1f,%.1f) 实测(%.2f,%.2f) 偏差 %.2f m' % (
                    name, x, y, p[0], p[1], err))
                if want:
                    raise SystemExit('!! 车辆摆不到位，受控真值不成立，停跑')
        return True

    def spin_wait_wall(self, dur):
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.05)

    def others_near(self, x, y, skip, radius=4.0):
        return sorted([(n, math.hypot(p[0] - x, p[1] - y))
                       for n, p in self.latest.items()
                       if n != skip and n.startswith('car_')
                       and math.hypot(p[0] - x, p[1] - y) < radius], key=lambda t: t[1])

    def wait_clear(self, name, x, y, radius=4.0, budget=90.0):
        """落点附近有别的车就**不摆**：车体重叠会被接触求解器沿车头轴弹射，
        实测签名是"落点偏差 17~18 m、3 s 位移 16~17 m、方向沿车头（yaw=0 就是 +x）"，
        上一轮受控跑连 car_2 都被这样弹掉，真值当场不存在了。
        本脚本全程 20 Hz 灌零速，车不会自己走；这里等的是"接手时还在飞的车落回静止"。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < budget:
            near = self.others_near(x, y, name, radius)
            if not near:
                return True
            print('    落点 (%.1f,%.1f) 附近有 %s —— 等净空' % (
                x, y, ', '.join('%s %.1f m' % t for t in near)))
            self.spin_wait_wall(5.0)
        print('    !! 等了 %.0f s 仍不净空，硬摆（靠后面的停稳判定兜底）' % budget)
        return False

    def calm(self, budget=90.0, thresh=0.30):
        """等全场车辆都静止再开始摆位。判据只用位置增量（/model_states 的 twist 列
        对静止的车也报 4~5 m/s，不可信）。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < budget:
            last = dict(self.latest)
            self.spin_wait_wall(3.0)
            now = dict(self.latest)
            moving = sorted(n for n in now if n in last and math.hypot(
                now[n][0] - last[n][0], now[n][1] - last[n][1]) > thresh)
            if not moving:
                print('  全场静止（3 s 位移均 <%.2f m）' % thresh)
                return True
            print('  还在动：%s' % ', '.join(moving))
        print('  !! %.0f s 没等来静止，继续（还有 wait_clear 兜底）' % budget)
        return False

    def settle(self, name, x, y, yaw=0.0, want=True):
        """摆位并**等它停住**，判据只看位置增量：
          * 落点有 ~0.85–1.25 m 的系统性 +x 偏差（park_test.sh 实测四辆车同方向同量），
            所以"到位"按 1.5 m 容差判，按 0.35 m 判会把每辆车都判成摆不上；
          * /model_states 的 twist 对**静止**的车也会报 4~5 m/s（同样实测），
            拿它判静止会一直重试 —— 判稳只能用 3 s 内的位移。"""
        for i in range(3):
            self.wait_clear(name, x, y)
            self.place(name, x, y, yaw, want=want)
            p0 = self.latest.get(name)
            self.spin_wait_wall(3.0)
            p1 = self.latest.get(name)
            if p0 is None or p1 is None:
                continue
            drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            err = math.hypot(p1[0] - x, p1[1] - y)
            if drift < 0.25 and err < 1.5:
                return True
            print('  %s 未停稳（第 %d 次）：落点偏差 %.2f m，3 s 位移 %.2f m，'
                  '上报速度 %.2f m/s（这个速度列不可信，只看位移）' % (
                      name, i + 1, err, drift, self.vel.get(name, -1)))
        p = self.latest.get(name)
        print('  !! %s 落不稳：目标(%.1f,%.1f) 实测(%s)' % (
            name, x, y, ('%.2f,%.2f' % (p[0], p[1])) if p else '?'))
        if want:
            raise SystemExit('!! 本车摆不到位/停不住，受控真值不成立，停跑')
        return False

    def stop_cmd(self):
        self.drive_car2 = False
        for _ in range(6):
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def guard_count(self):
        """数检测节点日志里的护栏告警行数 —— 护栏生效的唯一外部痕迹。
        先跑一次看真实回包再用（不猜过滤式）：marker 串必须与 speed_detect_node.py
        里 warn 的 "超物理上限" 一致。"""
        try:
            with open(GUARD_LOG, encoding='utf-8', errors='replace') as f:
                return sum(1 for ln in f if GUARD_MARKER in ln)
        except OSError:
            return -1

    # ------------------------------------------------------------------ 一趟
    def run_pass(self, label, limit, jump_at=None):
        """以 V_CMD 定速东行穿过两个点位；返回本趟的事件与真值对照。
        jump_at 不为空时，车过该 x 后立刻被 /set_entity_state 前移 JUMP_M 米，
        用来专门验证跳变护栏（见模块 docstring 第 3 趟说明）。"""
        self.zone(op=SetSpeedZone.Request.OP_SET_LIMIT, zone_id=INST_ID,
                  limit_kmh=limit)
        self.zone(op=SetSpeedZone.Request.OP_SET_LIMIT, zone_id=SEG_ID,
                  limit_kmh=limit)
        print('\n== %s（阈值 %.1f km/h，指令 %.2f m/s = %.2f km/h%s）==' % (
            label, limit, V_CMD, V_CMD * 3.6,
            ('，x>%.1f 处注入 %.1f m 瞬移' % (jump_at, JUMP_M)) if jump_at else ''))
        self.settle('car_2', START_X, LANE_Y)
        self.spin_wait_sim(4.0)          # 让速度窗口在起点先攒稳（起点静止，读数应为 0）

        n_cap, n_seg, n_al = len(self.caps), len(self.segs), len(self.als)
        g0 = self.guard_count()
        samples = []                     # (仿真秒, x, y)
        prev = None
        jumped = False
        cross = {}                       # x 断面 -> 插值时刻
        deadline = self.t() + 150.0
        timeout = True
        stalled = False
        x_best = None                    # 历史最大 x：只认"确实前进"，倒滑不算
        t_advance = self.t()
        next_report = self.t() + 10.0
        self.drive_car2 = True
        while rclpy.ok() and self.t() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            p = self.latest.get('car_2')
            if p is None:
                continue
            x, y, yaw = p
            now = self.t()
            samples.append((now, x, y))
            if x_best is None or x > x_best + 0.05:
                x_best, t_advance = x, now
            if now - t_advance > 18.0:
                # 卡死：继续跑只会把真值搅浑（倒碰/顶楼期间的位移不是巡航），直接作废弃趟
                stalled = True
                timeout = False
                print('  !! 卡死：x=%.2f 已 %.0f s 没前进，本趟作废。在场车辆：' % (
                    x, now - t_advance))
                for n in sorted(self.latest):
                    q = self.latest[n]
                    print('       %s (%.2f,%.2f) 距本车 %.1f m' % (
                        n, q[0], q[1], math.hypot(q[0] - x, q[1] - y)))
                break
            if now >= next_report:
                next_report = now + 10.0
                near = min(((math.hypot(q[0] - x, q[1] - y), n)
                            for n, q in self.latest.items() if n != 'car_2'),
                           default=(99.0, '-'))
                print('  ... x=%6.2f y=%5.2f 航向%+5.0f° 横向偏差%.2f m 最近他车 %s %.1f m' % (
                    x, y, math.degrees(yaw), abs(y - LANE_Y), near[1], near[0]))
            if prev is not None:
                px, py, pt = prev
                if x > px:
                    for cx in (ENTRY_X - RADIUS, EXIT_X - RADIUS, 2.5, 5.5):
                        if cx not in cross and px <= cx < x:
                            f = (cx - px) / (x - px)
                            cross[cx] = pt + f * (now - pt)
            prev = (x, y, now)
            if jump_at is not None and not jumped and x >= jump_at:
                # 注入一个"行驶中不可能出现"的位姿跳变：8 m 在 0.2 s 窗口里是 144 km/h。
                # 护栏必须把它判成非行驶跳变：作废在途区间、且不产生超速告警。
                print('  !! 护栏注入：car_2 由 x=%.2f 瞬移到 x=%.2f（造 %.1f m 跳变）' % (
                    x, x + JUMP_M, JUMP_M))
                self.place('car_2', x + JUMP_M, LANE_Y, 0.0)
                jumped = True
                prev = None              # 别让传送距离被当成"车开过了断面"
                continue
            if x >= END_X:
                timeout = False
                break
            cmd = Twist()
            cmd.linear.x = V_CMD
            cmd.angular.z = max(-A_CAP, min(A_CAP,
                                    K_Y * (LANE_Y - y) - K_H * yaw))
            self.pub_cmd.publish(cmd)
        self.stop_cmd()
        self.spin_wait_sim(5.0)          # 等"离开圆"的抓拍边沿与统计轮

        caps = self.caps[n_cap:]
        segs = self.segs[n_seg:]
        als = self.als[n_al:]

        # 真值：与检测节点同口径的两个量
        truth_inst = None
        if 2.5 in cross and 5.5 in cross:
            dt = cross[5.5] - cross[2.5]
            truth_inst = 3.0 / dt * 3.6 if dt > 1e-6 else None
        truth_seg = None
        if (EXIT_X - RADIUS) in cross and (ENTRY_X - RADIUS) in cross:
            dt = cross[EXIT_X - RADIUS] - cross[ENTRY_X - RADIUS]
            truth_seg = 20.0 / dt * 3.6 if dt > 1e-6 else None

        if timeout:
            print('  !! 150 s 没走完路线，本趟按异常记录')
        if stalled:
            print('  本趟已作废，不进对照')
        if samples:
            print('  横向偏差 |y-%.1f| 最大 %.2f m，终点 x=%.2f，走完用时 %.1f s' % (
                LANE_Y, max(abs(s[2] - LANE_Y) for s in samples),
                samples[-1][1], samples[-1][0] - samples[0][0]))
        for m in caps:
            print('  瞬时 %s %s 牌=%s 雷达%.2f 位姿%.2f 峰%.2f 夹角%.1f° 阈值%.2f %s' % (
                m.zone_id, m.vehicle_id, m.plate_number or '-', m.speed_kmh,
                m.speed_pose_kmh, m.speed_peak_kmh, m.beam_angle_deg,
                m.limit_kmh, '超速' if m.over else '正常'))
        for m in segs:
            print('  区间 %s %s 牌=%s L=%.1f t=%.2f 均速%.2f 峰%.2f 阈值%.2f %s' % (
                m.zone_id, m.vehicle_id, m.plate_number or '-', m.length_m,
                m.travel_time_s, m.avg_kmh, m.max_kmh, m.limit_kmh,
                '超速' if m.over else '正常'))
        for a in als:
            print('  告警 %s %s %s %s 速度%.2f 阈值%.2f 超%.2f 位置(%.2f,%.2f) 时刻%.2f' % (
                a.kind, a.zone_id, a.vehicle_id, a.plate_number or '-',
                a.speed_kmh, a.limit_kmh, a.over_kmh,
                a.pose.position.x, a.pose.position.y, a.ts))
        if not caps:
            print('  !! 本趟没有任何瞬时抓拍 —— 点位不在车道上或车没进圆')
        if not segs:
            print('  !! 本趟没有区间结果 —— 边沿没触发（检查 entry/exit 与半径）')

        g1 = self.guard_count()
        guard_lines = g1 - g0 if (g0 >= 0 and g1 >= 0) else -1
        insane = [m for m in caps if m.speed_kmh > 12.0 or m.speed_pose_kmh > 12.0]
        insane += [m for m in segs if m.avg_kmh > 12.0 or m.max_kmh > 12.0]
        insane += [a for a in als if a.speed_kmh > 12.0]
        if jump_at is not None:
            print('  护栏判定：注入 %.1f m 瞬移 -> 护栏新增告警行 %d，'
                  '本趟超物理上限读数 %d 条，区间结果 %d 条，超速告警 %d 条' % (
                      JUMP_M, guard_lines, len(insane), len(segs),
                      sum(1 for a in als if int(a.kind) in (1, 2) or a.over_kmh > 0)))
            if not jumped:
                print('  !! 瞬移没注入成功（车没开到 x=%.1f 就结束本趟）' % jump_at)
            elif insane:
                print('  !! 护栏漏了：本趟仍出现超物理上限的读数，见上面明细')
            elif segs:
                print('  !! 在途区间没作废：跳变趟居然还出了区间结果')
            else:
                print('  护栏生效：跳变被拦掉，在途区间作废，没有产生假告警')

        self.passes.append({
            'label': label,
            'limit_kmh': limit,
            'cmd_mps': V_CMD,
            'jump_inject_m': JUMP_M if jump_at else None,
            'jump_inject_at': jump_at,
            'jump_done': jumped,
            'guard_lines_this_pass': guard_lines,
            'insane_readings': len(insane),
            'truth_instant_kmh': round(truth_inst, 3) if truth_inst else None,
            'truth_segment_kmh': round(truth_seg, 3) if truth_seg else None,
            'cross_ts': {str(k): round(v, 3) for k, v in cross.items()},
            'captures': [{
                'capture_id': m.capture_id, 'zone_id': m.zone_id,
                'vehicle_id': m.vehicle_id, 'plate_number': m.plate_number,
                'speed_kmh': round(m.speed_kmh, 3),
                'speed_pose_kmh': round(m.speed_pose_kmh, 3),
                'speed_peak_kmh': round(m.speed_peak_kmh, 3),
                'limit_kmh': round(m.limit_kmh, 3),
                'over': m.over, 'beam_angle_deg': round(m.beam_angle_deg, 2),
                'x': round(m.pose.position.x, 3), 'y': round(m.pose.position.y, 3),
                'sim_ts': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            } for m in caps],
            'segments': [{
                'segment_id': m.segment_id, 'zone_id': m.zone_id,
                'vehicle_id': m.vehicle_id, 'plate_number': m.plate_number,
                'length_m': round(m.length_m, 3),
                'travel_time_s': round(m.travel_time_s, 3),
                'avg_kmh': round(m.avg_kmh, 3), 'max_kmh': round(m.max_kmh, 3),
                'limit_kmh': round(m.limit_kmh, 3), 'over': m.over,
                'sim_ts': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            } for m in segs],
            'alerts': [{
                'kind': int(a.kind), 'ref_id': a.ref_id, 'zone_id': a.zone_id,
                'vehicle_id': a.vehicle_id, 'plate_number': a.plate_number,
                'speed_kmh': round(a.speed_kmh, 3),
                'limit_kmh': round(a.limit_kmh, 3),
                'over_kmh': round(a.over_kmh, 3),
                'ts': round(a.ts, 3),
                'x': round(a.pose.position.x, 3), 'y': round(a.pose.position.y, 3),
            } for a in als],
        })
        return truth_inst, truth_seg

    # ------------------------------------------------------------------ 汇总
    def show_csv(self):
        """导出之后回读三张 CSV，只挑本次受控点位的行 —— 证明"跑出来的事件真的落进了导出表"，
        而不是只在内存里过了一遍（CSV 必须 DictReader + utf-8-sig，表头是中文的）。"""
        import csv
        for name in ('speed_captures.csv', 'speed_segments.csv', 'speed_alerts.csv'):
            path = os.path.join(self.outdir, name)
            print('  --- %s' % name)
            if not os.path.exists(path):
                print('      (文件不存在)')
                continue
            with open(path, encoding='utf-8-sig', newline='') as f:
                n = 0
                for r in csv.DictReader(f):
                    if not str(r.get('zone_id', '')).startswith('SZ-CTRL'):
                        continue
                    n += 1
                    keys = ('capture_id', 'segment_id', 'ref_id', 'kind', 'zone_id',
                            'vehicle_id', 'plate_number', 'speed_kmh', 'speed_pose_kmh',
                            'avg_kmh', 'travel_time_s', 'length_m', 'limit_kmh',
                            'over_kmh', 'over', 'x', 'y', 'ts')
                    print('      ' + '  '.join('%s=%s' % (k, r[k])
                          for k in keys if k in r and r[k] != ''))
                print('      受控行 %d 条' % n)

    def dump(self):
        os.makedirs(self.outdir, exist_ok=True)
        r = self.call(self.cli_exp, Trigger.Request(), '/speed/export')
        print('\n[/speed/export] %s' % r.message)
        self.show_csv()
        rows = []
        for zid, m in sorted(self.stats.items()):
            rows.append({
                'zone_id': zid, 'mode': int(m.mode), 'road': m.road,
                'limit_kmh': round(m.limit_kmh, 3), 'window_s': m.window_s,
                'flow': m.flow, 'vehicles': m.vehicles,
                'avg_kmh': round(m.avg_kmh, 3), 'max_kmh': round(m.max_kmh, 3),
                'min_kmh': round(m.min_kmh, 3), 'over_count': m.over_count,
                'over_ratio': round(m.over_ratio, 4),
                'total_flow': m.total_flow, 'total_over': m.total_over,
                'pending_segment': m.pending_segment,
            })
            print('  统计 %s %s limit=%.2f 流量%d 车次数%d 均速%.2f 峰值%.2f 超速%d（%.0f%%）累积%d/%d' % (
                zid, '区间' if m.mode else '瞬时', m.limit_kmh, m.flow, m.vehicles,
                m.avg_kmh, m.max_kmh, m.over_count, m.over_ratio * 100,
                m.total_flow, m.total_over))
        out = {'passes': self.passes, 'stats': rows,
               'export_message': r.message,
               'lane_y': LANE_Y, 'cmd_mps': V_CMD,
               'zones': {'instant': {'id': INST_ID, 'center': [INST_X, LANE_Y],
                                     'radar': [INST_X, RADAR_Y], 'beam_deg': 0.0,
                                     'radius': RADIUS},
                         'segment': {'id': SEG_ID, 'entry': [ENTRY_X, LANE_Y],
                                     'exit': [EXIT_X, LANE_Y], 'length_m': 20.0,
                                     'radius': RADIUS}}}
        path = os.path.join(self.outdir, 'speed_ctrl_evidence.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
        print('  已写 %s' % path)


def main():
    global _T0
    _T0 = time.monotonic()
    outdir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/v2x_records'
    rclpy.init()
    node = Runner(outdir)
    try:
        node.t()                                  # 等 /clock 通
        node.spin_wait_wall(6.0)                  # 等 DDS 把三个服务端都发现完再发第一个请求
        print('[ctrl] 仿真时钟 %.1f s，已见车辆 %d 辆' % (
            node.sim_t, len(node.latest)))
        node.zone(op=SetSpeedZone.Request.OP_CLEAR)
        print('[ctrl] 接手现场：先等全场静止，再逐辆挪到避让位')
        node.calm()
        for n, (x, y) in PARK.items():            # 避让：四辆车挪到车道外（摆不稳只报不断跑）
            node.settle(n, x, y, want=False)
        node.spin_wait_wall(2.0)
        for n in sorted(node.latest):             # 走廊净空检查：别的车压在车道上会撞停本车
            if n == 'car_2':
                continue
            q = node.latest[n]
            if abs(q[1] - LANE_Y) < 4.0 and -14.0 < q[0] < 20.0:
                print('  !! %s 还在走廊上 (%.2f,%.2f)，本趟可能被它撞停' % (n, q[0], q[1]))
        node.zone(op=SetSpeedZone.Request.OP_ADD, zone_id=INST_ID, mode=0,
                  limit_kmh=2.0, center_x=INST_X, center_y=LANE_Y,
                  radius=RADIUS, radar_x=INST_X, radar_y=RADAR_Y, beam_deg=0.0,
                  road='受控单跑·北环东行 y=4.6')
        node.zone(op=SetSpeedZone.Request.OP_ADD, zone_id=SEG_ID, mode=1,
                  limit_kmh=2.0, entry_x=ENTRY_X, entry_y=LANE_Y,
                  exit_x=EXIT_X, exit_y=LANE_Y, radius=RADIUS,
                  radar_x=ENTRY_X, radar_y=RADAR_Y, beam_deg=0.0,
                  road='受控单跑·北环东行 20 m')
        node.spin_wait_sim(3.0)
        for label, limit, jump_at in (
                ('第 1 趟 阈值 2.0（巡航 2.52，应告警）', 2.0, None),
                ('第 2 趟 阈值 99.0（同速，应零告警）', 99.0, None),
                ('第 3 趟 阈值 2.0 + 注入 %.0f m 瞬移（跳变护栏专项）' % JUMP_M,
                 2.0, JUMP_AT)):
            ti = ts = None
            for attempt in range(3):
                ti, ts = node.run_pass(
                    label if attempt == 0 else '%s（重试 %d）' % (label, attempt),
                    limit, jump_at=jump_at)
                print('  真值对照：瞬时 %s / 区间 %s' % (
                    '%.2f km/h' % ti if ti else '缺（没走完）',
                    '%.2f km/h' % ts if ts else '缺（没触发）'))
                last = node.passes[-1]
                if jump_at is not None:
                    # 护栏趟的达标判据与常规趟不同：区间"没结果"才是对的（在途通行被作废）。
                    # 护栏行数为 0 说明检测节点是旧的（warn 只打前 5 次），要先换新实例。
                    if last['jump_done'] and last['insane_readings'] == 0 \
                            and last['guard_lines_this_pass'] > 0:
                        break
                    print('  护栏趟未达标（注入成功=%s 护栏新增行=%d 超上限读数=%d），重来' % (
                        last['jump_done'], last['guard_lines_this_pass'],
                        last['insane_readings']))
                    continue
                if ti and ts:
                    break
                # 上一趟没走通（被乱跑的车撞停 / 断面边沿没触发）：把走廊上的车重新挪开再来
                for n in sorted(node.latest):
                    if n == 'car_2':
                        continue
                    q = node.latest[n]
                    if abs(q[1] - LANE_Y) < 8.0 and -20.0 < q[0] < 24.0:
                        print('  清走廊：%s 在 (%.2f,%.2f)' % (n, q[0], q[1]))
                        node.settle(n, *PARK.get(n, (-15.0, -14.6)), want=False)
        node.stop_cmd()
        node.dump()
        node.zone(op=SetSpeedZone.Request.OP_CLEAR)   # 回到 YAML 基线
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


_T0 = time.monotonic()

if __name__ == '__main__':
    main()
