#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务五（五）条目 4：停车场闸机管控逻辑（核验→抬杆→通行→地感离场→落杆）。

接口一览（点位与阈值全部来自 parking_zones.yaml，改几何不用改代码）

  输入  /parking/gate/<gid>/state   BarrierState         插件按 joint 实测值发布（动力学权威）
  输入  /plate/capture              PlateCapture         两路卡口相机的抓拍事件（libv2x_gate）
  输入  /plate/result               PlateResult          识别结果，按 capture_id 回指抓拍
  输入  /parking/gate_manual_cmd    std_msgs/String      "gate_in:1.5708" 或 "gate_in:AUTO"
  输入  /clock                      rosgraph_msgs/Clock  仿真钟（BEST_EFFORT，实测 ~10 Hz）
  输出  /parking/gate/<gid>/cmd     std_msgs/Float64     目标角
  输出  /parking/fault_inject       std_msgs/String      srv OP_FAULT 的注入通道
  输出  /parking/gate_state         BarrierState         转发 + 按手动/自动改写 source
  输出  /parking/pass               ParkingPass          一次通行一条（进场/出场各一条）
  输出  /parking/actions            ParkingAction        每一次抬/落/保持/故障/手动一条
  服务  /parking/gate               SetParkingGate       手动控制与故障注入（有回包）
  服务  /parking/whitelist          SetParkingWhitelist  白名单/临时卡热改

为什么所有判定都吃 BarrierState 的实测值，而不是本节点自己数定时器：
  条目 3 要的是"真实物理碰撞与升降动力学"。抬没抬到位、有没有砸到车，插件已经从
  joint 的实测角/角速度/力矩里判出来了 —— tools/test_barrier_dyn.py 两道闸各 17 条
  断言全过（抬杆 0.85 s、落杆 0.65 s、超时阈值 5 s 余量 5.9 倍；杆压到车 1 s 内
  测出堵转并把目标角归到抬起）。本节点再独立数一遍时间，就会造出"节点说抬了、
  杆还躺着"的第二套真相，取证时两条对不上。

进出两道闸的地感都**跨在杆轴上**，这一点是被实测逼出来的（原入口线圈放在杆轴北侧
  0.70 m）：占用框半边 (长+宽)/2 + 半线圈 = 1.60 m，线圈放在 -10.35 时占用框南缘在
  -11.95，而核验位 -12.21 的车要往前爬 0.26 m 才检出 —— 按巡航 0.25 m/s 至少 1.3 s，
  比 hold_time_s 1.5 s 慢，所以"到点且地感始终空闲"的分支先落了杆，杆随即砸在正在过杆
  的车头上（实测：clear_s 永远 -1、车顶在放平的杆上原地空转，整条车道 deadlock）。
  现在两道闸都让占用框盖住"杆下危险区"并两侧各留 arrive_radius：车头距杆还有 0.50 m
  就已占用，整条车尾越过杆 0.90 m 才释放。于是"离场"只有一个判据 —— 占用→释放的下降沿；
  从未占用不再是"落杆"的理由，只是"再等 noshow_s 确认车根本没往前走"的理由。
  两者的框都是插件那个刻意放大的外接方框，漏判会落杆砸车，多判只是晚一点落杆。
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, String

from v2x_msgs.msg import (BarrierState, ParkingAction, ParkingPass,
                          PlateCapture, PlateResult)
from v2x_msgs.srv import SetParkingGate, SetParkingWhitelist

KIND_NAME = {ParkingPass.KIND_MONTHLY: '月租', ParkingPass.KIND_TEMP: '临时',
             ParkingPass.KIND_UNKNOWN: '未登记', ParkingPass.KIND_BLACKLIST: '黑名单'}
RESULT_NAME = {ParkingPass.RESULT_ALLOWED: '放行', ParkingPass.RESULT_DENIED: '拒绝',
               ParkingPass.RESULT_PENDING: '挂起'}
FAULT_NAME = {BarrierState.FAULT_NONE: 'NONE', BarrierState.FAULT_STUCK: 'STUCK',
              BarrierState.FAULT_RAISE_FAIL: 'RAISE_FAIL',
              BarrierState.FAULT_NO_CLOSE: 'NO_CLOSE',
              BarrierState.FAULT_SENSOR_FAIL: 'SENSOR_FAIL'}
STATE_NAME = {BarrierState.STATE_CLOSED: 'CLOSED', BarrierState.STATE_OPENING: 'OPENING',
              BarrierState.STATE_OPEN: 'OPEN', BarrierState.STATE_CLOSING: 'CLOSING',
              BarrierState.STATE_FAULT: 'FAULT'}


def stamp_of(sec):
    t = Time()
    t.sec = int(sec)
    t.nanosec = int(round((sec - int(sec)) * 1e9)) % 1_000_000_000
    return t


def now_of(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Cycle:
    """一次在途通行。四个时刻（到场/抬到位/离场/落到位）填进同一条 ParkingPass，
    所以放行那条记录在落杆完成时才发；拒绝那条当场就发（后三段永远是 -1）。"""

    def __init__(self, g, cap, ts):
        self.gate = g
        self.direction = g.direction
        self.vehicle_id = cap.vehicle_id
        self.capture_id = cap.capture_id
        self.image_path = cap.image_path
        self.pose = cap.pose
        self.arrive_s = ts
        self.plate = ''
        self.recognized = False
        self.confidence = 0.0
        self.kind = ParkingPass.KIND_UNKNOWN
        self.result = ParkingPass.RESULT_PENDING
        self.reason = '等待识别结果'
        self.ticket_id = ''
        self.whitelist_hit = 0
        self.verified = False            # 已核验并下发抬杆（之后就不该再吃核验超时）
        self.raise_s = -1.0
        self.clear_s = -1.0
        self.lower_s = -1.0
        self.lowering = False            # 已下发落杆，别在 tick 里反复下指令
        # 开到位之后"这辆车确实压过地感"的证据。出口闸的车在核验位就已经压在框内，
        # 所以初值直接取当前读数，不能等上升沿 —— 等上升沿落杆条件永远不成立。
        self.occ_seen = g.st.loop_occupied if g.st else False
        self.released_at = None          # 最近一次"占用→释放"的仿真时刻
        self.held_logged = False         # ACT_HOLD_OPEN 每次占用只发一条
        self.degraded = False


class Gate:
    def __init__(self, g, pk):
        self.id = g['id']
        self.direction = g['direction']              # 'in' / 'out'
        self.label = g.get('label', g['id'])
        self.st = None
        self.mode = 'auto'                           # 'auto' / 'manual'
        self.cycle = None
        self.fault_latched = False
        self.degraded = False                        # 地感失效 → 计时放行
        # 故障清除那一拍如果杆下压着车，落杆会被防砸挡回。这个时刻记下"欠一发收尾落杆"，
        # 由 tick 在地感释放后补上（见 tick 里 c is None 那段），否则一次故障演出就把车道
        # 永久敞开，直到下一辆车自己触发一次核验。
        self.pending_clear_at = None
        bar = pk['barrier']
        self.open_rad = float(bar['open_angle'])
        self.closed_rad = float(bar['closed_angle'])
        self.hold_s = float(bar['hold_time_s'])
        self.clearance_s = float(bar['clearance_delay_s'])
        self.refuse = bool(bar['refuse_lower_while_occupied'])
        self.verify_timeout_s = float(pk['control']['verify_timeout_s'])
        # 降级放行的等待：地感读数不可信时"车已过杆"只能按时间保证，取"从核验位按
        # 巡航速度开到占用框远端"的时间再 +3 s。框半边 (长+宽)/2=1.20 m 与插件同一个式子。
        hx = 0.5 * (float(pk['vehicle']['length']) + float(pk['vehicle']['width']))
        lcx, lcy = [float(v) for v in g['loop']['center']]
        lsx, lsy = [float(v) for v in g['loop']['size']]
        qy = float(g['queue_point'][1])
        half_y = hx + 0.5 * lsy
        far = lcy + half_y if self.direction == 'in' else lcy - half_y
        cruise = float(pk['shuttle']['cruise_speed'])
        self.degrade_wait_s = abs(far - qy) / max(cruise, 0.05) + 3.0
        self.occ = (lcx, lcy, hx + 0.5 * lsx, half_y)
        # "地感始终空闲"到点之后还要再等这么久才允许落杆：按"从核验位开到占用框靠车
        # 来的那一侧"再 +3 s 算。跨过杆轴的线圈让正常通行一定先占用，所以走到这条分支
        # 只可能是车压根没从核验位往前走 —— 那时杆下必然空着，落杆才安全。
        near = lcy - half_y if self.direction == 'in' else lcy + half_y
        self.noshow_s = abs(qy - near) / max(cruise, 0.05) + 3.0


class ParkingManagerNode(Node):
    def __init__(self, **kw):
        # **kw 让 parameter_overrides 能透传给 rclpy：自测脚本要直接指定源码树里的
        # parking_zones.yaml，不必等 colcon 把配置装进 share 目录。
        super().__init__('parking_manager', **kw)
        self.declare_parameter('config_file', '')
        # use_sim_time 由 rclpy 自己声明，这里再 declare 会抛 ParameterAlreadyDeclared。
        path = self.get_parameter('config_file').value or os.path.join(
            get_package_share_directory('v2x_demo'), 'config', 'parking_zones.yaml')
        pk = self.load_config(path)
        self.pk = pk
        self.gates = {g['id']: Gate(g, pk) for g in pk['gates']}
        self.sim_now = 0.0
        self.last_clock = None
        # 通行编号必须跨 manager 重启接着往下数，见 resume_pass_seq。
        self.seq_pass = self.resume_pass_seq()
        self.seq_act = 0
        self.captures = {}               # capture_id -> (PlateCapture, 抓拍仿真秒)
        self.tickets = {}                # ticket_id -> {plate, issued_s, ttl, label}
        self.whitelist = {}              # plate -> {'kind': uint8, 'label': str}
        for plate, v in (pk['control'].get('whitelist') or {}).items():
            self.whitelist[plate] = {'kind': ParkingPass.KIND_MONTHLY,
                                     'label': (v or {}).get('label', '月租车')}
        self.whitelist_base = dict(self.whitelist)
        self.blacklist = set(pk['control'].get('blacklist') or [])
        self.unknown_as_temp = bool(pk['control']['unknown_as_temp'])
        self.temp_ttl = float(pk['control']['temp_ticket_ttl_s'])

        ctl = pk['control']
        qos = QoSProfile(depth=50)
        self.pub_state = self.create_publisher(BarrierState, ctl['state_topic'], qos)
        self.pub_pass = self.create_publisher(ParkingPass, ctl['pass_topic'], qos)
        self.pub_act = self.create_publisher(ParkingAction, ctl['action_topic'], qos)
        self.pub_fault = self.create_publisher(String, ctl['fault_topic'], qos)
        self.cmd_pub = {gid: self.create_publisher(
            Float64, '/parking/gate/%s/cmd' % gid, qos) for gid in self.gates}
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for gid in self.gates:
            self.create_subscription(BarrierState, '/parking/gate/%s/state' % gid,
                                     lambda m, k=gid: self.on_state(k, m), qos)
        self.create_subscription(PlateCapture, '/plate/capture', self.on_capture, qos)
        self.create_subscription(PlateResult, '/plate/result', self.on_result, qos)
        self.create_subscription(String, ctl['manual_cmd_topic'], self.on_manual, 10)
        self.create_service(SetParkingGate, ctl['gate_srv'], self.on_gate_srv)
        self.create_service(SetParkingWhitelist, ctl['whitelist_srv'], self.on_wl_srv)
        self.create_timer(0.10, self.tick)
        self.create_timer(10.0, self.check_clock)
        self.get_logger().info(
            '[parking] 管控就绪：%s｜白名单 %d 条、黑名单 %d 条、未登记按临时车=%s｜'
            '保持全开 %.1f s、释放确认 %.1f s、防砸联锁=%s｜配置 %s' % (
                '/'.join(self.gates), len(self.whitelist), len(self.blacklist),
                self.unknown_as_temp, self.gates[list(self.gates)[0]].hold_s,
                self.gates[list(self.gates)[0]].clearance_s,
                '开' if self.gates[list(self.gates)[0]].refuse else '关', path))

    @staticmethod
    def load_config(path):
        import yaml
        with open(path, encoding='utf-8') as fh:
            return yaml.safe_load(fh)['parking']

    def resume_pass_seq(self):
        """通行编号跨重启续号：扫一遍记录器那份追加式 JSONL，取已有最大号接着往下数。

        pass_id 的计数器在本进程里（self.seq_pass），账本却在进程外（/tmp/v2x_parking/
        parking_pass.jsonl 只追加、从不重写）。管理器一重启就从 P0001 重数，同一个账本
        里于是出现两条 P0001-gate_in —— 2026-10-14 条目 5 的对账器就是靠这一点报 FAIL 的，
        而按 pass_id 检索/导给人看时，撞号的两条根本分不开。账本轮转过（空文件/无文件）
        时返回 0，第一条照旧是 P0001。
        """
        import json
        rec = self.pk.get('records') or {}
        path = os.path.join(rec.get('dir', '/tmp/v2x_parking'),
                            rec.get('pass_file', 'parking_pass.jsonl'))
        mx, torn = 0, 0
        try:
            with open(path, encoding='utf-8') as fh:
                for ln in fh:
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        pid = json.loads(ln).get('pass_id') or ''
                    except ValueError:
                        torn += 1
                        continue
                    num = pid.partition('-')[0]
                    if num.startswith('P') and num[1:].isdigit():
                        mx = max(mx, int(num[1:]))
        except OSError:
            return 0
        if mx:
            self.get_logger().info('[parking] 通行编号续自现有账本 %s：最大 P%04d，下一个 P%04d%s'
                                  % (os.path.basename(path), mx, mx + 1,
                                     '（跳过 %d 行残缺）' % torn if torn else ''))
        return mx

    # ── 时钟 ──────────────────────────────────────────────────────────────
    def on_clock(self, m):
        self.sim_now = m.clock.sec + m.clock.nanosec * 1e-9
        self.last_clock = time.monotonic()

    def check_clock(self):
        """所有计时阈值都是仿真秒。gzserver 暂停时仿真钟不走，这里只提醒不改状态，
        免得"暂停 20 s 回来后杆自己落下去了"那种无法复现的读数。"""
        if self.last_clock is not None and time.monotonic() - self.last_clock > 5.0:
            self.get_logger().warn(
                '[parking] /clock 已 %.1f s 未更新（gzserver 没起来或带了 -u/--pause），'
                '管控计时随之停摆' % (time.monotonic() - self.last_clock))

    # ── 闸机实测状态 ───────────────────────────────────────────────────────
    def on_state(self, gid, s):
        g = self.gates[gid]
        prev = g.st
        g.st = s
        # 在进任何分支之前先记下"当时有没有在途通行"。on_action_done 会在处理落杆的
        # 过程中把 g.cycle 清空，如果末尾那段用清空之后的值判断，一次自动落杆会被
        # 记两条 ACT_LOWER（动作日志的对账口径是一次抬杆配一次落杆）。
        had_cycle = g.cycle is not None
        # source 表示"这段杆归谁管"，插件只会填 SRC_AUTO；手动占权时在这里改写，
        # 界面与 JSONL 才看得出是人工抬的。
        out = BarrierState()
        for f in s.get_fields_and_field_types():
            setattr(out, f, getattr(s, f))
        out.source = BarrierState.SRC_MANUAL if g.mode == 'manual' else s.source
        self.pub_state.publish(out)
        if prev is None:
            return

        if s.fault != prev.fault:
            self.on_fault_edge(g, s, prev)
        if g.fault_latched:
            return

        if s.at_target and s.state != prev.state:
            act = None
            if s.state == BarrierState.STATE_OPEN:
                act = ParkingAction.ACT_RAISE
            elif s.state == BarrierState.STATE_CLOSED:
                act = ParkingAction.ACT_LOWER
            if act is not None:
                # had_cycle 优先于 mode：在途通行**必须**被这一拍闭合。手动/自动交叠时
                # （演手动落杆正好撞上在途通行）若只按 mode 走下面那条手动分支，
                # g.cycle 永不归零，这道闸之后的每一张抓拍都被"在途未结束"忽略，
                # 摆渡车连废几轮后自己挂停 —— 2026-10-14 10:24 gate_in 就是这么死的。
                if had_cycle:
                    self.on_action_done(
                        g, s, act,
                        BarrierState.SRC_MANUAL if g.mode == 'manual' else s.source)
                elif g.mode == 'manual':
                    self.log_action(g, act, BarrierState.SRC_MANUAL, s.fault,
                                    reached=True, to_rad=s.target_rad,
                                    note='手动目标角到位（实测 %.3f rad，%.2f s）' % (
                                        s.angle_rad, s.action_elapsed_s))
        if (s.state == BarrierState.STATE_CLOSED and s.at_target and not had_cycle and
                prev.state != BarrierState.STATE_CLOSED and g.mode == 'auto'):
            # 没有对应在途通行却落到位 = 手动或复位后的正常收尾，补一条 AUTO 落杆日志，
            # 不然动作日志里"抬了没落"会对不平。故障清除时欠下的那发收尾落杆到这里就清了。
            g.pending_clear_at = None
            self.log_action(g, ParkingAction.ACT_LOWER, BarrierState.SRC_AUTO,
                            s.fault, reached=True, to_rad=s.target_rad,
                            note='无在途通行的落杆（手动或故障恢复后的收尾）')

    def on_action_done(self, g, s, act, src):
        """闭合这次通行。src 由调用方给：手动占权时这一拍归人工，日志与界面要看得出。"""
        c = g.cycle
        if act == ParkingAction.ACT_RAISE:
            if c.result != ParkingPass.RESULT_ALLOWED:
                c.raise_s = self.sim_now
                c.result = ParkingPass.RESULT_ALLOWED
                c.reason = '核验通过，抬杆到位 %.2f s' % s.action_elapsed_s
            self.log_action(g, act, src, s.fault, reached=True,
                            to_rad=s.target_rad,
                            note='%s 开到位，实测 %.3f rad，耗时 %.2f s' % (
                                c.plate or '无牌', s.angle_rad, s.action_elapsed_s))
        else:
            c.lower_s = self.sim_now
            c.reason += '，落杆到位'
            self.log_action(g, act, src, s.fault, reached=True, to_rad=s.target_rad,
                            note='通行结束，杆下 %s' % (s.vehicle_id or '无车'))
            g.cycle = None
            self.emit_pass(c)

    def on_fault_edge(self, g, s, prev):
        """故障位/清除的边沿：注入或自测都从同一条 BarrierState 里看出来。"""
        if s.fault == BarrierState.FAULT_SENSOR_FAIL:
            g.degraded = True
            if g.cycle is not None:
                g.cycle.degraded = True
            self.log_action(g, ParkingAction.ACT_DEGRADE, s.source, s.fault,
                            reached=True, to_rad=s.target_rad,
                            note='地感失效，降级为计时放行（开到位后再等 %.1f s 落杆）'
                                 % g.degrade_wait_s)
            return
        if s.fault != BarrierState.FAULT_NONE:
            g.fault_latched = True
            self.log_action(g, ParkingAction.ACT_FAULT, s.source, s.fault,
                            reached=False, to_rad=s.target_rad,
                            note='%s：实测角 %.3f 目标 %.2f τ=%.0f N·m 本次动作已 %.2f s'
                                 % (FAULT_NAME[s.fault], s.angle_rad, s.target_rad,
                                    s.effort_nm, s.action_elapsed_s))
            c = g.cycle
            if c is not None and c.raise_s < 0.0:
                c.reason = '核验已下发抬杆，但闸机故障未到位（%s）' % FAULT_NAME[s.fault]
                self.emit_pass(c)
                g.cycle = None
            return
        g.fault_latched = False
        g.degraded = False
        # 机械故障是插件"安全侧保持抬起"停住的，落杆指令当时已被挡下（lowering=True）。
        # 清除后必须把 lowering 归零让 tick 重新下发，否则这次通行永远卡在"已下发落杆"
        # 的状态里 —— 杆一直抬着，下一辆车进不来。
        if g.cycle is not None:
            g.cycle.lowering = False
        self.log_action(g, ParkingAction.ACT_FAULT_CLEAR, s.source,
                        BarrierState.FAULT_NONE, reached=True, to_rad=s.target_rad,
                        note='故障清除，恢复自动（当前 %s）' % STATE_NAME[s.state])
        self.after_clear(g, s)

    def after_clear(self, g, s):
        """故障清除后的收尾。插件的安全侧把杆停在**抬起**位，人工清障之后车道是敞开的；
        此刻如果没有在途通行（故障发生时那条已经按 PENDING 记账并清掉了），必须让杆
        自动回到落下位，否则一次故障演出就把车道永久放开。仍然吃防砸联锁。"""
        if g.cycle is not None or g.mode != 'auto':
            return
        if s.target_rad < 0.5 * (g.open_rad + g.closed_rad):
            return                    # 杆本来就在落下位，不用管
        if s.loop_occupied and g.refuse and not g.degraded:
            g.pending_clear_at = self.sim_now
            self.log_action(g, ParkingAction.ACT_REFUSE, BarrierState.SRC_AUTO,
                            s.fault, reached=False, to_rad=g.closed_rad,
                            note='故障清除后要落杆，但地感 %s 仍被 %s 压着（车离杆后自动补落杆）'
                                 % (s.loop_id, s.vehicle_id or '未知车'))
            return
        g.pending_clear_at = None
        self.command(g, g.closed_rad, BarrierState.SRC_AUTO, ParkingAction.ACT_LOWER_CMD,
                     note='故障清除后收尾：无在途通行且杆下 %s，落杆复位'
                          % (s.vehicle_id or '空闲'))

    # ── 抓拍与识别 ─────────────────────────────────────────────────────────
    def on_capture(self, cap):
        g = self.gates.get(cap.gate_id)
        if g is None:
            return                        # 任务三的两道卡口，与停车场无关
        ts = now_of(cap)
        self.captures[cap.capture_id] = (cap, ts)
        if len(self.captures) > 200:      # 只留最近 100 条，防长跑涨内存
            for k in sorted(self.captures, key=lambda k: self.captures[k][1])[:-100]:
                self.captures.pop(k)
        if g.cycle is not None:
            self.get_logger().warn('[parking] %s 在途通行未结束又收到抓拍 %s，忽略'
                                   % (g.id, cap.capture_id))
            return
        if g.fault_latched:
            self.get_logger().warn('[parking] %s 故障挂起中，不受理核验（抓拍 %s）'
                                   % (g.id, cap.capture_id))
            return
        g.cycle = Cycle(g, cap, ts)
        # 新通行接管这道闸，之前欠的收尾落杆就不必再补（补了会把刚抬起来迎车的杆按回去）。
        g.pending_clear_at = None
        self.get_logger().info('[parking] %s 到场 %s（%s 向，抓拍 %s → %s）' % (
            g.id, cap.vehicle_id, g.direction, cap.capture_id, cap.image_path))

    def on_result(self, r):
        got = self.captures.get(r.capture_id)
        if got is None:
            return
        cap, _ = got
        g = self.gates.get(cap.gate_id)
        if g is None or g.cycle is None or g.cycle.capture_id != r.capture_id:
            return
        c = g.cycle
        c.plate = r.plate_number
        c.recognized = bool(r.plate_number)
        c.confidence = float(r.confidence)
        self.verify(g, c, r.algorithm)

    def verify(self, g, c, algorithm):
        """名单判定 + 抬杆。三种结局：放行（抬杆）/ 拒绝（不动杆）/ 挂起（等人工）。"""
        now = self.sim_now
        plate = c.plate
        if plate in self.blacklist:
            c.kind = ParkingPass.KIND_BLACKLIST
            c.whitelist_hit = 3
            c.reason = '命中黑名单，拒绝抬杆'
            self.reject(g, c)
            return
        if not plate:
            c.kind = ParkingPass.KIND_UNKNOWN
            c.reason = '识别无结果（algorithm=%s），等待人工核验或手动抬杆' % algorithm
            self.reject(g, c, ParkingPass.RESULT_PENDING)
            return
        entry = self.whitelist.get(plate)
        if entry:
            c.kind, c.whitelist_hit = entry['kind'], 1
            if c.kind == ParkingPass.KIND_TEMP and not c.ticket_id:
                c.ticket_id = entry['label']       # 人工 OP_ISSUE_TEMP 登记的卡号
        else:
            tid = self.live_ticket(plate, now)
            if tid:
                c.kind, c.ticket_id, c.whitelist_hit = ParkingPass.KIND_TEMP, tid, 2
            elif self.unknown_as_temp and g.direction == 'in':
                c.kind, c.whitelist_hit = ParkingPass.KIND_TEMP, 2
                c.ticket_id = self.issue_temp(plate, now, '未登记车牌，入口按临时车发卡')
            elif g.direction == 'out':
                c.kind, c.whitelist_hit = ParkingPass.KIND_UNKNOWN, 2
                c.reason = '无入场记录（名单外且无在途临时卡），拒绝出场'
                self.reject(g, c)
                return
            else:
                c.kind, c.whitelist_hit = ParkingPass.KIND_UNKNOWN, 2
                c.reason = '未登记车牌且 unknown_as_temp=false，拒绝进场'
                self.reject(g, c)
                return
        if g.mode == 'manual':
            # 人工占权期间自动流程让位：核验结论照样记进 ParkingPass（名单命中是真的），
            # 但杆归人管，不能和手动按钮抢同一个 joint 的目标角。
            c.verified = True
            c.reason = '核验通过（%s），但闸机处于人工占权状态，自动流程不下发抬杆' % \
                KIND_NAME[c.kind]
            self.reject(g, c, ParkingPass.RESULT_PENDING)
            return
        c.result = ParkingPass.RESULT_PENDING
        c.verified = True
        self.get_logger().info('[parking] %s %s(%s) 核验通过 → 抬杆%s' % (
            g.id, plate, KIND_NAME[c.kind], ' 卡=%s' % c.ticket_id if c.ticket_id else ''))
        self.command(g, g.open_rad, BarrierState.SRC_AUTO, ParkingAction.ACT_RAISE_CMD,
                     note='%s %s%s 核验通过' % (
                         plate, KIND_NAME[c.kind],
                         '（卡 %s）' % c.ticket_id if c.ticket_id else ''))

    def reject(self, g, c, result=ParkingPass.RESULT_DENIED):
        # result 必须在这里显式写：ParkingPass.result 的默认值是 RESULT_ALLOWED=0，
        # 不写就等于把一次"拒绝放行"记成"已放行"。三种不动杆的结局口径不同：
        # 黑名单/名单外是 DENIED，识别空结果与核验超时是 PENDING（等人），
        # 人工占权让位也是 PENDING（结论没问题，只是杆归人管）。
        c.result = result
        c.raise_s = c.clear_s = c.lower_s = -1.0
        self.log_action(g, ParkingAction.ACT_REFUSE, BarrierState.SRC_AUTO,
                        g.st.fault if g.st else BarrierState.FAULT_NONE,
                        reached=False, to_rad=g.st.target_rad if g.st else g.closed_rad,
                        note='%s：%s' % (c.plate or '未识别', c.reason))
        self.emit_pass(c)
        g.cycle = None
        self.get_logger().warn('[parking] %s 拒绝放行：%s（%s）'
                               % (g.id, c.reason, c.plate or '无牌'))

    def live_ticket(self, plate, now):
        for tid, t in list(self.tickets.items()):
            if t['plate'] != plate:
                continue
            if now - t['issued_s'] > t['ttl']:
                self.tickets.pop(tid)
                if self.whitelist.get(plate, {}).get('label') == tid:
                    self.whitelist.pop(plate, None)
                self.get_logger().info('[parking] 临时卡 %s 超时作废（%s）' % (tid, plate))
                continue
            return tid
        return None

    def issue_temp(self, plate, now, note):
        tid = 'T-%d' % int(round(now * 1000))
        self.tickets[tid] = {'plate': plate, 'issued_s': now, 'ttl': self.temp_ttl,
                             'label': note}
        self.get_logger().info('[parking] 发临时卡 %s 给 %s（有效期 %.0f s，%s）'
                               % (tid, plate, self.temp_ttl, note))
        return tid

    # ── 抬杆后的通行判定（0.1 s 一拍）──────────────────────────────────────
    def tick(self):
        now = self.sim_now
        if now <= 0.0:
            return
        for g in self.gates.values():
            c = g.cycle
            if c is None:
                # 没有对应在途通行的那道闸平时不用管，只有一种情况要管：故障清除那一拍
                # 被防砸挡回的落杆。这里在地感释放后把它补上 —— 不然 tick 一见 c is None
                # 就跳过，after_clear 承诺的"不等人工自动落回"就成了空话。
                pend = g.pending_clear_at
                if (pend is not None and g.mode == 'auto' and not g.fault_latched and
                        g.st is not None and not g.st.loop_occupied and
                        g.st.state != BarrierState.STATE_CLOSED and
                        now - pend >= g.clearance_s):
                    self.command(g, g.closed_rad, BarrierState.SRC_AUTO,
                                 ParkingAction.ACT_LOWER_CMD,
                                 note='故障清除后的收尾落杆：地感已释放 %.2f s'
                                      % (now - pend))
                    g.pending_clear_at = None
                continue
            if g.st is None or g.fault_latched:
                continue
            # 核验超时：识别节点没起来、或外部算法不回包时，不能让这条通行永远挂着。
            if (not c.verified and c.result == ParkingPass.RESULT_PENDING and
                    now - c.arrive_s > g.verify_timeout_s):
                c.reason = '%.1f s 内没有识别结果（识别节点未在线或算法未回包）' % \
                    g.verify_timeout_s
                self.reject(g, c, ParkingPass.RESULT_PENDING)
                continue
            if c.result != ParkingPass.RESULT_ALLOWED or c.lowering:
                continue
            s = g.st
            if s.state != BarrierState.STATE_OPEN or not s.at_target:
                continue                  # 还在抬（实测 0.85 s），或已被车顶起来
            if g.degraded:
                if now - c.raise_s > g.degrade_wait_s:
                    c.lowering = True
                    self.command(g, g.closed_rad, BarrierState.SRC_AUTO,
                                 ParkingAction.ACT_LOWER_CMD,
                                 note='计时放行：开到位 %.1f s，地感不可信'
                                      % (now - c.raise_s))
                continue
            if not c.occ_seen and s.loop_occupied:
                c.occ_seen = True         # 车头压上地感
                c.held_logged = False
            if c.occ_seen:
                if s.loop_occupied:
                    c.released_at = None
                    if now - c.raise_s >= g.hold_s and not c.held_logged:
                        c.held_logged = True
                        self.log_action(g, ParkingAction.ACT_HOLD_OPEN,
                                        BarrierState.SRC_AUTO, s.fault, reached=True,
                                        to_rad=s.target_rad,
                                        note='保持全开：地感 %s 正被 %s 压着'
                                             % (s.loop_id, s.vehicle_id or '未知车'))
                    continue              # 还没离场
                if c.released_at is None:
                    c.released_at = now
                    c.clear_s = now       # 下降沿就是"车尾离开杆下"，条目 3 的离场检测
                    continue
                if now - c.released_at >= g.clearance_s and now - c.raise_s >= g.hold_s:
                    c.lowering = True
                    self.command(g, g.closed_rad, BarrierState.SRC_AUTO,
                                 ParkingAction.ACT_LOWER_CMD,
                                 note='地感释放 %.2f s 后落杆（开到位已 %.2f s）'
                                      % (now - c.released_at, now - c.raise_s))
            elif now - c.raise_s >= max(g.hold_s, g.noshow_s):
                # 从未压上地感。线圈现在跨在杆轴上，正常过杆一定先占用，所以这里只剩
                # 一种解释：车没从核验位往前走。上一版在这里按 hold_s 到点就落，实测
                # 把杆砸在了正在爬的车头上（0.26 m 的检出死区比 1.5 s 的保持窗还短）。
                c.lowering = True
                self.command(g, g.closed_rad, BarrierState.SRC_AUTO,
                             ParkingAction.ACT_LOWER_CMD,
                             note='开到位 %.1f s 且地感始终空闲（等车走窗 %.1f s 已过）：'
                                  '判定车未过杆，落杆' % (now - c.raise_s, g.noshow_s))

    # ── 下发目标角 ─────────────────────────────────────────────────────────
    def command(self, g, rad, source, act, note=''):
        m = Float64()
        m.data = float(rad)
        self.cmd_pub[g.id].publish(m)
        self.log_action(g, act, source, g.st.fault if g.st else BarrierState.FAULT_NONE,
                        reached=False, to_rad=rad, note=note or '下发目标角')
        self.get_logger().info('[parking] %s 下发目标角 %.2f rad（%s）%s' % (
            g.id, rad, '手动' if source == BarrierState.SRC_MANUAL else '自动', note))

    def log_action(self, g, act, source, fault, reached, to_rad=None, note=''):
        s, c = g.st, g.cycle
        a = ParkingAction()
        self.seq_act += 1
        a.action_id = 'A%05d-%s' % (self.seq_act, g.id)
        a.header.stamp = stamp_of(self.sim_now)
        a.header.frame_id = 'world'
        a.action = act
        a.gate_id = g.id
        a.vehicle_id = c.vehicle_id if c else (s.vehicle_id if s else '')
        a.plate_number = c.plate if c else ''
        a.source = source
        a.fault = fault
        a.from_rad = s.angle_rad if s else 0.0
        a.to_rad = g.closed_rad if to_rad is None else float(to_rad)
        a.elapsed_s = s.action_elapsed_s if s else 0.0
        a.reached = reached
        a.note = note
        self.pub_act.publish(a)

    def emit_pass(self, c):
        p = ParkingPass()
        self.seq_pass += 1
        p.pass_id = 'P%04d-%s' % (self.seq_pass, c.gate.id)
        p.header.stamp = stamp_of(self.sim_now)
        p.header.frame_id = 'world'
        p.direction = ParkingPass.DIR_IN if c.direction == 'in' else ParkingPass.DIR_OUT
        p.gate_id = c.gate.id
        p.plate_number = c.plate
        p.recognized = c.recognized
        p.plate_confidence = round(c.confidence, 4)
        p.capture_id = c.capture_id
        p.image_path = c.image_path
        p.vehicle_id = c.vehicle_id
        p.pose = c.pose
        p.kind = c.kind
        p.result = c.result
        p.reason = c.reason
        p.ticket_id = c.ticket_id
        p.whitelist_hit = c.whitelist_hit
        p.arrive_s = round(c.arrive_s, 3)
        p.raise_s = round(c.raise_s, 3)
        p.clear_s = round(c.clear_s, 3)
        p.lower_s = round(c.lower_s, 3)
        self.pub_pass.publish(p)
        self.settle_ticket(c)
        self.get_logger().info(
            '[parking] %s %s 牌=%s %s %s｜到场 %.2f 抬到位 %.2f 离场 %.2f 落到位 %.2f｜%s'
            % (p.pass_id, '进场' if p.direction == ParkingPass.DIR_IN else '出场',
               p.plate_number or '无', KIND_NAME[p.kind], RESULT_NAME[p.result],
               p.arrive_s, p.raise_s, p.clear_s, p.lower_s, p.reason))

    def settle_ticket(self, c):
        """出场放行即核销临时卡（SetParkingWhitelist 的注释就是这么承诺的）。不核销的话
        `tickets` 列表会一直涨，而更糟的是同一张卡在 temp_ticket_ttl_s（默认 1 h）内
        可以被下一辆未登记车反复复用成"已在场"，出场对账就再也对不上了。"""
        if not (c.ticket_id and c.direction == 'out' and
                c.result == ParkingPass.RESULT_ALLOWED):
            return
        t = self.tickets.pop(c.ticket_id, None)
        e = self.whitelist.get(c.plate)
        if e and e['kind'] == ParkingPass.KIND_TEMP and e['label'] == c.ticket_id:
            self.whitelist.pop(c.plate, None)
        self.get_logger().info('[parking] 临时卡 %s 已核销（%s%s）' % (
            c.ticket_id, c.plate, '' if t else '，注：卡已不在途，按车牌清名单'))

    # ── 手动控制（话题）────────────────────────────────────────────────────
    def on_manual(self, m):
        text = m.data.strip()
        if ':' not in text:
            self.get_logger().warn('[parking] 手动指令应为 gate_id:角度 或 gate_id:AUTO，'
                                   '收到 %r' % text)
            return
        gid, val = [x.strip() for x in text.split(':', 1)]
        g = self.gates.get(gid)
        if g is None:
            self.get_logger().warn('[parking] 手动指令里的闸机 %s 不存在（现有 %s）'
                                   % (gid, '/'.join(self.gates)))
            return
        if val.upper() in ('AUTO', 'CLEAR'):
            g.mode = 'auto'
            self.log_action(g, ParkingAction.ACT_MANUAL, BarrierState.SRC_MANUAL,
                            g.st.fault if g.st else BarrierState.FAULT_NONE,
                            reached=True, note='撤销手动占权，交还自动')
            self.get_logger().info('[parking] %s 交还自动' % g.id)
            return
        try:
            rad = float(val)
        except ValueError:
            self.get_logger().warn('[parking] 手动角度无法解析：%r' % val)
            return
        self.manual(g, rad, '话题 %s' % text)

    def manual(self, g, rad, src_note):
        """手动抬落杆，返回是否真的下发了。唯一的硬约束还是防砸：杆下压着车时不许
        手动落 —— 真实闸机的手动按钮也受地感联锁，否则条目 4 的"手动控制"就成了
        砸车开关。返回 False 时 srv 回包必须写清被挡回，不能报"已下发"。"""
        g.mode = 'manual'
        want_open = rad >= 0.5 * (g.open_rad + g.closed_rad)
        if (not want_open and g.refuse and not g.degraded and g.st is not None and
                g.st.loop_occupied):
            self.log_action(g, ParkingAction.ACT_REFUSE, BarrierState.SRC_MANUAL,
                            g.st.fault, reached=False, to_rad=rad,
                            note='手动落杆被防砸联锁挡回：地感 %s 正被 %s 压着'
                                 % (g.st.loop_id, g.st.vehicle_id or '未知车'))
            self.get_logger().warn('[parking] %s 手动落杆被挡（杆下有车）' % g.id)
            return False
        if g.cycle is not None and not want_open:
            g.cycle.reason += '（%s 落杆）' % src_note
            g.cycle.lowering = True
        self.command(g, rad, BarrierState.SRC_MANUAL, ParkingAction.ACT_MANUAL,
                     note='手动下发：%s' % src_note)
        return True

    # ── 服务：闸机 ─────────────────────────────────────────────────────────
    def on_gate_srv(self, req, resp):
        targets = list(self.gates) if not req.gate_id else [req.gate_id]
        bad = [t for t in targets if t not in self.gates]
        if bad:
            resp.success = False
            resp.message = '没有这道闸机：%s（现有 %s）' % (
                ','.join(bad), '/'.join(self.gates))
            return resp
        if req.op == SetParkingGate.Request.OP_LIST:
            resp.success = True
            resp.message = '闸机 %d 道：%s' % (len(self.gates), '、'.join(self.gates))
        elif req.op in (SetParkingGate.Request.OP_RAISE, SetParkingGate.Request.OP_LOWER):
            done, refused = [], []
            for gid in targets:
                g = self.gates[gid]
                if req.op == SetParkingGate.Request.OP_RAISE:
                    rad = g.open_rad if abs(req.angle_rad + 999.0) < 1e-6 \
                        else float(req.angle_rad)
                else:
                    rad = g.closed_rad
                if self.manual(g, rad, '服务 %s' % (
                        'OP_RAISE' if req.op == SetParkingGate.Request.OP_RAISE else 'OP_LOWER')):
                    done.append('%s→%.2f rad' % (gid, rad))
                else:
                    refused.append(gid)
            resp.success = not refused
            resp.message = '已下发 %s%s' % ('、'.join(done) or '无',
                                           '；被防砸联锁挡回 %s' % '、'.join(refused)
                                           if refused else '')
        elif req.op == SetParkingGate.Request.OP_AUTO:
            for gid in targets:
                self.gates[gid].mode = 'auto'
            resp.success = True
            resp.message = '%s 已交还自动控制' % ','.join(targets)
        elif req.op in (SetParkingGate.Request.OP_FAULT, SetParkingGate.Request.OP_CLEAR_FAULT):
            kinds = {BarrierState.FAULT_STUCK: 'STUCK',
                     BarrierState.FAULT_RAISE_FAIL: 'RAISE_FAIL',
                     BarrierState.FAULT_NO_CLOSE: 'NO_CLOSE',
                     BarrierState.FAULT_SENSOR_FAIL: 'SENSOR_FAIL'}
            kind = 'NONE' if req.op == SetParkingGate.Request.OP_CLEAR_FAULT \
                else kinds.get(req.fault_kind, 'NONE')
            if req.op == SetParkingGate.Request.OP_FAULT and kind == 'NONE':
                resp.success = False
                resp.message = 'fault_kind 不是已知故障位：%d' % req.fault_kind
                return self.gate_snapshot(resp)
            for gid in targets:
                m = String()
                m.data = '%s:%s' % (gid, kind)
                self.pub_fault.publish(m)
            resp.success = True
            resp.message = '故障通道已下发 %s（%s）' % (kind, ','.join(targets))
        else:
            resp.success = False
            resp.message = '未知 op=%d' % req.op
        return self.gate_snapshot(resp)

    def gate_snapshot(self, resp):
        """回包里带上每道闸的结构化状态：取证时不用再去 echo 话题猜有没有生效。"""
        for gid in self.gates:
            s = self.gates[gid].st
            if s is None:
                resp.lines.append('%s 还没收到状态（查 gzserver 是否带着 libv2x_barrier.so'
                                  '起来了、世界是不是暂停的）' % gid)
                continue
            resp.states.append(s)
            resp.lines.append('%s %s fault=%s 角=%.3f 目标=%.2f 地感=%s(%s) '
                              '抬%d 落%d mode=%s' % (
                                  gid, STATE_NAME[s.state], FAULT_NAME[s.fault],
                                  s.angle_rad, s.target_rad,
                                  '占用' if s.loop_occupied else '空闲',
                                  s.vehicle_id or '-', s.raise_count, s.lower_count,
                                  self.gates[gid].mode))
        return resp

    # ── 服务：白名单/临时卡 ────────────────────────────────────────────────
    def on_wl_srv(self, req, resp):
        plates, kinds, labels = (list(req.plate_numbers), list(req.kinds),
                                 list(req.labels))
        resp.success = True
        if req.op == SetParkingWhitelist.Request.OP_RESET:
            self.whitelist = dict(self.whitelist_base)
            self.tickets.clear()
            resp.message = '已回到 YAML 基线（%d 条），运行期临时卡全部作废' % \
                len(self.whitelist)
        elif req.op == SetParkingWhitelist.Request.OP_ADD:
            if len(plates) != len(kinds):
                resp.success = False
                resp.message = 'plate_numbers 与 kinds 长度不一致（%d/%d）' % (
                    len(plates), len(kinds))
                return self.wl_fill(resp)
            for i, p in enumerate(plates):
                self.whitelist[p] = {'kind': kinds[i],
                                     'label': labels[i] if i < len(labels) else ''}
            resp.message = '已写入 %d 条' % len(plates)
        elif req.op == SetParkingWhitelist.Request.OP_REMOVE:
            gone = [p for p in plates if self.whitelist.pop(p, None) is not None]
            resp.message = '删除 %d 条：%s' % (len(gone), ','.join(gone) or '无')
        elif req.op == SetParkingWhitelist.Request.OP_ISSUE_TEMP:
            if not plates:
                resp.success = False
                resp.message = 'OP_ISSUE_TEMP 需要 plate_numbers[0]'
                return self.wl_fill(resp)
            note = labels[0] if labels else '人工登记'
            tid = self.issue_temp(plates[0], self.sim_now, note)
            self.whitelist[plates[0]] = {'kind': ParkingPass.KIND_TEMP, 'label': tid}
            resp.message = '已发临时卡 %s 给 %s（%s）' % (tid, plates[0], note)
        elif req.op == SetParkingWhitelist.Request.OP_REVOKE_TEMP:
            hit = []
            for p in plates:
                for tid in [t for t, v in self.tickets.items() if v['plate'] == p]:
                    self.tickets.pop(tid)
                    hit.append(tid)
                if self.whitelist.get(p, {}).get('kind') == ParkingPass.KIND_TEMP:
                    self.whitelist.pop(p, None)
            resp.message = '已收回临时卡：%s' % (','.join(hit) or '无')
        elif req.op != SetParkingWhitelist.Request.OP_LIST:
            resp.success = False
            resp.message = '未知 op=%d' % req.op
        return self.wl_fill(resp)

    def wl_fill(self, resp):
        resp.whitelist = list(self.whitelist)
        resp.whitelist_kinds = [self.whitelist[p]['kind'] for p in self.whitelist]
        resp.whitelist_labels = [self.whitelist[p]['label'] for p in self.whitelist]
        resp.tickets = list(self.tickets)
        return resp


def main():
    rclpy.init()
    node = ParkingManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
