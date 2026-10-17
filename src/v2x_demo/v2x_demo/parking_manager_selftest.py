#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车场闸机管控节点合成数据自测（不需要 Gazebo，也不 spin）。

闭环长什么样：节点下发 Float64 目标角 → 假闸机按 tools/test_barrier_dyn.py 的**实测**
升降速率（抬 0.85 s / 落 0.67 s）走角 → 按 parking_zones.yaml 与插件**同一个**外接方框
式子（半边 (车长+车宽)/2 + 线圈半尺寸）算地感占用 → 回灌 BarrierState。所以断言的是
"节点在真实动力学时序下会不会做错决定"，不是脚本喂给它一套早就写好的剧本。
其中占用几何那条断言（两道闸的核验位都在占用框内）就是把"等待即占用"这个设计写死成
可复跑的证据 —— 2026-10-13 修入口地感检出死区（#79，落杆砸车）之后入口核验位也进了框，
实测栈里 gate_in 抬到位那一拍的地感就是"占用(car_6)"，所以"保持全开/防砸重抬"在入口
同样是设计行为；反过来，谁把点位改回框外，这里也会先炸。

覆盖条目 3/4：
  A 月租车进场全程：核验→抬杆→地感占用→保持全开→释放边沿→落杆→一条 ParkingPass，
     四个时刻各自落在实测耗时上、严格单调；抬/落/下发各一条（对账口径一次抬配一次落）。
  B 识别结果为空 → PENDING + ACT_REFUSE，杆不动。
  C 识别节点没回包 → verify_timeout_s 到点收口，同样不动杆、不猜车牌。
  D 黑名单优先于白名单（京B88990 两份名单里都有）。
  E 未登记车牌：入口自动发临时卡放行 → 同一辆车出场凭在途卡放行 → 落杆即核销 →
     第三次出场就是"无入场记录"拒绝。
  F unknown_as_temp=false 时未登记车牌直接拒入。
  G SENSOR_FAIL 降级计时放行：地感读数冻结在"占用"，节点按 degrade_wait_s 到点落杆，
     且落杆那一刻真实占用确实已释放（证明计时放行不是拿车换效率）；clear_s 保持 -1，
     不谎称"检测到了离场"。
  H STUCK 抬杆中途：当场按 PENDING 记账并清在途，故障期间节点不再对杆指手画脚；
     清除后节点自动把安全侧抬起的杆收尾落回，不需要人工再点一次。
  I 防砸联锁：出口闸的核验位本身在地感框内（等待即占用），手动落杆必须被挡回且 srv
     回包 success=False；人工占权期间自动流程让位（不下发抬杆，结论照样记录）。
  J 两个 srv 的全部 op（含未知 op / 未知闸机 / 名单长度不一致 / 无效故障类型）。

用法（WSL）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 src/v2x_demo/v2x_demo/parking_manager_selftest.py [parking_zones.yaml]
不给路径就用 share 目录下已安装的那份（装了就直接测安装产物）。退出码 0=PASS 1=FAIL。
"""
import collections
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from v2x_msgs.msg import BarrierState, ParkingAction, ParkingPass, PlateCapture, PlateResult
from v2x_msgs.srv import SetParkingGate, SetParkingWhitelist

import parking_manager_node as pmn

DT = 0.1                 # 一拍仿真秒；真插件 20 Hz，这里取粗一档，判定全是边沿式的
RAISE_S = 0.85           # tools/test_barrier_dyn.py 实测：抬杆到位耗时（两道闸一致）
LOWER_S = 0.67           # 实测落杆 0.65~0.70
CAR = 'car_6'
MONTHLY = '沪A12345'      # YAML 白名单里就有
BOTH = '京B88990'        # 白名单 + 黑名单，用来验优先级
TEMP = '苏E66666'        # 名单外，走临时车分支
FAILED = []


def check(cond, what, detail=''):
    if cond:
        print('  ok   %s' % what)
    else:
        FAILED.append(what)
        print('  FAIL %s  %s' % (what, detail))


def banner(t):
    print('\n── %s' % t)


class Sink:
    """替掉 ROS 发布器：自测不 spin，直接收节点在回调里 publish 出来的消息。

    两份账：**hist** 只增不清，是"节点一共下发过什么"的对账依据；**msgs** 是待投递
    队列，假闸机每拍 take() 走它。断言一律读 hist —— 早先用 take()/clear() 做断言
    等于把消息从假闸机手里抢走，节点那条抬杆指令根本没送到杆上，测的是"我偷看了
    信箱"而不是"闸机动没动"。
    """

    def __init__(self):
        self.msgs = []
        self.hist = []
        self.base = 0

    def publish(self, m):
        self.msgs.append(m)
        self.hist.append(m)

    def mark(self):
        """打一个场景起点：new() 只回这一场发生的，跨场对账用 hist。"""
        self.base = len(self.hist)

    def new(self):
        return self.hist[self.base:]

    def last(self):
        return self.msgs[-1] if self.msgs else None

    def take(self):
        out, self.msgs = self.msgs, []
        return out

    def clear(self):
        self.msgs = []


def pose_at(x, y):
    return Pose(position=Point(x=x, y=y, z=7.0), orientation=Quaternion(w=1.0))


def clock_at(t):
    c = Clock()
    c.clock.sec = int(t)
    c.clock.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000
    return c


class FakeGate:
    """一台假闸机：受目标角驱动，按 yaml 几何算地感，能演 SENSOR_FAIL/STUCK 两种注入。

    与 libv2x_barrier.so 保持一致的三处行为（都是实测出来的，不是想当然）：
      - 重复目标角不重起计时（插件 `fabs(t - target_) < 1e-6 return`）；
      - SENSOR_FAIL 冻结 loop_occupied/vehicle_id 在注入瞬间的读数；
      - STUCK 就是"杆动不了"：角冻结在注入瞬间，目标角保持不动（安全侧由真插件在
        **测出**故障时才改写，注入态不参与判据）。
    """

    def __init__(self, node, gid, gate_cfg, cruise):
        self.node = node
        self.g = node.gates[gid]
        self.gid = gid
        self.lcx, self.lcy, self.bx, self.by = self.g.occ
        self.loop_id = gate_cfg['loop']['id']
        self.tol = float(node.pk['barrier']['angle_tolerance'])
        self.rate_up = self.g.open_rad / RAISE_S
        self.rate_dn = self.g.open_rad / LOWER_S
        self.queue = [float(v) for v in gate_cfg['queue_point']]
        self.sign = 1.0 if self.g.direction == 'in' else -1.0
        self.cruise = cruise
        self.reset()

    def occupied_at(self, x, y):
        return (abs(x - self.lcx) <= self.bx + 1e-9 and
                abs(y - self.lcy) <= self.by + 1e-9)

    def reset(self, with_car=True):
        self.angle = self.g.closed_rad
        self.target = self.g.closed_rad
        self.action_start = None
        self.completed = True
        self.fault = BarrierState.FAULT_NONE
        self.car = list(self.queue) if with_car else None
        self.moving = False
        self.real_occ = self.occupied_at(*self.car) if self.car else False
        self.occupied = self.real_occ
        self.edges = []                  # [(t, occupied)] 供断言 clear_s 落在下降沿
        self.raise_count = self.lower_count = 0

    # ── 来自节点/注入通道的指令 ────────────────────────────────────────────
    def set_target(self, rad, t):
        if abs(rad - self.target) < 1e-6:
            return
        self.target = float(rad)
        self.action_start = t
        self.completed = False

    def inject(self, kind, t):
        f = {'STUCK': BarrierState.FAULT_STUCK,
             'RAISE_FAIL': BarrierState.FAULT_RAISE_FAIL,
             'NO_CLOSE': BarrierState.FAULT_NO_CLOSE,
             'SENSOR_FAIL': BarrierState.FAULT_SENSOR_FAIL}.get(
            kind, BarrierState.FAULT_NONE)
        self.fault = f
        # 冻结读数就是"注入那一刻地感报什么就一直报什么"
        self.occupied = self.real_occ

    # ── 每拍 ───────────────────────────────────────────────────────────────
    def move(self, t):
        if self.car and self.moving and self.fault != BarrierState.FAULT_STUCK:
            self.car[1] += self.sign * self.cruise * DT
        self.real_occ = self.occupied_at(*self.car) if self.car else False
        occ = self.occupied if self.fault == BarrierState.FAULT_SENSOR_FAIL \
            else self.real_occ
        if occ != self.occupied:
            self.edges.append((t, occ))
        if self.fault != BarrierState.FAULT_SENSOR_FAIL:
            self.occupied = occ
        if self.fault == BarrierState.FAULT_STUCK:
            return                      # 杆定住，但目标角保持（插件的演出）
        if abs(self.target - self.angle) <= self.tol:
            return                      # 已在到位容差内：插件保持，不再走角
        step = (self.rate_up if self.target > self.angle else self.rate_dn) * DT
        if abs(self.target - self.angle) <= step:
            self.angle = self.target
        else:
            self.angle += step if self.target > self.angle else -step

    def snapshot(self, t):
        s = BarrierState()
        s.header.stamp = pmn.stamp_of(t)
        s.header.frame_id = 'world'
        s.gate_id = self.gid
        at = abs(self.target - self.angle) <= self.tol
        mid = 0.5 * (self.g.open_rad + self.g.closed_rad)
        if at:
            s.state = BarrierState.STATE_OPEN if self.angle >= mid \
                else BarrierState.STATE_CLOSED
        else:
            s.state = BarrierState.STATE_OPENING if self.target > self.angle \
                else BarrierState.STATE_CLOSING
        s.at_target = at
        s.source = BarrierState.SRC_AUTO
        s.fault = self.fault
        s.target_rad = self.target
        s.angle_rad = self.angle
        s.angle_error_rad = self.target - self.angle
        s.angvel_rps = 0.0 if (at or self.fault == BarrierState.FAULT_STUCK) \
            else (self.rate_up if self.target > self.angle else -self.rate_dn)
        # 卡滞时插件的实测力矩顶满（判据 A 就靠这个 + 角速度归零）
        s.effort_nm = 285.0 if self.fault == BarrierState.FAULT_STUCK else 12.0
        s.action_elapsed_s = 0.0 if self.action_start is None else t - self.action_start
        s.loop_id = self.loop_id
        s.loop_occupied = self.occupied
        s.vehicle_id = (self.car_name() if self.occupied else '')
        if at and not self.completed:
            self.completed = True
            if self.target >= mid:
                self.raise_count += 1
            else:
                self.lower_count += 1
        s.raise_count = self.raise_count
        s.lower_count = self.lower_count
        return s

    def car_name(self):
        return CAR if self.car else ''


class Harness:
    def __init__(self, node, pk):
        self.node = node
        self.pk = pk
        self.t = 100.0
        self.state = Sink()
        self.passes = Sink()
        self.acts = Sink()
        self.faults = Sink()
        node.pub_state, node.pub_pass, node.pub_act, node.pub_fault = \
            self.state, self.passes, self.acts, self.faults
        self.cmds = {gid: Sink() for gid in node.gates}
        node.cmd_pub = self.cmds
        self.fakes = {gid: FakeGate(node, gid, self.gate_cfg(gid),
                                    float(pk['shuttle']['cruise_speed']))
                      for gid in node.gates}
        self.cap_seq = 0
        for gid, f in self.fakes.items():
            node.gates[gid].st = None
            node.on_state(gid, f.snapshot(self.t))

    def gate_cfg(self, gid):
        for g in self.pk['gates']:
            if g['id'] == gid:
                return g
        raise KeyError(gid)

    def reset(self, clear_lists=True):
        for gid, f in self.fakes.items():
            g = self.node.gates[gid]
            f.reset()
            g.cycle = None
            g.mode = 'auto'
            g.degraded = False
            g.fault_latched = False
            g.pending_clear_at = None
            g.st = None
            self.cmds[gid].clear()
            self.cmds[gid].mark()
            self.node.on_state(gid, f.snapshot(self.t))
        for s in (self.state, self.passes, self.acts, self.faults):
            s.clear()
            s.mark()
        if clear_lists:
            self.node.whitelist = dict(self.node.whitelist_base)
            self.node.tickets.clear()
            self.node.blacklist = set()
        self.node.unknown_as_temp = bool(self.pk['control']['unknown_as_temp'])

    def step(self, n=1):
        for _ in range(n):
            self.t += DT
            self.node.on_clock(clock_at(self.t))
            for m in self.faults.take():
                gid, kind = m.data.split(':', 1)
                if gid in self.fakes:
                    self.fakes[gid].inject(kind, self.t)
            for gid, f in self.fakes.items():
                got = self.cmds[gid].take()
                if got:
                    f.set_target(got[-1].data, self.t)
            for gid, f in self.fakes.items():
                f.move(self.t)
                self.node.on_state(gid, f.snapshot(self.t))
            self.node.tick()

    def wait_for(self, pred, max_s, after=None):
        """超时必须报出来：判据不成立时"零事件"和"没等到"是两回事（记忆里那条
        逐条 try/except 吞成零事件假象的坑，这里用显式超时顶回去）。"""
        end = self.t + max_s
        while self.t < end:
            self.step()
            if pred():
                return True
        if after:
            after()
        return False

    # ── 抓拍/识别注入 ──────────────────────────────────────────────────────
    def capture(self, gid):
        f = self.fakes[gid]
        cfg = self.gate_cfg(gid)['camera']
        self.cap_seq += 1
        cap = PlateCapture()
        cap.header.stamp = pmn.stamp_of(self.t)
        cap.capture_id = 'C%04d-%s' % (self.cap_seq, gid)
        cap.gate_id = gid
        cap.vehicle_id = CAR
        cap.pose = pose_at(*f.queue)
        cap.speed = 0.0
        cap.image_width = int(cfg['width'])
        cap.image_height = int(cfg['height'])
        cap.image_encoding = 'bgr8'
        cap.image_path = '%s/%s.png' % (cfg['capture_dir'], cap.capture_id)
        self.node.on_capture(cap)
        return cap

    def result(self, cap, plate, conf=0.93):
        r = PlateResult()
        r.header.stamp = pmn.stamp_of(self.t)
        r.capture_id = cap.capture_id
        r.gate_id = cap.gate_id
        r.capture_stamp = cap.header.stamp
        r.plate_number = plate
        r.confidence = conf
        r.bbox_x, r.bbox_y, r.bbox_w, r.bbox_h = 620, 300, 640, 130
        r.algorithm = 'builtin_opencv'
        r.process_ms = 180.0
        r.vehicle_pose = cap.pose
        r.speed = cap.speed
        r.truth_number = plate
        r.matched = bool(plate)
        self.node.on_result(r)
        return r

    def act_of(self, action):
        return [a for a in self.acts.new() if a.action == action]

    def note_of(self, action):
        return ' | '.join(a.note for a in self.act_of(action))


def call_gate(node, **kw):
    req, resp = SetParkingGate.Request(), SetParkingGate.Response()
    for k, v in kw.items():
        setattr(req, k, v)
    node.on_gate_srv(req, resp)
    return resp


def call_wl(node, **kw):
    req, resp = SetParkingWhitelist.Request(), SetParkingWhitelist.Response()
    for k, v in kw.items():
        setattr(req, k, v)
    node.on_wl_srv(req, resp)
    return resp


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('v2x_demo'), 'config',
                         'parking_zones.yaml')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config',
                        'parking_zones.yaml')


def make_node(cfg_path):
    # parameter_overrides 要的是 rclpy Parameter，其 value 必须是原生 str；
    # 塞 rcl_interfaces 的 ParameterValue 消息会在 declare_parameter 里直接 TypeError。
    return pmn.ParkingManagerNode(parameter_overrides=[
        Parameter(name='config_file', value=cfg_path)])


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else default_config()
    print('配置文件：%s' % cfg)
    rclpy.init()
    node = make_node(cfg)
    pk = node.pk
    h = Harness(node, pk)
    fi, fo = h.fakes['gate_in'], h.fakes['gate_out']
    print('降级等待：入口 %.1f s / 出口 %.1f s｜核验超时 %.1f s' % (
        node.gates['gate_in'].degrade_wait_s, node.gates['gate_out'].degrade_wait_s,
        node.gates['gate_in'].verify_timeout_s))

    GQ, WQ = SetParkingGate.Request, SetParkingWhitelist.Request

    # ══════════════════════════════ A 月租车进场全程 ════════════════════════
    banner('A 白名单月租车进场：核验→抬杆→地感→释放→落杆→一条通行记录')
    check(fi.occupied and fo.occupied,
          '两道闸的核验位都在占用框内（抬到位时车仍压在杆下 = 保持全开/防砸的前提）',
          'in=%s out=%s' % (fi.occupied, fo.occupied))
    h.reset()
    cap = h.capture('gate_in')
    t_arr = h.t
    check(not h.cmds['gate_in'].new() and not h.passes.new(), '只收到抓拍绝不动杆',
          'cmd=%d pass=%d' % (len(h.cmds['gate_in'].new()), len(h.passes.new())))
    h.step(3)
    h.result(cap, MONTHLY)
    got = h.cmds['gate_in'].new()
    check(bool(got) and abs(got[-1].data - fi.g.open_rad) < 1e-6,
          '识别结果一到就下发抬起目标角', '%s' % [m.data for m in got])
    t_cmd = h.t
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok, '闸机实测抬到位（3 s 内）', h.note_of(ParkingAction.ACT_RAISE))
    if ok:
        check(0.7 <= h.t - t_cmd <= 1.2, '抬杆耗时贴着实测 0.85 s',
              '%.2f s' % (h.t - t_cmd))
    fi.moving = True                     # 抬到位才起步，和摆渡车一样
    ok = h.wait_for(lambda: bool(h.passes.new()), 40.0,
                    lambda: print('  末条动作：%s' % h.acts.last().note))
    check(ok, '一次进场在 40 s 仿真内收口', '已走 %.1f s' % (h.t - t_arr))
    p = h.passes.last()
    if p:
        check(p.result == ParkingPass.RESULT_ALLOWED and p.kind == ParkingPass.KIND_MONTHLY
              and p.whitelist_hit == 1, '月租放行', '%d/%d/%d' % (
                  p.result, p.kind, p.whitelist_hit))
        check(p.direction == ParkingPass.DIR_IN and p.gate_id == 'gate_in' and
              p.vehicle_id == CAR and p.plate_number == MONTHLY, '记录归属正确')
        check(p.capture_id == cap.capture_id and p.image_path == cap.image_path,
              '记录回指抓拍与原图', '%s %s' % (p.capture_id, p.image_path))
        check(abs(p.pose.position.y - fi.queue[1]) < 1e-6, '记录里是核验位位姿',
              'y=%.2f' % p.pose.position.y)
        check(abs(p.arrive_s - t_arr) < 1e-6, 'arrive_s = 抓拍时刻',
              '%.3f/%.3f' % (p.arrive_s, t_arr))
        check(0.7 <= p.raise_s - t_cmd <= 1.2, 'raise_s 落在实测抬杆耗时上',
              '%.2f s' % (p.raise_s - t_cmd))
        rel = [e for e in fi.edges if not e[1]]
        check(bool(rel) and abs(p.clear_s - rel[0][0]) <= DT + 1e-9,
              'clear_s 就是地感释放的下降沿（不是节点自己数出来的）',
              'clear=%.2f 边沿=%s' % (p.clear_s, rel[:1]))
        check(p.raise_s < p.clear_s < p.lower_s, '四个时刻严格单调',
              '%.2f/%.2f/%.2f/%.2f' % (p.arrive_s, p.raise_s, p.clear_s, p.lower_s))
        check(p.recognized and 0.0 < p.plate_confidence <= 1.0, '识别标志与置信度入档')
    c = collections.Counter(a.action for a in h.acts.new())
    check(c[ParkingAction.ACT_RAISE_CMD] == 1 and c[ParkingAction.ACT_RAISE] == 1 and
          c[ParkingAction.ACT_LOWER_CMD] == 1 and c[ParkingAction.ACT_LOWER] == 1,
          '一次抬杆配一次落杆（动作日志对得平）', '%s' % dict(c))
    check(c[ParkingAction.ACT_HOLD_OPEN] == 1, '保持全开每次占用只记一条',
          '%d 条' % c[ParkingAction.ACT_HOLD_OPEN])
    check(not c[ParkingAction.ACT_REFUSE] and not c[ParkingAction.ACT_FAULT],
          '正常流程不出拒绝/故障')
    check(len(h.passes.new()) == 1, '一次通行只落一条记录',
          '%d 条' % len(h.passes.new()))
    hold = h.act_of(ParkingAction.ACT_HOLD_OPEN)
    check(p is not None and bool(h.act_of(ParkingAction.ACT_RAISE)) and bool(hold) and
          (hold[0].header.stamp.sec + hold[0].header.stamp.nanosec * 1e-9 -
           p.raise_s) >= node.gates['gate_in'].hold_s - 1e-6,
          '保持全开那条确实在 hold_time_s 之后', '')
    check(abs(fi.angle - fi.g.closed_rad) < 1e-6 and not fi.occupied,
          '收尾状态：杆落回关闭位、车道空闲')
    check(fi.raise_count == 1 and fi.lower_count == 1, '插件计数与动作日志同数',
          '%d/%d' % (fi.raise_count, fi.lower_count))

    # ══════════════════════════════ B 识别为空 ═════════════════════════════
    banner('B 识别结果为空：挂起 + 拒绝动作，杆不动')
    h.reset()
    cap = h.capture('gate_in')
    h.result(cap, '')
    p = h.passes.last()
    check(bool(p) and p.result == ParkingPass.RESULT_PENDING and not p.recognized,
          '空结果按挂起记录', '%s' % getattr(p, 'result', None))
    check(p is not None and '等待人工核验' in p.reason, '原因写明等人工',
          p.reason if p else '')
    check(not h.cmds['gate_in'].new() and not h.act_of(ParkingAction.ACT_RAISE_CMD),
          '没牌不抬杆')
    check(bool(h.act_of(ParkingAction.ACT_REFUSE)), '拒绝进动作日志')

    # ══════════════════════════════ C 核验超时 ═════════════════════════════
    banner('C 识别节点不回包：verify_timeout_s 到点自己收口')
    h.reset()
    cap = h.capture('gate_in')
    vt = node.gates['gate_in'].verify_timeout_s
    ok = h.wait_for(lambda: bool(h.passes.new()), vt + 5.0)
    p = h.passes.last()
    check(ok and p is not None, '超时后有一条记录落地', '走了 %.1f s' % (h.t - cap
          .header.stamp.sec))
    check(p is not None and abs(p.arrive_s + vt - h.t) < 1.0,
          '收口时刻就是核验超时阈值（不是无限等）', 'h.t=%.2f' % h.t)
    check(p is not None and p.result == ParkingPass.RESULT_PENDING and
          ('%.1f s' % vt) in p.reason, '原因点名超时阈值', p.reason if p else '')
    check(not h.cmds['gate_in'].new(), '超时同样不动杆')

    # ══════════════════════════════ D 黑名单优先级 ═════════════════════════
    banner('D 黑名单优先于白名单（同一个号在两份名单里）')
    h.reset()
    node.blacklist = {BOTH}
    check(BOTH in node.whitelist, '前提：这个号确实在白名单里')
    cap = h.capture('gate_in')
    h.result(cap, BOTH)
    p = h.passes.last()
    check(p is not None and p.result == ParkingPass.RESULT_DENIED and
          p.kind == ParkingPass.KIND_BLACKLIST and p.whitelist_hit == 3,
          '命中黑名单直接拒绝', '%s' % (getattr(p, 'kind', None)))
    check(not h.cmds['gate_in'].new(), '黑名单不动杆')

    # ══════════════════════ E 临时车：发卡→出场→核销→再出场被拒 ════════════
    banner('E 未登记车牌：入口自动发卡放行 → 出场凭在途卡放行 → 落杆核销 → 再出场被拒')
    h.reset()
    cap = h.capture('gate_in')
    h.result(cap, TEMP)
    got = h.cmds['gate_in'].new()
    check(bool(got) and abs(got[-1].data - fi.g.open_rad) < 1e-6,
          '未登记车牌按临时车放行（unknown_as_temp=true）',
          '%s' % [m.data for m in got])
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok, '入口闸实测抬到位')
    fi.moving = True                     # 抬到位才起步，和 A 同一条时序
    ok = h.wait_for(lambda: bool(h.passes.new()), 40.0)
    p_in = h.passes.last()
    check(ok and p_in is not None and p_in.result == ParkingPass.RESULT_ALLOWED and
          p_in.kind == ParkingPass.KIND_TEMP, '进场记录为临时车放行',
          '%s' % getattr(p_in, 'kind', None))
    tid = p_in.ticket_id if p_in else ''
    check(tid.startswith('T-') and tid in node.tickets, '临时卡已发出在册', tid)

    # 出场：不清名单，模拟"同一辆车还在场内"
    h.reset(clear_lists=False)
    cap = h.capture('gate_out')
    h.result(cap, TEMP)
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok, '出口闸实测抬到位')
    fo.moving = True                     # 出口核验位本身在框内，不动就永远不释放
    ok = h.wait_for(lambda: bool(h.passes.new()), 40.0)
    p_out = h.passes.last()
    check(ok and p_out is not None and p_out.result == ParkingPass.RESULT_ALLOWED and
          p_out.kind == ParkingPass.KIND_TEMP and p_out.whitelist_hit == 2,
          '出场凭在途临时卡放行（不是靠名单）', '%s' % (
              getattr(p_out, 'whitelist_hit', None)))
    check(p_out is not None and p_out.direction == ParkingPass.DIR_OUT and
          p_out.ticket_id == tid, '出场记录同卡号',
          getattr(p_out, 'ticket_id', None))
    check(tid not in node.tickets, '落杆即核销，卡不再在册', 'tickets=%s' % (
        list(node.tickets)))
    h.step(2)
    cap = h.capture('gate_out')
    h.result(cap, TEMP)
    p3 = h.passes.last()
    check(p3 is not None and p3.result == ParkingPass.RESULT_DENIED and
          '无入场记录' in p3.reason, '核销后同一张号再出场就是拒绝',
          p3.reason if p3 else '')

    # ══════════════════════ F unknown_as_temp=false ════════════════════════
    banner('F 关掉临时车分支：未登记车牌直接拒入')
    h.reset()
    node.unknown_as_temp = False
    cap = h.capture('gate_in')
    h.result(cap, '黑A00001')
    p = h.passes.last()
    check(p is not None and p.result == ParkingPass.RESULT_DENIED and
          p.kind == ParkingPass.KIND_UNKNOWN and 'unknown_as_temp' in p.reason,
          '拒绝并写明开关状态', p.reason if p else '')
    check(not h.cmds['gate_in'].new(), '不动杆')

    # ══════════════════════ G 地感失效：降级计时放行 ════════════════════════
    banner('G SENSOR_FAIL：降级为计时放行，冻结的占用读数不冒充"检测到离场"')
    h.reset()
    g = node.gates['gate_in']
    cap = h.capture('gate_in')
    h.result(cap, MONTHLY)
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok, '先正常抬杆')
    fi.moving = True
    ok = h.wait_for(lambda: fi.occupied, 12.0)
    check(ok, '车确实压上地感了才注入故障', '边沿=%s' % fi.edges)
    r = call_gate(node, op=GQ.OP_FAULT, gate_id='gate_in',
                  fault_kind=BarrierState.FAULT_SENSOR_FAIL)
    check(r.success, 'OP_FAULT 走故障通道下发', r.message)
    # 注入走的是"话题→假闸机→BarrierState→节点看边沿"这条闭环，
    # 下发那一拍节点还看不见故障，必须等一步再把降级断言当真值读。
    ok = h.wait_for(lambda: g.degraded, 1.0)
    check(ok, 'BarrierState 里的 SENSOR_FAIL 被节点看成降级边沿')
    check(bool(h.act_of(ParkingAction.ACT_DEGRADE)), '降级动作入日志',
          h.note_of(ParkingAction.ACT_DEGRADE))
    check(g.degraded and not g.fault_latched, '地感失效只降级，不挂故障闸')
    fl = h.faults.new()
    check(bool(fl) and fl[-1].data == 'gate_in:SENSOR_FAIL',
          '故障通道收到的是 gate_in:SENSOR_FAIL',
          getattr(fl[-1], 'data', '') if fl else '空')
    # 抬到位时刻从 ACT_RAISE 那条的 stamp 取（节点说的时刻必须是插件说的时刻）
    a = h.act_of(ParkingAction.ACT_RAISE)[0]
    t_raise = a.header.stamp.sec + a.header.stamp.nanosec * 1e-9
    lo = None
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_LOWER_CMD)),
                    g.degrade_wait_s + 2.0)
    check(ok, '计时放行下发了落杆', h.note_of(ParkingAction.ACT_LOWER_CMD))
    if ok:
        b = h.act_of(ParkingAction.ACT_LOWER_CMD)[0]
        t_lo = b.header.stamp.sec + b.header.stamp.nanosec * 1e-9
        check(g.degrade_wait_s <= t_lo - t_raise <= g.degrade_wait_s + 0.35,
              '落杆时刻 = 抬到位 + degrade_wait_s', '%.2f s' % (t_lo - t_raise))
        check('计时放行' in b.note, '日志点名是计时放行', b.note)
        check(fi.occupied and not fi.real_occ,
              '冻结的占用读数仍是"有车"，真实地感其实已释放',
              'occupied=%s real_occ=%s' % (fi.occupied, fi.real_occ))
    ok = h.wait_for(lambda: bool(h.passes.new()), 5.0)
    p = h.passes.last()
    check(ok and p is not None and p.result == ParkingPass.RESULT_ALLOWED,
          '降级流程照样出放行记录')
    check(p is not None and abs(p.clear_s + 1.0) < 1e-9,
          'clear_s 保持 -1：地感不可信时不谎称检测到离场', '%.2f' % (
              p.clear_s if p else -99))
    check(p is not None and p.lower_s > 0.0, '落杆到位时刻仍然记录')
    call_gate(node, op=GQ.OP_CLEAR_FAULT, gate_id='gate_in')
    h.step(20)
    check(not g.degraded and bool(h.act_of(ParkingAction.ACT_FAULT_CLEAR)),
          '清除后降级状态解除并入日志')

    # ══════════════════════ H 抬杆中途卡滞 ═════════════════════════════════
    banner('H STUCK：抬杆中途卡滞 → 按挂起记账；清除那一拍先防砸挡回，车离杆后自动落回')
    h.reset()
    cap = h.capture('gate_in')
    h.result(cap, MONTHLY)
    h.step(3)                     # 杆在半途（约 0.5 rad）
    call_gate(node, op=GQ.OP_FAULT, gate_id='gate_in',
              fault_kind=BarrierState.FAULT_STUCK)
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_FAULT)), 1.0)
    check(ok, '故障入日志', h.note_of(ParkingAction.ACT_FAULT))
    p = h.passes.last()
    check(p is not None and p.result == ParkingPass.RESULT_PENDING and
          abs(p.raise_s + 1.0) < 1e-9 and 'STUCK' in p.reason,
          '抬不到位就按挂起记账、raise_s=-1', '%s / %.2f' % (
              getattr(p, 'reason', ''), getattr(p, 'raise_s', -99)))
    ang = fi.angle
    h.cmds['gate_in'].mark()          # 只看这一刻以后的新下发，队列留给假闸机
    h.step(30)
    check(not h.cmds['gate_in'].new() and abs(fi.angle - ang) < 1e-6,
          '故障挂起期间节点不再指挥、杆也不再动', '%s' % [
              m.data for m in h.cmds['gate_in'].new()])
    fl = h.faults.new()
    check(bool(fl) and fl[-1].data == 'gate_in:STUCK',
          '故障通道收到的仍是 gate_in:STUCK',
          getattr(fl[-1], 'data', '') if fl else '空')
    call_gate(node, op=GQ.OP_CLEAR_FAULT, gate_id='gate_in')
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_REFUSE)), 1.0)
    check(ok and '车离杆后自动补落杆' in h.note_of(ParkingAction.ACT_REFUSE),
          '清除那一拍杆下压着车：先记防砸挡回，并欠下一发收尾落杆',
          h.note_of(ParkingAction.ACT_REFUSE))
    check(node.gates['gate_in'].pending_clear_at is not None, '待收尾落杆已登记')
    check(not node.gates['gate_in'].fault_latched, '故障闸已松开')
    fi.moving = True
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_LOWER_CMD)), 40.0)
    check(ok and '收尾落杆' in h.note_of(ParkingAction.ACT_LOWER_CMD),
          '车离杆后节点把欠的收尾落杆补上（车道不会永久敞开）',
          h.note_of(ParkingAction.ACT_LOWER_CMD))
    check(node.gates['gate_in'].pending_clear_at is None, '补完就不再重复下发')
    ok = h.wait_for(lambda: abs(fi.angle - fi.g.closed_rad) < fi.tol, 5.0)
    check(ok, '杆回到关闭位', '角=%.3f' % fi.angle)
    check(bool(h.act_of(ParkingAction.ACT_LOWER)) and
          '无在途通行' in h.note_of(ParkingAction.ACT_LOWER),
          '无在途通行的落杆补了一条日志（抬落计数仍对得平）',
          h.note_of(ParkingAction.ACT_LOWER))
    check(not node.gates['gate_in'].cycle, '在途通行已清空')

    # ══════════════════════ I 防砸联锁与人工占权 ════════════════════════════
    banner('I 出口闸等待即占用：手动落杆必须被防砸联锁挡回；人工占权时自动让位')
    h.reset()
    check(fo.occupied, '前提：出口核验位就在占用框内')
    node.on_manual(String(data='gate_out:%.4f' % fo.g.closed_rad))
    check(bool(h.act_of(ParkingAction.ACT_REFUSE)) and
          '防砸联锁' in h.note_of(ParkingAction.ACT_REFUSE),
          '话题手动落杆被挡回并记 ACT_REFUSE', h.note_of(ParkingAction.ACT_REFUSE))
    check(not h.cmds['gate_out'].new(), '被挡回时一条目标角都没下发',
          '%s' % [m.data for m in h.cmds['gate_out'].new()])
    r = call_gate(node, op=GQ.OP_LOWER, gate_id='gate_out')
    check(not r.success and '挡回' in r.message, 'srv 落杆同样回失败（不报假成功）',
          r.message)
    r = call_gate(node, op=GQ.OP_RAISE, gate_id='gate_out', angle_rad=-999.0)
    check(r.success and '1.57' in r.message, 'srv 抬杆用 YAML 抬起角', r.message)
    got = h.cmds['gate_out'].new()
    check(bool(got) and abs(got[-1].data - fo.g.open_rad) < 1e-6, '抬起角已下发')
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok and h.act_of(ParkingAction.ACT_RAISE)[0].source ==
          BarrierState.SRC_MANUAL, '到位日志记为人工来源', '%s' % (
              h.act_of(ParkingAction.ACT_RAISE)[0].source if ok else None))
    fwd = h.state.last()
    check(fwd is not None and fwd.source == BarrierState.SRC_MANUAL,
          '转发出去的状态带人工标记', '%s' % getattr(fwd, 'source', None))
    h.cmds['gate_out'].mark()      # 上面人工抬杆那次下发不算"抢杆"，从这里重新起算
    cap = h.capture('gate_out')
    h.result(cap, MONTHLY)
    p = h.passes.last()
    check(p is not None and p.result == ParkingPass.RESULT_PENDING and
          '人工占权' in p.reason, '人工占权时自动流程让位（不抢杆）',
          p.reason if p else '')
    check(not h.cmds['gate_out'].new(), '让位期间没有新的下发')
    check(p is not None and p.kind == ParkingPass.KIND_MONTHLY and
          p.whitelist_hit == 1, '名单结论照样入档（拒绝的是抬杆不是核验）')
    node.on_manual(String(data='gate_out:AUTO'))
    check(node.gates['gate_out'].mode == 'auto' and
          bool(h.act_of(ParkingAction.ACT_MANUAL)), '交还自动入日志')

    # ═════════════ I2 人工占权撞上在途通行：这一拍必须把通行闭合掉 ═════════════
    # 2026-10-14 10:23:47 的现场：条目3 演手动落杆时 gate_in 正有一笔在途通行，CLOSED
    # 到位那一拍只走了 on_state 的手动分支，日志记成"手动目标角到位"、g.cycle 没归零。
    # 后果是这道闸之后每张抓拍都被"在途通行未结束"忽略，摆渡车连废 3 轮后停驶挂起。
    banner('I2 在途通行 + 人工落杆到位：通行要闭合、这道闸不能被锁死')
    h.reset()
    fi = h.fakes['gate_in']
    cap = h.capture('gate_in')
    h.result(cap, MONTHLY)
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_RAISE)), 3.0)
    check(ok and fi.occupied, '前提：自动核验已抬杆到位、车还压在入口地感上')
    node.on_manual(String(data='gate_in:%.4f' % fi.g.open_rad))
    check(node.gates['gate_in'].mode == 'manual' and node.gates['gate_in'].cycle,
          '人工占权接管，此时在途通行仍未结束')
    fi.moving = True
    ok = h.wait_for(lambda: not fi.occupied, 40.0)
    check(ok, '车开出占用框，地感释放（手动期间自动流程不抢杆）')
    node.on_manual(String(data='gate_in:%.4f' % fi.g.closed_rad))
    ok = h.wait_for(lambda: bool(h.act_of(ParkingAction.ACT_LOWER)), 6.0)
    check(ok, '人工落杆到位了', h.note_of(ParkingAction.ACT_LOWER))
    low = h.act_of(ParkingAction.ACT_LOWER)
    check(bool(low) and low[0].source == BarrierState.SRC_MANUAL,
          '这一下归人工（日志看得出是谁下的令）',
          '%s' % (low[0].source if low else None))
    check(bool(low) and '通行结束' in low[0].note,
          '记的是"通行结束"而不是"手动目标角到位"——通行真的闭合了',
          low[0].note if low else '')
    p = h.passes.new()
    check(bool(p) and p[-1].result == ParkingPass.RESULT_ALLOWED and p[-1].lower_s > 0.0,
          '在途通行随这一拍入档（不必等自动落杆）',
          p[-1].reason if p else '一条通行都没出')
    check(not node.gates['gate_in'].cycle, '在途通行已清空')
    h.capture('gate_in')
    check(node.gates['gate_in'].cycle is not None,
          '下一张抓拍被受理：这道闸没有被一次手动落杆永久锁死', 'cycle 仍是 None')

    # ══════════════════════ J 两个 srv 的 op 覆盖 ═══════════════════════════
    banner('J srv 全 op（异常只回 success=False，不能打死节点）')
    r = call_gate(node, op=GQ.OP_LIST)
    check(r.success and len(r.lines) == 2 and len(r.states) == 2,
          'OP_LIST 回两道闸的结构化状态', r.message)
    check(r.states and all(s.gate_id in ('gate_in', 'gate_out') for s in r.states),
          '回包里的闸机 id 与话题命名空间一致')
    r = call_gate(node, op=GQ.OP_RAISE, gate_id='gate_不存在')
    check(not r.success and '没有这道闸机' in r.message, '未知闸机回失败', r.message)
    r = call_gate(node, op=GQ.OP_FAULT, gate_id='gate_in', fault_kind=99)
    check(not r.success and 'fault_kind' in r.message, '无效故障类型回失败', r.message)
    r = call_gate(node, op=88)
    check(not r.success, '未知 op 回失败', r.message)
    r = call_gate(node, op=GQ.OP_AUTO)
    check(r.success and all(g.mode == 'auto' for g in node.gates.values()),
          '空 gate_id = 两道都交还自动', r.message)

    r = call_wl(node, op=WQ.OP_LIST)
    n0 = len(node.whitelist)
    check(r.success and len(r.whitelist) == n0 and
          len(r.whitelist_kinds) == n0, 'OP_LIST 回完整名单', r.message)
    r = call_wl(node, op=WQ.OP_ADD, plate_numbers=['浙A1', '浙A2'],
                kinds=[ParkingPass.KIND_MONTHLY])
    check(not r.success and '不一致' in r.message, '长度不一致回失败', r.message)
    r = call_wl(node, op=WQ.OP_ADD, plate_numbers=['浙A1', '浙A2'],
                kinds=[ParkingPass.KIND_MONTHLY, ParkingPass.KIND_TEMP],
                labels=['月租'])
    check(r.success and node.whitelist['浙A2']['kind'] == ParkingPass.KIND_TEMP and
          len(node.whitelist) == n0 + 2, 'OP_ADD 新增/覆盖生效', r.message)
    r = call_wl(node, op=WQ.OP_ISSUE_TEMP, plate_numbers=['闽D7'], labels=['来访施工'])
    tid = r.tickets[-1] if r.tickets else ''
    check(r.success and node.whitelist['闽D7']['label'] == tid and tid.startswith('T-'),
          'OP_ISSUE_TEMP 发卡并入名单', r.message)
    r = call_wl(node, op=WQ.OP_ISSUE_TEMP)
    check(not r.success, '发卡缺车牌回失败', r.message)
    r = call_wl(node, op=WQ.OP_REVOKE_TEMP, plate_numbers=['闽D7'])
    check(r.success and '闽D7' not in node.whitelist and not node.tickets,
          'OP_REVOKE_TEMP 收回', r.message)
    r = call_wl(node, op=WQ.OP_REMOVE, plate_numbers=['浙A1'])
    r2 = call_wl(node, op=WQ.OP_REMOVE, plate_numbers=['根本不存在的号'])
    check(r.success and '浙A1' not in node.whitelist and '删除 0 条' in r2.message,
          'OP_REMOVE 命中为真、落空写明 0 条', '%s / %s' % (r.message, r2.message))
    r = call_wl(node, op=WQ.OP_RESET)
    check(r.success and list(node.whitelist) == list(node.whitelist_base) and
          not node.tickets, 'OP_RESET 回到 YAML 基线', '%s' % list(node.whitelist))
    r = call_wl(node, op=77)
    check(not r.success, '未知 op 回失败', r.message)
    check(rclpy.ok() and len(h.passes.hist) > 0, '节点活到全部热改之后')
    ids = [p.pass_id for p in h.passes.hist]
    check(len(ids) == len(set(ids)), 'pass_id 全程不重号', '%d/%d' % (
        len(ids), len(set(ids))))
    a_ids = [a.action_id for a in h.acts.hist]
    check(len(a_ids) == len(set(a_ids)), 'action_id 全程不重号')

    node.destroy_node()
    rclpy.shutdown()
    if FAILED:
        print('\nSELFTEST FAIL：%d 项\n  - %s' % (len(FAILED), '\n  - '.join(FAILED)))
        return 1
    print('\nSELFTEST PASS：进场/挂起/超时/黑名单/临时卡核销/降级计时/卡滞恢复/'
          '防砸联锁/人工占权/热改名单全部符合预期')
    return 0


if __name__ == '__main__':
    try:
        rc = main()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)
