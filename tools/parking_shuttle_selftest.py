#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""parking_shuttle_node 的"本段中途落杆"分支离线自测（不需要 Gazebo）。

为什么要单独测这一段：#77 那轮的现场是"车顶着放平的杆原地空转 20 s、净位移 0.9 mm"，
而当时的代码里没有"起步之后杆还会再落下"这个概念 —— hold 阶段只等"下一段"的杆，
起步后就把杆的事忘了。补上 pending_boom 之后，判据分成两个方向（车在杆轴哪一侧）
加一个恢复条件（杆重新抬起），这些在真机上要靠掐 hold_time_s 的时序才能复现，
所以这里把节点实例化、用合成的 /model_states + /clock + BarrierState 直接喂 tick()：

  1) 杆全程抬起 → 正常下发前进指令，且不退回（这条同时是 2) 的负样本：证明"退回"
     真的挂在实测角上，不是"车在杆轴南侧就无脑倒车"）；
  2) 杆在车还没越过杆轴时落下 → 倒车退回上一航点，作废计数 +1；
  3) 杆在车已经越过杆轴后落下 → 只能停等（倒回去等于从放平的杆下穿过去），零速；
  4) 停等超过 GATE_WAIT_S → 打一条点名"停等人工抬杆"的 warn，别让日志读成死机；
  5) 停等期间杆重新抬起 → 挂起解除，恢复行车。
  7) 配对转向点（entry#6→exit#0 同坐标差 π）→ 推进后回到 approach，但到位按**轮轴**判：
     #80 那版"直接跳 align"的捷径已随 #89 的轮轴到位判定一起撤销（跳步会把底盘甩在航点
     外 0.57 m）。这条测的是"被转向甩开的那一拍必须停下原地转、不许带着饱和角速度行车"，
     并配"轮轴到位那一拍底盘正好压在航点上 → align → hold"与坐标不同的负样本。
  8) 指令整形（执行层）：非零指令一拍只能涨 RATE_*·dt、零速当拍生效、低于 MIN_* 的
     请求顶到死区下限。155 轮的"阶跃→打滑→31.76 rad/s 弹射→之后对指令全无声响"
     就是缺这一层，而它在真机上要攒很久才复现，这里只看发出去的那一帧。
  9) 原地转预算：合成一个"每拍都真在转、却永远转不到位"的流（watch_spin 看不出来），
     必须在 2π rad 的预算内挂起 —— 用户要的"别再原地画圈"要有硬上限，不能只靠增益调好。
 10) 闭环积分（"别再原地画圈"的正样）：拿节点自己发出的整形后指令按差速运动学推车
     （轮轴沿车头走 v·dt、车头转 ω·dt，底盘由轮轴反推），起点取 155 轮实测被预算挂起
     的那一帧。开环断言看不出"两支来回切着绕圈"，只有闭环能证明行车律自己收敛。
 11) 提前归零门（BRAKE_LAG）：样本直接取 /tmp/pk190_trace.txt 里 exit#3 那一拍
     （误差 +0.09 rad、实测 ω +0.25 rad/s、指令录到 −0.04），先复刻旧律算出同一数，
     再断言门把它归零；顺带三个"门不该管闲事"的反面样本（反转中、2.32 rad 大角度、
     静止起步）。合成积分器是最小相位的，跑不出场内那个死循环，所以旧律挂起这件事
     不写成断言，闭环那半只当"加门之后不回退"的门禁。

跑法（WSL，需要 source 过 ~/v2x_ws/install 以便 import v2x_msgs）：
    python3 tools/parking_shuttle_selftest.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, 'src', 'v2x_demo'))

import rclpy                                        # noqa: E402
from gazebo_msgs.msg import ModelStates             # noqa: E402
from geometry_msgs.msg import Pose, Twist           # noqa: E402
from rclpy.parameter import Parameter               # noqa: E402
from rosgraph_msgs.msg import Clock                 # noqa: E402

from v2x_msgs.msg import BarrierState               # noqa: E402
from v2x_demo import parking_shuttle_node as shut   # noqa: E402

CFG = os.path.abspath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    os.pardir, 'src', 'v2x_demo', 'config', 'parking_zones.yaml'))

BOOM_Y = -11.05          # 两道闸的杆轴，与 yaml/体检同源
BOOM = 'gate_in'
IDX_AFTER_QUEUE = 2      # entry_route[2] = (12.00, -9.55)：核验位之后那个"已过杆"的航点

FAILS = []


def ok(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        FAILS.append(msg)


class Sink:
    """替掉两个 publisher：断言只看"这一拍下发了什么指令、自报了句什么"。"""

    def __init__(self):
        self.cmds, self.notes = [], []

    def publish(self, msg):
        if isinstance(msg, Twist):
            self.cmds.append(msg)
        else:
            self.notes.append(msg.data)

    @property
    def cmd(self):
        return self.cmds[-1]

    @property
    def note(self):
        return self.notes[-1]


class LogSpy:
    def __init__(self, real):
        self.real, self.warns, self.infos = real, [], []

    def warn(self, m):
        self.warns.append(str(m))

    def info(self, m):
        self.infos.append(str(m))

    def error(self, m):
        self.real.error(str(m))


def feed(node, sink, y, sim_now, boom_state=None, boom_angle=0.0):
    """喂一帧"车在 (12.00, y)、车头正北"的位姿 + 仿真钟，可选地把闸机状态打到指定档。"""
    feed_pose(node, sink, 12.00, y, 1.5708, sim_now, boom_state, boom_angle)


def feed_pose(node, sink, x, y, yaw, sim_now, boom_state=None, boom_angle=0.0):
    """闸机状态必须每帧重喂：boom_up 用墙钟判新鲜度（3 s 没更新就不算数），一次喂完连跑
    五拍会把后面几拍全判成"状态过期"，测出来的就不是几何而是新鲜度了。"""
    ms = ModelStates()
    ms.name = [node.model]
    p = Pose()
    p.position.x, p.position.y = x, y
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    ms.pose = [p]
    node.on_models(ms)
    c = Clock()
    c.clock.sec = int(sim_now)
    c.clock.nanosec = int(round((sim_now - int(sim_now)) * 1e9))
    node.on_clock(c)
    if boom_state is not None:
        g = BarrierState()
        g.gate_id = BOOM
        g.state = boom_state
        g.angle_rad = boom_angle
        g.target_rad = 1.5708 if boom_state == BarrierState.STATE_OPEN else 0.0
        g.at_target = boom_state in (BarrierState.STATE_OPEN,
                                     BarrierState.STATE_CLOSED)
        node.on_gate(BOOM, g)
    else:
        node.gates.pop(BOOM, None)
    node.pub_cmd, node.pub_status = sink, sink


def run_case(node, sink, y, sim_now, boom_state, boom_angle=0.0):
    feed(node, sink, y, sim_now, boom_state, boom_angle)
    node.tick()


def fresh(node, idx=IDX_AFTER_QUEUE, pending=BOOM):
    """把节点摆到"正在从核验位开往通道带、本段要越 gate_in"这个挂起态。"""
    node.idx = idx
    node.stage = 'approach'
    node.pending_boom = pending
    node.hold_gate = None
    node.boom_stall = None
    node.boom_stall_logged = False
    node.rest_until = 0.0
    node.ref = None
    node.back_to = None


def main():
    if not os.path.exists(CFG):
        print('配置文件不存在：%s' % CFG)
        return 2
    rclpy.init()
    node = shut.ShuttleNode(parameter_overrides=[
        Parameter('config_file', Parameter.Type.STRING, CFG)])
    sink = Sink()
    spy = LogSpy(node.get_logger())
    node.get_logger = lambda: spy
    node.pub_cmd = node.pub_status = sink
    # 本自测全程手工摆放车位与航点序号（每个 case 都是在测"跑途中某一态"），
    # 所以关掉新进程那一次 seat_at 自动选路 —— 留着它，case 摆好的 idx 会被改掉。
    node.seated = True

    # ── 0) 前置：这条航段确实被判成"要越杆"，且杆轴几何与 yaml 一致 ──────────
    ok(node.crosses_boom((12.00, -12.21), (12.00, -9.55)) == BOOM,
       '核验位→通道带这一段的 hold_gate 应为 %s（实测 %r）' % (
           BOOM, node.crosses_boom((12.00, -12.21), (12.00, -9.55))))
    booms = {b[0]: b[1] for b in node.booms}
    ok(abs(booms.get(BOOM, 99.0) - BOOM_Y) < 1e-9,
       'gate_in 杆轴 y 应为 %.2f（实测 %s）' % (BOOM_Y, booms.get(BOOM)))

    # ── 1) 杆抬起：正常行车，且这条是 2) 的负样本 ──────────────────────────
    fresh(node)
    ab0 = node.aborted
    run_case(node, sink, -11.60, 100.0, BarrierState.STATE_OPEN, 1.5708)
    ok(node.pending_boom == BOOM and node.boom_stall is None,
       '杆抬起后停等计时应解除（boom_stall=%r）；pending_boom 现在是每拍按'
       '实际线段算出的活值，抬起只代表能过、不再清零它（实测 %r）' % (
           node.boom_stall, node.pending_boom))
    ok(sink.cmd.linear.x > 0.0, '杆抬起时应当前进，实测 v=%.2f' % sink.cmd.linear.x)
    ok(node.stage == 'approach' and node.aborted == ab0,
       '杆抬起时不该退回、不该计作废（stage=%s 作废=%d）' % (node.stage, node.aborted))

    # ── 2) 未过杆轴就落杆 → 倒车退回 ──────────────────────────────────────
    fresh(node)
    ab0 = node.aborted
    run_case(node, sink, -11.60, 100.0, BarrierState.STATE_CLOSED)
    ok(node.stage == 'back',
       '未越过杆轴而杆落下应转入倒车，实测 stage=%s' % node.stage)
    ok(node.aborted == ab0 + 1, '作废计数应 +1（实测 %d）' % node.aborted)
    ok(node.back_to == (12.00, -12.21),
       '退回目标应是核验位，实测 %s' % (node.back_to,))
    ok(abs(sink.cmd.linear.x) < 1e-9,
       '决定退回那一拍应先零速，实测 v=%.2f' % sink.cmd.linear.x)

    # ── 3) 已过杆轴才落杆 → 只能停等 ──────────────────────────────────────
    fresh(node)
    ab0 = node.aborted
    run_case(node, sink, -10.60, 100.0, BarrierState.STATE_CLOSED)
    ok(node.stage == 'approach',
       '已过杆轴时不能倒车（等于从杆下穿回去），应留在本段，实测 stage=%s' % node.stage)
    ok(node.aborted == ab0 and node.back_to is None,
       '停等分支不该计作废、不该设退回点（作废=%d back=%s）' % (
           node.aborted, node.back_to))
    ok(abs(sink.cmd.linear.x) < 1e-9 and abs(sink.cmd.angular.z) < 1e-9,
       '停等那一拍必须双零，实测 v=%.2f w=%.2f' % (
           sink.cmd.linear.x, sink.cmd.angular.z))
    ok(node.boom_stall is not None, '停等应记下起点用于超时告警（实测 %r）' % (
        node.boom_stall,))

    # ── 4) 停等超过 GATE_WAIT_S → 点名"在等人工抬杆" ──────────────────────
    w0 = len(spy.warns)
    node.boom_stall = 100.0 - shut.GATE_WAIT_S - 1.0
    run_case(node, sink, -10.60, 100.0, BarrierState.STATE_CLOSED)
    ok(len(spy.warns) == w0 + 1,
       '超过 %.0f s 应恰好补一条 warn（实测 +%d 条）' % (
           shut.GATE_WAIT_S, len(spy.warns) - w0))
    ok(spy.warns and '人工抬杆' in spy.warns[-1],
       'warn 要写明在等人工抬杆，实测：%r' % (spy.warns[-1] if spy.warns else None,))
    run_case(node, sink, -10.60, 101.0, BarrierState.STATE_CLOSED)
    ok(len(spy.warns) == w0 + 1, '同一次停等只播一条 warn（实测 %d 条）' % (
        len(spy.warns) - w0))

    # ── 5) 杆重新抬起 → 解除挂起、恢复行车 ────────────────────────────────
    run_case(node, sink, -10.60, 102.0, BarrierState.STATE_OPEN, 1.5708)
    ok(node.boom_stall is None and abs(sink.cmd.linear.x) > 0.0,
       '抬起后停等计时要清空且真的给油（stall=%r v=%.2f）；'
       'pending_boom 是活值，车尾还在杆口余量内时它依旧点名 %r，这正是我们要的' % (
           node.boom_stall, sink.cmd.linear.x, BOOM))
    ok(sink.cmd.linear.x > 0.0,
       '抬起后应恢复前进，实测 v=%.2f' % sink.cmd.linear.x)

    # ── 6) 状态读不到（插件没加载）→ 停在原地而不是往前顶 ─────────────────
    fresh(node)
    run_case(node, sink, -11.60, 100.0, None)
    ok(node.stage == 'back' and abs(sink.cmd.linear.x) < 1e-9,
       '读不到闸机状态时按"未过杆"退回处理，实测 stage=%s v=%.2f' % (
           node.stage, sink.cmd.linear.x))

    # ── 7) 配对转向点（同 (x,y) 换朝向）：推进回 approach，到位按**轮轴**判 ────
    # 现场 15:44~15:56 卡在 exit#0 原地绕圈 12 min：车位 entry#6 与出场起点 exit#0
    # 是同坐标、朝向差 π 的配对点，而原地转绕的是**差速轴**（实测车心被甩 0.42 m 画圆），
    # arrive_radius 只有 0.12 m → "停下原地转 / 转一下车心甩开"互相结合就是死循环。
    # 当时（#80）的修法是"配对点直接跳 align、把 approach 整段跳过"；#89 把到位判定改
    # 成按轮轴中点之后，这条捷径必须**撤销**：轮轴目标 = 航点 + axle·û(该点朝向) 与当前
    # 朝向无关，转向甩不动它，老实回 approach 才收敛。留着跳步的实测后果是 reset#1 转完
    # 90° 底盘落在 (12.40,-13.20) 而航点是 (12.00,-12.80)，横向偏 0.57 m —— 抓拍余量没有。
    p6, p7 = node.seq[6], node.seq[7]
    ok(p6[2] == p7[2] and p6[3] == p7[3] and abs(shut.wrap(p6[4] - p7[4])) > 3.0,
       '前置：entry#6 与 exit#0 应同坐标、朝向差 π（实测 %s / %s）' % (p6, p7))
    node.idx = 6
    node.stage = 'hold'
    node.hold_gate = None
    node.hold_started = 190.0
    node.hold_until = 200.0
    node.pending_boom = None
    node.boom_stall = None
    node.rest_until = 0.0
    node.ref = None
    # 车被 entry#6 的 align 甩开 0.42 m、车头还偏 0.22 rad —— 正是现场那一帧的数值。
    # 轮轴此刻在 (8.63,-7.86)，而 exit#0 要轮轴落在 (8.40,-8.35)，偏 0.54 m；方位差
    # 2.49 rad 早已越过饱和点 turn/1.5=0.42 → 这一拍必须是"停下原地转"：既不许带着
    # 饱和角速度行车（绕圈的成因），也不许直接跳进 align（旧捷径）。
    feed_pose(node, sink, 8.70, -8.25, 1.79, 200.5, None)
    node.tick()
    ok(node.stage == 'approach' and node.idx == 7,
       '配对点推进后应回到 approach，实测 stage=%s idx=%d' % (node.stage, node.idx))
    # next_point 只改状态、当拍发的是"进入下一航点"的零速；行驶指令在下一拍。
    feed_pose(node, sink, 8.70, -8.25, 1.79, 200.65, None)
    node.tick()
    ok(abs(sink.cmd.linear.x) < 1e-9 and abs(sink.cmd.angular.z) > 0.0,
       '方向差越过饱和点 → 停下原地转，实测 v=%.2f w=%.2f' % (
           sink.cmd.linear.x, sink.cmd.angular.z))
    # 轮轴到位那一拍：底盘正好压在航点上（(8.40,-7.95)+朝向南的 0.40 m 轮轴前探）
    feed_pose(node, sink, 8.40, -7.95, -1.5708, 200.8, None)
    node.tick()
    ok(node.stage == 'align' and node.idx == 7,
       '轮轴到位（底盘恰落在航点上）应转入 align，实测 stage=%s idx=%d' % (
           node.stage, node.idx))
    feed_pose(node, sink, 8.40, -7.95, -1.5708, 201.0, None)
    node.tick()
    ok(node.stage == 'hold', '朝向也转正后应转入 hold，实测 stage=%s' % node.stage)
    ok(node.hold_gate is None,
       'exit#0→exit#1 不越杆，hold_gate 应为 None（实测 %r）' % (node.hold_gate,))
    feed_pose(node, sink, 8.40, -7.95, -1.5708, 201.2, None)
    node.tick()
    ok(node.idx == 8 and node.stage == 'approach',
       '负样本：坐标不同的下一航点照样走 approach，实测 idx=%d stage=%s' % (
           node.idx, node.stage))

    # ── 8) 指令整形：斜坡起步、零速立刻生效、死区下限顶住 ─────────────────
    # 现场 155 轮判死的形态是"一拍从 0 顶到满指令 → 插件追不上 max_wheel_acceleration
    # → 轮子打滑攒欠账 → 一次 31.76 rad/s 的弹射，随后对任何指令全无声响"。
    # 这条用例不需要 Gazebo 就能钉住执行层：只看"发出去的那一帧是多少"。
    node.cmd_v = node.cmd_w = 0.0
    node.dt = 0.05
    full = Twist()
    full.linear.x = node.cruise
    node.publish(full, 'x')
    v1 = sink.cmd.linear.x
    node.publish(full, 'x')
    v2 = sink.cmd.linear.x
    ok(0.0 < v1 <= shut.RATE_V * 0.05 + 1e-9,
       '第一拍只能给到加速度上限 %.3f m/s，实测 %.3f' % (shut.RATE_V * 0.05, v1))
    ok(v2 > v1, '第二拍应比第一拍快（斜坡在走），实测 %.3f→%.3f' % (v1, v2))
    for _ in range(60):
        node.publish(full, 'x')
    ok(abs(sink.cmd.linear.x - node.cruise) < 1e-6,
       '斜坡收敛到巡航值 %.2f，实测 %.3f' % (node.cruise, sink.cmd.linear.x))
    node.publish(Twist(), 'x')
    ok(abs(sink.cmd.linear.x) < 1e-9,
       '零速不排队、当拍就生效（刹车不能被斜坡拖住），实测 %.3f' % sink.cmd.linear.x)
    tiny = Twist()
    tiny.angular.z = 0.05              # 低于死区的示例值：不能真的发出去
    node.cmd_w = 0.0
    for _ in range(30):
        node.publish(tiny, 'x')
    ok(abs(sink.cmd.angular.z - shut.MIN_W) < 1e-6,
       '低于 MIN_W 的转向请求要顶到死区下限 %.2f（发 0.05 等于只攒轮位欠账），实测 %.3f'
       % (shut.MIN_W, sink.cmd.angular.z))

    # ── 9) 原地转预算：转不够就挂起，不许无限画圈 ──────────────────────────
    # 喂一个"车一直在转、却永远转不到该朝向"的合成流（越转越过、过冲回来再越过），
    # watch_spin 看不出问题（每拍都有角速度进展），只有预算能拦住它。
    node.stage = 'align'
    node.idx = 7
    node.parked = False
    node.park_reason = ''
    node.escape_now = False
    node.rest_until = 0.0
    node.ref = node.aref = None
    node.aref_at = None
    node.reset_leg_stats()
    yaw = -1.5708
    ticks = 0
    while not node.parked and ticks < 4000:
        yaw += 0.021                    # 每拍都在真转（>0.02 的进展阈值），但转不到位
        feed_pose(node, sink, 8.40, -7.95, yaw, 300.0 + ticks * 0.05, None)
        node.tick()
        ticks += 1
    ok(node.parked, '合成"一直转却转不到位"的流应在 %d 拍内挂起，实测未挂起' % ticks)
    ok('累计' in node.park_reason,
       '挂起理由要写明是原地转预算到顶（画圈），实测：%r' % node.park_reason)
    ok(ticks * 0.05 <= 30.0,
       '预算 2π rad 换算成时间应 <=30 s，实测用了 %.1f s（%d 拍）' % (
           ticks * 0.05, ticks))
    ok(abs(sink.cmd.angular.z) < 1e-9,
       '挂起后只发零速，实测 w=%.2f' % sink.cmd.angular.z)

    # ── 10) 闭环积分：从 155 轮"画圈那一帧"起步，必须收敛到点 ────────────────
    # 上面九条都是"喂一帧、看一拍"的开环断言，钉不住画圈这件事本身 —— 画圈是指令
    # 与几何**长期**不自洽的产物。这一条不喂合成轨迹，而是拿节点自己发出去的那一帧
    # （已经过 shape 整形）按差速运动学把车推着走，再把推出来的位姿喂回节点：
    #   x += v·cosθ·dt，y += v·sinθ·dt，θ += ω·dt
    # 起点取 155 轮实测被预算挂起那一帧（reset#2：轮轴离目标 0.50 m、方向差 1.53 rad）。
    # 旧行车支 ω=1.5α 在 α=0.30、d=0.50 时只给 0.45，而"这条弧要过目标点"需要 0.53
    # —— 车实际半径 1.00 m 比需要的 0.85 m 宽，每一趟都从目标外侧绕过去，α 重新涨过
    # drive_tol 再原地转一次；两支来回切，2π 的原地转预算就是在这种切法里烧光的。
    # 现在 ω = 2v·sinα/d 与车速上限 v_cap = turn·d/(2|sinα|) 是同一条弧的两半，闭环
    # 应当直接收敛：一次原地转收方向 → 一段弧压到点上 → align → hold。
    #
    # 积分器按**轮轴**推状态而不是按底盘：差速车原地转的瞬时中心是轮轴，节点一切
    # 判定也用轮轴，所以真值是"轮轴沿车头方向走 v·dt、车头转 ω·dt"，底盘 = 轮轴
    # 往后收 axle·û(朝向)（朝向一转，底盘就绕着轮轴甩过去）。拿底盘当旋转中心喂上去
    # 会把转向那几十拍的轮轴位移抹成零，测出来的收敛是假的。
    node.parked = False
    node.park_reason = ''
    node.escape_now = False
    node.rest_until = 0.0
    node.ref = node.aref = None
    node.aref_at = None
    node.cmd_v = node.cmd_w = 0.0
    node.idx = 16
    node.stage = 'approach'
    node.reset_leg_stats()
    wx, wy, wth = node.seq[16][2], node.seq[16][3], node.seq[16][4]
    a = node.axle
    cx, cy, yaw = 11.9551, -13.0496, 0.633          # 155 轮实测被挂起的那一帧
    px, py = cx + a * math.cos(yaw), cy + a * math.sin(yaw)
    ticks, mile, reached_align = 0, 0.0, False
    while ticks < 1200 and not node.parked and node.stage != 'hold':
        s = 400.0 + ticks * 0.05
        feed_pose(node, sink, px - a * math.cos(yaw), py - a * math.sin(yaw),
                  yaw, s, None)
        node.tick()
        c = sink.cmd
        mile += abs(c.linear.x) * 0.05
        px += c.linear.x * math.cos(yaw) * 0.05
        py += c.linear.x * math.sin(yaw) * 0.05
        yaw = shut.wrap(yaw + c.angular.z * 0.05)
        if node.stage == 'align':
            reached_align = True
        ticks += 1
    ok(not node.parked,
       '闭环 %.1f s 内不该挂起（画圈就是靠预算挂起来兜底的），实测 parked=%s 理由=%r' % (
           ticks * 0.05, node.parked, node.park_reason))
    ok(node.stage == 'hold',
       '应跑完 approach→align 进 hold，实测 stage=%s idx=%d' % (node.stage, node.idx))
    ok(reached_align, '中途必须真的判到轮轴到位（转入过 align）')
    ok(node.spin_rad < 2.5,
       '本航点原地转只该用掉收方向的那一点额度，实测 %.1f rad（预算 %.1f）—— '
       '偏大就是在两支之间来回切着绕圈' % (node.spin_rad, shut.SPIN_BUDGET_RAD))
    cx, cy = px - a * math.cos(yaw), py - a * math.sin(yaw)
    d_fin = math.hypot(wx - cx, wy - cy)
    ok(d_fin < 0.20,
       '底盘最后应落在航点 (%.2f,%.2f) 附近，实测 (%.2f,%.2f) 偏 %.2f m' % (
           wx, wy, cx, cy, d_fin))
    ok(abs(shut.wrap(wth - yaw)) < shut.ALIGN_TOL + 1e-6,
       '车头朝向应转到 %.2f，实测 %.2f（差 %.3f rad）' % (
           wth, yaw, shut.wrap(wth - yaw)))
    ok(mile < 2.0,
       '这一段直线只有 0.50 m，实测里程 %.2f m —— 绕圈的直接读数就是里程远超几何距离' % (
           mile))

    # ── 11) 提前归零门：钉在 1016 轮真机逐拍流的那一拍上 ─────────────────────
    def old_law(err, cap=None):
        """改门之前那份转向律的原样复刻（只有 P + K_DAMP·实测角速度）。"""
        want = node.k_ang * err - shut.K_DAMP * node.yaw_rate
        lim = (node.turn if cap is None else cap) \
            if abs(err) >= shut.SPIN_SETTLE_BAND else shut.SPIN_SETTLE_RATE
        return shut.clip(want, lim)

    def spin_lagged(err0, law):
        """带一阶滞后的 align 相闭环：只用来当"加门之后不回退"的门禁。"""
        node.parked, node.park_reason = False, ''
        node.escape_now = False
        node.rest_until = 0.0
        node.ref = node.aref = None
        node.aref_at = None
        node.cmd_v = node.cmd_w = 0.0
        node.yaw_rate, node.yaw_meas_ok = 0.0, False
        node.idx = 7
        node.stage = 'align'
        node.reset_leg_stats()
        th = node.seq[7][4]
        yaw = shut.wrap(th - err0)
        w_act = 0.0
        ticks, flip_amp, err_hist = 0, 0.0, []
        while ticks < 2000 and not node.parked and node.stage == 'align':
            feed_pose(node, sink, 8.40, -7.95, yaw, 500.0 + ticks * 0.05, None)
            node.turn_cmd = law
            node.tick()
            err_hist.append(shut.wrap(th - node.pose[2]))
            w_act += (sink.cmd.angular.z - w_act) * 0.05 / shut.BRAKE_LAG
            yaw = shut.wrap(yaw + w_act * 0.05)
            ticks += 1
        node.turn_cmd = shut.ShuttleNode.turn_cmd.__get__(node)
        for k in range(1, len(err_hist)):
            if err_hist[k] * err_hist[0] < 0.0:
                flip_amp = max(flip_amp, abs(err_hist[k]))
        return {'ticks': ticks, 'parked': node.parked, 'why': node.park_reason,
                'flips': node.hunt_flips, 'flip_amp': flip_amp,
                'spin': node.spin_rad, 'stage': node.stage,
                'resid': shut.wrap(th - node.pose[2])}

    # 这一条的样本不是合成轨迹，是 /tmp/pk190_trace.txt 里 exit#3 align 段 t=+2.24 s
    # 那一行的实测三件套：朝向差 +0.09 rad（还要继续往正方向转）、指令 w=−0.04（已经
    # 翻负）、实测 ω=+0.25 rad/s（车头确实还在正转）。旧律在那一拍的取值是
    # want = 1.2·0.09 − 0.6·0.25 = −0.042 —— 和录到的 −0.04 对上，先证明这份复刻是
    # 真的在跑场内那一拍；接着车头被这条反向指令拽回 3.07→3.03，误差从 +0.07 又长回
    # +0.15，每长回来一次就是一个"回修"，5 次被 spin_guard 挂起（场内报的 ±0.37 rad
    # 翻转 5 次就是这个循环跑满的样子）。
    # 门的临界点 |err| = BRAKE_LAG·ω + ω²/(2·RATE_W) = 0.4·0.25+0.25²/3 = 0.121 rad，
    # 和阻尼翻负的临界点 K_DAMP·ω/k_ang = 0.125 rad 是同一件事的两种写法：前者说
    # "剩下的差错不过刹车行程"，后者说"阻尼项已经盖过 P 项"。所以那一拍的正确指令是
    # 归零让动量走完，不是 −0.04。
    # 为什么不写成"旧律必须挂起"：上面第 10 条那套积分器把指令当拍作用到车上，是
    # 最小相位的，跑不出场内那个死循环（实测旧律过冲 0.000 rad）—— 拿一个复现不出
    # 病灶的模型去断言"旧律会挂起"，只会逼我把模型越调越玄。定点核验只用录到的数，
    # 闭环那半留在下面当**不回退**的门禁（加了门以后原来收敛的场景仍须收敛）。
    node.yaw_rate = 0.25
    old_at_rec = old_law(0.09)
    ok(abs(old_at_rec - (-0.042)) < 0.005,
       '前置：旧律在"误差 +0.09、实测 ω +0.25"这一拍应算出场内录到的 −0.04，'
       '实测 %.3f —— 对不上就是这份复刻没在跑场内那一拍' % old_at_rec)
    ok(abs(node.turn_cmd(0.09)) < 1e-9,
       '同一拍上门的取值应是 0.0（归零让动量走完），实测 %.3f' % node.turn_cmd(0.09))
    for e in (0.12, 0.09, 0.05, 0.02):
        ok(abs(node.turn_cmd(e)) < 1e-9 and old_law(e) < 0.0,
           'ω=+0.25、误差 +%.2f rad：旧律发反向（%.3f）正是病灶，门应当归零'
           '（实测 %.3f）' % (e, old_law(e), node.turn_cmd(e)))
    # 两个方向的"门不该管闲事"：转错了方向要照旧给足修正；大角度转的行程远大于
    # 刹车行程，门必须整个不参与（场内 90° 那几拍的 |w| 峰 0.98 就是这一段）。
    node.yaw_rate = -0.25
    ok(abs(node.turn_cmd(0.09) - old_law(0.09)) < 1e-9,
       '误差 +0.09 而车头在反转（ω=−0.25）时门不该介入，实测门=%.3f 旧律=%.3f' % (
           node.turn_cmd(0.09), old_law(0.09)))
    node.yaw_rate = 0.60
    err_big = 2.32                        # 场内 exit#4 实录的方向差峰值
    ok(abs(node.turn_cmd(err_big) - old_law(err_big)) < 1e-9
       and node.turn_cmd(err_big) > 0.9,
       '2.32 rad 的大角度转（实测 ω=+0.60 时刹车行程 %.2f rad 远小于误差）门必须'
       '整个不参与，实测门=%.3f 旧律=%.3f' % (
           node.brake_travel(), node.turn_cmd(err_big), old_law(err_big)))
    node.yaw_rate = 0.0
    ok(abs(node.turn_cmd(0.09) - 0.108) < 0.01,
       '静止时（ω=0）刹车行程为 0，门不参与、按 P 项起步，实测 %.3f' % node.turn_cmd(0.09))

    # 闭环不回退：同一个滞后对象上新律必须照样转到位、一次翻转都不计。
    new = spin_lagged(1.6, node.turn_cmd)
    print('  闭环 %.1f s  挂起=%s 翻转=%d 过冲=%.3f rad 用了 %.2f rad 残差=%.3f 到=%s' % (
        new['ticks'] * 0.05, new['parked'], new['flips'], new['flip_amp'],
        new['spin'], new['resid'], new['stage']))
    ok(not new['parked'] and new['stage'] == 'hold',
       '加门之后原来收敛的场景仍须收敛（不回退），实测 stage=%s 理由=%r' % (
           new['stage'], new['why']))
    ok(new['flips'] == 0, '闭环翻转次数应为 0，实测 %d 次' % new['flips'])
    ok(abs(new['resid']) <= shut.ALIGN_TOL + 1e-6,
       '新律结束时车头应在 ALIGN_TOL 内，实测差 %.3f rad' % new['resid'])
    ok(new['spin'] < shut.SPIN_BUDGET_RAD * 0.5,
       '一次 1.6 rad 的原地转不该烧掉半个预算，实测 %.2f rad（偏大就是在荡）' % (
           new['spin']))

    print('\n汇总：FAIL %d 条' % len(FAILS))
    for f in FAILS:
        print('  FAIL: ' + f)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
