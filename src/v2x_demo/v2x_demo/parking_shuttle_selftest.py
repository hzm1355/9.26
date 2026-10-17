#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车场摆渡车合成数据自测（不需要 Gazebo，不 spin，不发 ROS）。

闭环长什么样：本脚本自己当那辆车 —— 每拍把当前位姿喂进 on_models，调一次 tick()，
读回它下发的 Twist，再按差速运动学积分出下一拍位姿。积分落在**驱动轮轴中点**上
（轮轴在底盘原点前方 vehicle.axle_ahead=0.40 m，实测），因为原地转的瞬时中心就是轮轴：
绕 z 原地转时底盘原点会绕它画一个 0.40 m 的圆。照"底盘原地不动"去积分，这台假车就
不会复现真车的那个偏置，节点的控制律缺陷会被假车悄悄盖掉。
所以断言的是"节点拿到真实反馈后会不会做错决定"，而不是脚本替它写好的剧本。
闸机那边用 BarrierState 合成：杆的开合直接改 state/at_target，节点只认这份实测状态。

覆盖：
  A 从等待位出发，第一站停在入口核验位，并且停稳后确实发零速（卡口的 frame_wait
    靠的就是这段零速，车边走边拍的话位置残差会超出取景余量）。
  B 杆 CLOSED 时车**绝不越过杆轴** y=-11.05 —— 顶杆就是任务 #77 那套轮位误差积累，
    这条是整个自测里最硬的断言。
  C 杆报 OPEN 且 at_target 后才继续走；一路到 P1 车位，在车位停留 ≈ stall_dwell_s。
  D 一整轮（进场→出场→复位）跑完 cycle 递增；周期没凑满 loop_period_s 时先在等待位
    停着等（这里把 period 改小来专门走这条分支）。
  E 拒绝分支：杆一直 CLOSED → 等过 GATE_WAIT_S 后倒车退回等待位，aborted 计数 +1，
    且退回过程中车确实往回走。
  F 顶住看门狗：位置被冻住（模拟楔住）而指令还在给油 → STUCK_S 之后出现零速松开段。
  G 全程不越出停车场的场界（lot x0..x1 / y0..y1），所有航点都在场内。
  H 顶住/作废的封顶：杆一直不落 → 连续作废到 MAX_ABORT_STREAK 就转停驶挂起，
    挂起后每一拍仍是零速（静默不刹，只有真消息能清掉目标轮速）。
  I 路线中段作废 → 退回后接回 back_index 那一格继续跑，**不许硬跳 entry#0**
    （旧代码写死 idx=0，车物理上却在出场段第 6 格，下一脚油门的方向是错的）。
  J 倒车段压着放平的杆 → 倒车前也判越杆：杆没抬就一条倒车指令都不发，等超时转停驶。
  K 车不在路线上、第一脚油门就要越杆 → approach 每拍按"当前位置→目标"的实际
    线段重算该看哪根杆（旧逻辑只在 next_point 里按相邻航点算一次，这一段是漏的）。
  L 新进程起步时车在杆轴北侧 → 按当前位置重新选起点（park() 承诺的"重启本节点会
    按当前位置重新选路"就是这一条）。不选路时它要去杆轴南侧的 entry#0，于是停等抬杆，
    而 manager 要等车到核验位才抬杆 —— 两边互等，读起来就是"重启后车死在原地"。
  M 极限环起步区（任务 #89）：轮轴离目标 0.40 m、方向差 0.27 rad —— 这个区间正好卡在
    旧的"边走边修"容差 0.30 以内、却已经超出 ω 饱和点 turn/1.5=0.233，饱和的 ω 让车
    走半径 0.71 m 的圆，比剩余距离还大，于是绕着航点转圈贴不上（在环实测绕 12 min）。
    M 断言三件事：能在有限拍数内转入 align、直线里程不超过绕圈的下限、以及转向完成后
    底盘**正好**落在航点上（轮轴补偿闭环成立的判据）。

用法（WSL）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 src/v2x_demo/v2x_demo/parking_shuttle_selftest.py [parking_zones.yaml] [模型名]
模型名省略时测的是 yaml 里那份默认三段（car_6 时代的基线）；传 car_2 / car_3 就整段
换成 shuttle.fleets 里那一队的航线来跑 —— 断言全部按"本模型自己的航线"取参照。
退出码 0=PASS 1=FAIL。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Quaternion
from rclpy.parameter import Parameter
from v2x_msgs.msg import BarrierState

import parking_shuttle_node as psn

DT = 0.05
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
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class Car:
    """假车：替掉 gzserver 的反馈与两个发布器，按差速运动学走。"""

    def __init__(self, node, freeze=False):
        self.node = node
        self.cmd = Sink()
        self.status = Sink()
        node.pub_cmd = self.cmd
        node.pub_status = self.status
        x, y, h = node.routes['entry'][0]
        self.x, self.y, self.yaw = x, y, h
        self.t = 0.0
        self.freeze = freeze
        self.min_y_crossed = None     # 车到过最北的位置，用来查有没有越过核验位

    def gate(self, gid, state, at_target, angle=0.0, target=0.0):
        s = BarrierState()
        s.gate_id = gid
        s.state = state
        s.at_target = at_target
        s.angle_rad = angle
        s.target_rad = target
        self.node.on_gate(gid, s)

    def step(self):
        self.t += DT
        self.node.sim_now = self.t
        m = ModelStates()
        p = Pose()
        p.position.x = self.x
        p.position.y = self.y
        # 绕 z 的四元数：diff_drive 与 /model_states 用的就是这一套
        p.orientation = Quaternion(w=math.cos(self.yaw / 2.0),
                                   z=math.sin(self.yaw / 2.0))
        m.name = [self.node.model]
        m.pose = [p]
        self.node.on_models(m)
        self.node.tick()
        c = self.cmd.msgs[-1]
        v, w = c.linear.x, c.angular.z
        if not self.freeze:
            # 无滑移约束在驱动轮轴上：轮轴中点沿车头方向走 v，底盘挂在它后方 axle 处。
            # 所以"原地转 w"会把底盘甩出一段弧 —— 真车实测转 90° 平移 0.578 m。
            # 这里必须照抄这一条，否则自测里的车永远贴得准航点，节点里那套轮轴补偿
            # 就没有对手，绕圈那个 bug 在合成数据里根本复现不出来。
            a = self.node.axle
            px = self.x + a * math.cos(self.yaw)
            py = self.y + a * math.sin(self.yaw)
            px += v * math.cos(self.yaw) * DT
            py += v * math.sin(self.yaw) * DT
            self.yaw += w * DT
            self.x = px - a * math.cos(self.yaw)
            self.y = py - a * math.sin(self.yaw)
        return v, w

    def run(self, ticks):
        for _ in range(ticks):
            self.step()


def make_node(cfg, model=''):
    # model 非空时走的是分队航线（shuttle.fleets.<model>），与真机上
    # `ros2 run v2x_demo parking_shuttle_node.py --ros-args -p model:=car_3` 同一条码路。
    ov = [Parameter('config_file', Parameter.Type.STRING, cfg)]
    if model:
        ov.append(Parameter('model', Parameter.Type.STRING, model))
    return psn.ShuttleNode(parameter_overrides=ov)


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'config',
        'parking_zones.yaml')
    model = sys.argv[2] if len(sys.argv) > 2 else ''
    if not os.path.exists(cfg):
        print('找不到配置 %s' % cfg)
        return 2
    rclpy.init()
    try:
        run(cfg, model)
    finally:
        rclpy.shutdown()
    print('\n%s  (%d 项失败)' % ('PASS' if not FAILED else 'FAIL', len(FAILED)))
    return 0 if not FAILED else 1


def run(cfg, model=''):
    import yaml
    pk = yaml.safe_load(open(cfg, encoding='utf-8'))['parking']
    boom_y = pk['gates'][0]['boom']['pivot'][1]
    lot = pk['lot']
    node = make_node(cfg, model)
    # 断言的参照物一律取自**本模型自己的航线**（分队后 car_2 的排队位在场外、入库位是
    # P1 而不是 P3），不能再去比对 yaml 里那份 car_6 时代的默认三段 —— 那会把"用的是
    # B 队航线"这件事本身当成错误。
    ent, rst = node.routes['entry'], node.routes['reset']
    wait_pt = ent[0]
    stall = ent[-1]
    # 核验位不是"entry#1"，而是"入闸前最后一格还压在杆轴南侧"的那一格 —— 按几何认它，
    # 别按序号认它。car_2 要等 car_3 停进 P3 之后才进场，entry#0 多了一个场外排队位，
    # 核验位因此挪到 entry#2；拿序号当参照就会把"用的是 B 队航线"读成节点走错格。
    hold_i = next(i for i in range(1, len(ent))
                  if ent[i][1] > boom_y >= ent[i - 1][1]) - 1
    queue = ent[hold_i]
    banner('G 航点全在场界内')
    pts = [p for r in node.routes.values() for p in r]
    # 场外只允许"进场首点=复位末点"那一格（入口以南的排队位），其它航点必须在 lot 内。
    pad = [(wait_pt[0], wait_pt[1]), (rst[-1][0], rst[-1][1])]
    out = [p for p in pts
           if not (lot['x0'] <= p[0] <= lot['x1'] and lot['y0'] <= p[1] <= lot['y1'])
           and (p[0], p[1]) not in pad]
    check(not out, '三段路线的 %d 个航点都在 lot 内（排队位例外：%s）' % (len(pts), pad),
          str(out))
    offq = [p for p in pts if (p[0], p[1]) == tuple(wait_pt[:2])
            and not (lot['y0'] <= p[1] <= lot['y1'])]
    check(all(abs(p[1] - lot['y0']) <= 1.30 for p in offq),
          '场外排队位最多南出 lot 1.30 m（实测 %s）' % [(p[1] - lot['y0']) for p in offq])

    banner('A/B 出发→停在入口核验位；杆 CLOSED 时绝不越过杆轴')
    car = Car(node)
    # 默认没有任何闸机状态：节点必须停在原地，不能"没状态就赌一把"
    reached_hold = False
    n_at_hold = 0
    for i in range(200):
        car.step()
        if node.stage == 'hold' and node.hold_gate == 'gate_in':
            reached_hold = True
            n_at_hold = len(car.cmd.msgs)
            break
    check(reached_hold, '第 %d 拍在核验位进入 hold 并认准要等 gate_in' % (i + 1),
          'stage=%s gate=%s 位置=(%.2f,%.2f)' % (node.stage, node.hold_gate, car.x, car.y))
    check(abs(car.x - queue[0]) < 0.25 and abs(car.y - queue[1]) < 0.25,
          '停位就是本队航线的核验位 (%.2f,%.2f)' % (queue[0], queue[1]),
          '实际 (%.2f,%.2f)' % (car.x, car.y))
    # 取样必须从"进 hold 的那一拍"往后数，不能拿全局末尾 20 条：转进 hold 只发生在
    # 一拍之内，往前数 20 条会一直吃到还在行车的那几帧，量到的自然不是"停稳"。
    car.run(20)
    v_zero = all(abs(c.linear.x) < 1e-9 for c in car.cmd.msgs[n_at_hold:])
    check(v_zero, 'hold 期间连续 %d 拍都是零线速（抓拍要车停稳）' % (len(car.cmd.msgs) - n_at_hold),
          '非零线速 %d 拍' % sum(1 for c in car.cmd.msgs[n_at_hold:] if abs(c.linear.x) > 1e-9))
    for _ in range(120):
        car.step()
    check(car.y < boom_y, '没有杆状态时车没越过杆轴 y=%.2f' % boom_y,
          '车到 y=%.2f' % car.y)
    check(node.idx == hold_i, '还挂在核验位（idx=%d）' % hold_i)

    banner('C 报 OPEN+at_target 才放行，一路到本队车位并在车位停留')
    car.gate('gate_in', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
    crossed = None
    last_entry = len(node.routes['entry']) - 1
    for k in range(1200):
        car.step()
        if crossed is None and car.y > boom_y:
            crossed = k
        if node.stage == 'hold' and node.seq[node.idx][:2] == ('entry', last_entry):
            break
    check(crossed is not None, '抬杆后第 %d 拍越过杆轴' % (crossed if crossed is not None else -1))
    at_stall = node.seq[node.idx][:2] == ('entry', last_entry)
    check(at_stall, '走到进场段最后一个航点（车位）',
          'idx=%d stage=%s 位置=(%.2f,%.2f)' % (node.idx, node.stage, car.x, car.y))
    check(abs(car.x - stall[0]) < 0.30 and abs(car.y - stall[1]) < 0.30,
          '停位就是本队航线的入库位 (%.2f,%.2f)' % (stall[0], stall[1]),
          '实际 (%.2f,%.2f)' % (car.x, car.y))
    dwell_ticks = 0
    while node.stage == 'hold' and dwell_ticks < 800:
        car.step()
        dwell_ticks += 1
    check(abs(dwell_ticks * DT - pk['shuttle']['stall_dwell_s']) < 1.0,
          '车位停留 %.1f s ≈ stall_dwell_s %.1f s' % (
              dwell_ticks * DT, pk['shuttle']['stall_dwell_s']),
          '%.1f s' % (dwell_ticks * DT))

    banner('D 一整轮跑完；周期没凑满 loop_period_s 时先在等待位停着')
    start_cycle = node.cycle
    guard = 0
    while node.cycle == start_cycle and guard < 12000:
        # 两道杆全程保持抬起：这一段只测"能不能自己跑完一轮"，闸机分支在 A~C 已测
        car.gate('gate_in', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        car.gate('gate_out', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        car.step()
        guard += 1
    check(node.cycle == start_cycle + 1, '第 %d 轮在 %d 拍（%.0f s 仿真）内跑完' % (
        node.cycle + 1, guard, guard * DT), 'cycle=%d idx=%d stage=%s' % (
        node.cycle, node.idx, node.stage))
    # 容差 0.15 而不是原来随手写的 0.3：0.3 宽到装不下这个 bug 的量级差 —— 保留
    # "转向点直接跳 align"那条捷径时，原地转会把底盘甩离航点 0.40·√2=0.57 m，
    # 而按轮轴中点判到位再转正，底盘残差只有几厘米。卡死在 0.15 才分得开这两种。
    check(abs(car.x - wait_pt[0]) < 0.15 and
          abs(car.y - wait_pt[1]) < 0.15,
          '一圈回到本队进场首点（复位段终点=进场段起点）(%.2f,%.2f)'
          % (wait_pt[0], wait_pt[1]),
          '(%.2f,%.2f)' % (car.x, car.y))

    # 周期分支单独钉成一个确定值：yaml 里 loop_period_s=120 s 大于一圈实测的 98 s，
    # 正常跑确实会走到这一支，但"等多久"取决于上一拍落在哪儿，断言不稳。所以这里把
    # 周期钉成"从现在起还要等 10 s"，等待时长才是可比的。
    node.cycle_start = node.sim_now
    node.period_s = 10.0
    node.idx = len(node.seq) - 1
    node.stage = 'hold'
    node.hold_gate = None
    node.hold_until = node.sim_now
    node.hold_started = node.sim_now
    car.gate('gate_in', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
    v0, _w0 = car.step()
    check(abs(v0) < 1e-9 and '凑周期' in car.status.msgs[-1].data,
          '周期末尾先停在等待位凑时间', 'v=%.2f %s' % (v0, car.status.msgs[-1].data))
    # 凑周期的文本只出现在"刚走到序列末尾"的那一拍，之后每拍都改成通用的
    # "停留还剩 X s"（同一段零速，两种说法）。所以计时要按状态量 cycle 来卡，
    # 拿文本当循环条件会在第二拍就退出去，量到 0.1 s。
    waited = 0
    while node.cycle == start_cycle + 1 and waited < 400:
        car.gate('gate_out', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        v, _ = car.step()
        if node.cycle == start_cycle + 1 and abs(v) > 1e-9:
            break
        waited += 1
    check(abs(waited * DT - 10.0) < 1.5, '凑周期等了 %.1f s（设 10 s）' % (waited * DT),
          '%d 拍，最后一条 %s' % (waited, car.status.msgs[-1].data))
    # 复位段终点与进场段起点是同一个 (x,y)（reset#2 = entry#0 = 等待位），
    # 按 next_point 的规矩"坐标相同=原地转点"，回绕后落在 align 而不是 approach ——
    # 两条都算"重新起一轮"，判据只卡 idx 归零与 cycle 递增，别把朝向段当成跳段。
    check(node.idx == 0 and node.stage in ('approach', 'align') and
          node.cycle == start_cycle + 2,
          '凑满后回到序列起点并计入下一轮', 'idx=%d stage=%s cycle=%d' % (
              node.idx, node.stage, node.cycle))
    node.destroy_node()

    banner('E 杆一直不落 → 等过 GATE_WAIT_S 倒车退回等待位')
    node2 = make_node(cfg, model)
    car2 = Car(node2)
    car2.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
    aborted_at = None
    for k in range(int(90.0 / DT)):
        car2.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car2.step()
        if node2.stage == 'back':
            aborted_at = k
            break
    check(aborted_at is not None, '第 %d 拍（%.0f s）转入 back 退回' % (
        aborted_at + 1, (aborted_at + 1) * DT), 'stage=%s' % node2.stage)
    check(node2.aborted == 1, '作废计数 +1')
    check(aborted_at * DT >= psn.GATE_WAIT_S, '等待时长达到 GATE_WAIT_S=%.0f s 才放弃' % (
        psn.GATE_WAIT_S), '%.1f s' % (aborted_at * DT))
    y_back = car2.y
    y_max = car2.y
    for _ in range(400):
        car2.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        if node2.stage != 'back':
            break
        car2.step()
        y_max = max(y_max, car2.y)
    check(car2.y < y_back - 0.2 or node2.idx == hold_i - 1,
          '确实往南退回（车从 y=%.2f 回到 %.2f）' % (y_back, car2.y))
    check(node2.idx == node2.back_index == hold_i - 1 and
          node2.stage in ('approach', 'align', 'hold'),
          '退回后接回的是核验位前一格 entry#%d（back_index），不是硬跳序列头' % (
              node2.seq[hold_i - 1][1]),
          'idx=%d back_index=%d 期望=%d stage=%s' % (
              node2.idx, node2.back_index, hold_i - 1, node2.stage))
    check(y_max < boom_y, '退回全程没越过杆轴 y=%.2f' % boom_y,
          '最北到 y=%.2f' % y_max)
    node2.destroy_node()

    banner('F 顶住看门狗：位置冻住而指令还在给油 → 松开一段零速')
    node3 = make_node(cfg, model)
    car3 = Car(node3, freeze=True)
    rest_seen = 0
    resumed = 0
    for k in range(int((psn.STUCK_S + psn.STUCK_REST_S + 4.0) / DT)):
        v, w = car3.step()
        if k * DT > psn.STUCK_S and abs(v) < 1e-9:
            rest_seen += 1
        elif abs(v) > 1e-9 and rest_seen:
            resumed += 1
    want = int(psn.STUCK_REST_S / DT)
    check(rest_seen > 0, '冻结位移后出现零速松开段（%d 拍）' % rest_seen)
    check(want - 2 <= rest_seen <= want + 2,
          '松开段时长 ≈ STUCK_REST_S %.0f s（实测 %d 拍，应在 %d±2）' % (
              psn.STUCK_REST_S, rest_seen, want))
    check(resumed > 0, '松开之后重新给油（%d 拍），不是一停到底' % resumed)
    node3.destroy_node()

    banner('G2 全程轨迹不出场界')
    node4 = make_node(cfg, model)
    car4 = Car(node4)
    for _ in range(6000):
        car4.gate('gate_in', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        car4.gate('gate_out', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        car4.step()
    inside = lot['x0'] - 0.9 <= car4.x <= lot['x1'] + 0.9 and \
        lot['y0'] - 0.9 <= car4.y <= lot['y1'] + 0.9
    check(inside, '跑了 %.0f s 后车仍在场内（含车体半长的余量）' % (6000 * DT),
          '(%.2f,%.2f)' % (car4.x, car4.y))
    check(node4.cycle >= 1, '这段时间里至少完整跑完一轮', 'cycle=%d' % node4.cycle)
    node4.destroy_node()

    banner('H 杆一直不落 → 连续作废到上限就停驶挂起，挂起后仍连发零速')
    node5 = make_node(cfg, model)
    car5 = Car(node5)
    k = 0
    for k in range(int(600.0 / DT)):
        car5.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car5.step()
        if node5.parked:
            break
    check(node5.parked, '第 %d 拍（%.0f s 仿真）转入停驶挂起' % (k + 1, (k + 1) * DT),
          'stage=%s 作废=%d' % (node5.stage, node5.aborted))
    check(node5.aborted >= psn.MAX_ABORT_STREAK,
          '作废 %d 次到上限 %d 才停（不再无限退-进）' % (
              node5.aborted, psn.MAX_ABORT_STREAK))
    for _ in range(40):
        car5.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car5.step()
    idle = [c for c in car5.cmd.msgs[-40:]]
    check(all(abs(c.linear.x) < 1e-9 and abs(c.angular.z) < 1e-9 for c in idle),
          '停驶后 40 拍仍全是零速（静默不刹，必须真发消息钉住目标轮速）')
    check(any('停驶挂起' in s.data for s in car5.status.msgs[-40:]),
          '状态行点名停驶挂起，面板上读得出为什么不动')
    node5.destroy_node()

    banner('I 路线中段作废 → 接回 back_index，不硬跳 entry#0')
    node6 = make_node(cfg, model)
    car6 = Car(node6)
    # 这三段（I/J/K）测的都是"跑途中的某个状态"，不是新进程起步，所以显式关掉
    # seat_at 的"按当前位置重新选路"（它只在该进程第一次拿到位姿时生效一次）。
    node6.seated = True
    # 把车放到出场段第 5 格 (6.80,-9.95)：它的上一格是 (6.80,-9.00)（出场西行段北移到
    # 触发圈外，见 parking_zones.yaml 里那段注释），退回要真倒 0.95 m，
    # 而旧代码退完直接把 idx 写成 0 —— 车在出场段，航点却跳回进场段起点。
    flat = next(i for i, row in enumerate(node6.seq) if row[:2] == ('exit', 5))
    tx, ty, th = node6.seq[flat][2], node6.seq[flat][3], node6.seq[flat][4]
    car6.x, car6.y, car6.yaw = tx, ty, th
    node6.idx = flat
    node6.stage = 'hold'
    node6.hold_gate = 'gate_out'
    node6.hold_started = node6.sim_now
    for _ in range(int(120.0 / DT)):
        car6.gate('gate_out', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car6.step()
        if node6.stage != 'back' and node6.aborted:
            break
    check(node6.aborted >= 1, '中段这一轮照样作废并退回', 'aborted=%d' % node6.aborted)
    check(node6.idx == node6.back_index == flat - 1 and node6.idx != 0,
          '退回后接回的是 exit#%d（flat %d），不是 entry#0' % (
              node6.seq[node6.idx][1], node6.idx),
          'idx=%d back_index=%d 期望=%d' % (node6.idx, node6.back_index, flat - 1))
    check(math.hypot(car6.x - node6.seq[flat - 1][2],
                     car6.y - node6.seq[flat - 1][3]) <= node6.radius + 0.05,
          '车确实倒回了上一航点 (%.2f,%.2f)' % (car6.x, car6.y))
    node6.destroy_node()

    banner('J 倒车段压着放平的杆 → 一条倒车指令都不发')
    node7 = make_node(cfg, model)
    car7 = Car(node7)
    node7.seated = True
    # 车已越过 gate_in 杆轴（在北侧 y=-10.60），back_to 在南侧 y=-12.20：
    # 倒回去就是从放平的杆上碾过去，必须停住等抬杆。
    car7.x, car7.y = 12.00, -10.60
    car7.yaw = math.pi / 2.0
    node7.idx = 2
    node7.stage = 'back'
    node7.back_to = (12.00, -12.20)
    node7.back_index = 1
    y0 = car7.y
    for _ in range(int((psn.GATE_WAIT_S + 3.0) / DT)):
        car7.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car7.step()
    check(node7.back_gate == 'gate_in', '倒车前算出了这一段要越 gate_in',
          'back_gate=%s' % node7.back_gate)
    check(abs(car7.y - y0) < 0.05, '全程没动（一条倒车指令都没发）',
          'y 从 %.2f 到 %.2f' % (y0, car7.y))
    check(node7.parked and any('倒车段' in s.data for s in car7.status.msgs[-20:]),
          '等过 GATE_WAIT_S 转停驶，理由点名"倒车段"')
    node7.destroy_node()

    banner('K 车不在路线上、第一脚油门就要越杆 → 按实际线段拦下，抬杆后放行')
    node8 = make_node(cfg, model)
    car8 = Car(node8)
    # 这一条测的是"已经选好路、正往 entry#0 开"的中途形态（比如退回之后），
    # 所以关掉新进程那次 seat_at；新进程该怎么做由 L 单独测。
    node8.seated = True
    # 进程刚起来的典型形态：航点在 entry#0（等待位，杆轴南侧），车却停在杆轴北侧。
    car8.x, car8.y = 12.00, -9.80
    car8.yaw = -math.pi / 2.0
    node8.idx = 0
    node8.stage = 'approach'
    y0 = car8.y
    for _ in range(int((psn.GATE_WAIT_S + 2.0) / DT)):
        car8.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car8.step()
    check(node8.pending_boom == 'gate_in', 'approach 每拍按"当前位置→目标"认出了这根杆',
          'pending_boom=%s' % node8.pending_boom)
    check(abs(car8.y - y0) < 0.05, '没顶着放平的杆往前开', 'y=%.2f' % car8.y)
    check(node8.aborted == 0, '车已在杆轴另一侧 → 走停等而不是倒车作废',
          'aborted=%d' % node8.aborted)
    check(any('停等抬杆' in s.data for s in car8.status.msgs[-40:]),
          '超时打了点名"停等抬杆"的行，日志与面板都不读成死机',
          '末行=%s' % (car8.status.msgs[-1].data if car8.status.msgs else ''))
    for _ in range(int(60.0 / DT)):
        car8.gate('gate_in', BarrierState.STATE_OPEN, True, 1.5708, 1.5708)
        car8.step()
        if car8.y < boom_y:
            break
    check(car8.y < boom_y, '杆一抬就继续走（不是永久锁死）', 'y=%.2f' % car8.y)
    check(not node8.parked, '这一路没触发停驶', 'parked=%s' % node8.parked)
    node8.destroy_node()

    banner('L 新进程起步：车在杆轴北侧 → 按当前位置重新选路，不与 manager 互等')
    node9 = make_node(cfg, model)
    car9 = Car(node9)
    # 现场重启的典型形态：车停在场里 (12.00,-9.80)（gate_in 杆轴北侧 1.25 m），而构造
    # 时写死的起点是 entry#0（杆轴南侧 1.75 m）。不重新选路的话，第一脚油门那一段正压
    # 着放平的杆：live_boom 认得出、于是停等抬杆；可 manager 只在车到核验位之后才抬杆 ——
    # 车等杆、杆等车，互等成死锁（这正是本轮在环里读到的"重启后车就是不动"）。
    car9.x, car9.y, car9.yaw = 12.00, -9.80, -math.pi / 2.0
    y_min = car9.y
    moved0 = (car9.x, car9.y)
    for _ in range(int(20.0 / DT)):
        car9.gate('gate_in', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car9.gate('gate_out', BarrierState.STATE_CLOSED, True, 0.0, 0.0)
        car9.step()
        y_min = min(y_min, car9.y)
    check(node9.seated, '第一次 tick 就按实际位姿选了一次起点')
    check(node9.seq[node9.idx][3] > boom_y,
          '选中的航点在杆轴北侧（不必抬杆就能走到）',
          'idx=%d → %s' % (node9.idx, node9.seq[node9.idx][:4]))
    check(y_min > boom_y - 0.05, '20 s 里没越过杆轴、没顶过放平的杆',
          '最南到 y=%.2f' % y_min)
    check(math.hypot(car9.x - moved0[0], car9.y - moved0[1]) > 0.20,
          '它真的在往前走（离开原点 %.2f m），不是停在原地等抬杆' % math.hypot(
              car9.x - moved0[0], car9.y - moved0[1]))
    check(node9.aborted == 0 and not node9.parked,
          '既不作废也不停驶', 'aborted=%d parked=%s' % (node9.aborted, node9.parked))
    node9.destroy_node()

    banner('M 极限环起步区：离轮轴目标 0.4 m、方向差 0.27 rad → 必须贴上去，不许绕圈')
    # 这一条钉的是 2026-10-13 在环上"绕圈 12 min 出不来"的那个死区。死区的成因是
    # 两个阈值不重合：行车支的 ω 在 |path_err| > turn_speed/1.5 = 0.233 时饱和，
    # 饱和后车走的是半径 v/ω = 0.71 m 的圆，而目标点离轮轴只有 0.40 m —— 目标落在
    # 转弯圆里，bearing 误差就稳定在极限环 e*=asin(ωd/2v) 上，车绕着点转永远贴不上；
    # 偏偏旧的 PATH_ALIGN_TOL = 0.30 比饱和点宽，这一带没人接管。现在 drive_tol 取的
    # 是饱和点本身，超了就停下原地转（原地转时轮轴不动、bearing 恒定，误差单调收敛）。
    node10 = make_node(cfg, model)
    car10 = Car(node10)
    node10.seated = True
    k = next(j for j, row in enumerate(node10.seq) if row[0] == 'entry' and row[1] == 6)
    row = node10.seq[k]
    fx, fy = node10.pivot_want(row[2], row[3], row[4])   # 轮轴该落在哪儿
    yaw = -0.27                       # 让"轮轴→目标"的方位角比车头大 0.27 rad
    px, py = fx - 0.40, fy            # 轮轴现在在目标正西 0.40 m
    car10.x, car10.y = px - node10.axle * math.cos(yaw), py - node10.axle * math.sin(yaw)
    car10.yaw = yaw
    node10.idx = k
    node10.stage = 'approach'
    path = 0.0
    ticks = 0
    arrived = False
    while ticks < int(40.0 / DT):
        car10.step()
        ticks += 1
        v = car10.cmd.msgs[-1].linear.x
        path += abs(v) * DT           # 只累直线里程：绕圈的特征就是它无上限地涨
        if node10.stage in ('align', 'hold'):
            arrived = True
            break
    check(arrived, '%.0f s 内贴到 arrive_radius 并转入 align（旧代码在这一带绕圈）'
          % (ticks * DT), 'stage=%s 里程=%.2f m' % (node10.stage, path))
    check(path <= 1.50, '走过的路程 %.2f m 在"绕圈"之下（绕圈会一直涨到预算用满）' % path)
    for _ in range(int(20.0 / DT)):
        car10.step()
        if node10.stage == 'hold':
            break
    d = math.hypot(car10.x - row[2], car10.y - row[3])
    check(d <= node10.radius + 0.03,
          '原地转完底盘正好落在航点 (%.2f,%.2f) 上，偏 %.3f m —— 轮轴补偿的意义就在这'
          % (row[2], row[3], d), '底盘=(%.2f,%.2f)' % (car10.x, car10.y))
    check(abs(psn.wrap(row[4] - car10.yaw)) <= psn.ALIGN_TOL + 0.02,
          '车头朝向转正（残差 %.3f rad）' % abs(psn.wrap(row[4] - car10.yaw)))
    node10.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
