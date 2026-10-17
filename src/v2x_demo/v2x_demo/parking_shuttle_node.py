#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务五（五）条目 2/3/4：停车场摆渡车 —— 让车真的把"进场→入库→出场"跑一遍。

  输入  /model_states                    gazebo_msgs/ModelStates   ~200 Hz 位姿（真相）
  输入  /parking/gate/<gid>/state        v2x_msgs/BarrierState     道闸实测状态（插件权威）
  输入  /clock                           rosgraph_msgs/Clock       仿真钟（BEST_EFFORT）
  输出  /<model>/cmd_vel                 geometry_msgs/Twist       差速底盘指令
  输出  /parking/shuttle_status          std_msgs/String           面板与取证用的进度行

一辆车一个实例：`-p model:=car_3` 决定它开哪辆、订哪路的 /model_states、往哪个 cmd_vel
发指令；航线按同一个模型名从 shuttle.fleets 里取（见下面航点语义）。

为什么车要"看见杆抬起来才走"，而不是按时间表往前开：
  条目 3 要的是真实物理碰撞。杆没抬就往前顶，20 Hz 的指令会把楔住的轮位误差一路攒起
  去 —— 那正是任务 #77 里"摆位瞬间沿车头冲到 8.53 m/s"的成因，只不过这次是车自己顶
  出来的。所以每条要越过杆轴的航段都先等 BarrierState 报 OPEN 且 at_target；等不到就
  倒车退回等待位、本轮作废（拒绝抬杆本来就该过不去，这也是条目 4 要演示的分支）。

航点语义（parking_zones.yaml 的 shuttle.entry_route/exit_route/reset_route）：
  每个点 = [x, y, 到位后的车头朝向]。到位判定用 arrive_radius，朝向用原地旋转 ——
  旋转点都选在车体扫掠圆 hypot(1.6,0.8)/2=0.894 m 内无障碍的位置（yaml 顶部已核）。
  三段路线首尾相接成一个周期：进场（在核验位停住等抓拍与抬杆）→ 入库停 stall_dwell_s
  → 出场 → 复位回入口等待位，凑满 loop_period_s 后再起一轮。

每个航点走 approach（开过去）→ align（原地转到该点规定的朝向）→ hold（该停的停、
要过杆的等杆）三段；hold 是唯一会长时间发零速的地方，卡口的 frame_wait 就是靠这段
零速才敢等 3 s 帧。

执行层（"别漂移、别原地画圈"落在这一层，不是落在增益上）：
  1) 每一拍的指令先过 shape()：非零请求顶到死区下限（MIN_V/MIN_W），再按
     RATE_V/RATE_W 做斜坡。阶跃指令超过 diff_drive 的 max_wheel_acceleration 时
     ODE 把轮子推进打滑，轮速积在插件里而车身没走，攒够就是一次弹射（155 轮实测
     31.76 rad/s 那一脚）；低于死区的指令（原来脱困用的 0.08 m/s）则是"指令看着在、
     车身不动"，同样只在攒欠账。零速不排队，刹车当拍生效。
  2) 转向带微分阻尼（K_DAMP·实测角速度）：纯 P 到最后一格会过冲两三度再来回修，
     在画面上读起来就是车头左右摆。
     阻尼之前还有一道刹车行程门（BRAKE_LAG）：剩下的差错不过"归零后还会多转的量"
     时不再发转 —— 回修指令生效后是同样的过冲，误差符号来回翻就被翻转计数挂起。
  3) 每个航点有原地转预算 SPIN_BUDGET_RAD，另有误差翻转计数：到了就挂起点名，
     绝不靠"再给大一点的 ω"自救 —— 加码是楔死实例的成因，不是解法。
  4) 横向偏差按"上一航点轮轴→本航点轮轴"这条车道线算，写进状态行（可取证读数的
     口径），超过 LAT_WARN 报一次、超过 LAT_PARK 挂起。
  5) 行车指令按圆弧可行性取值：ω = 2v·sinα/d 与车速 v_cap = turn·d/(2|sinα|) 是同一
     条过目标点的弧的两半，v 一旦超过 v_cap，ω 顶到 turn、实际半径 v/turn 就大于 d，
     车绕着目标点画圆贴不上去（bearing 稳定在极限环 asin(turn·d/2v)）。所以行车支
     永远 v ≤ v_cap；连 MIN_V 都撑不住时这一拍只走原地转（轮轴不动、d 不变、α 单调
     收敛）。倒车段同一条判据用 ω 上限 turn*0.5 复核。155 轮 reset#2 转到 6.3 rad 被
     预算挂起，就是缺了这条判据 —— 挂起本身是对的，画圈才是 bug。
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from v2x_msgs.msg import BarrierState

ALIGN_TOL = 0.05         # 航点上"车头朝向已到位"的容差 rad
PATH_ALIGN_TOL = 0.30    # 行车途中允许边开边修的方向误差 rad，超过它就停下原地转
GATE_WAIT_S = 30.0       # 在核验位最多等抬杆这么久，等不到就退回（识别超时才 8 s）
STUCK_S = 5.0            # 一直给前进指令却走不到 2 cm 这么久 → 判定顶住了
STUCK_REST_S = 3.0       # 顶住之后松开的时长，让轮位误差归零再试
BACK_SPEED = 0.15        # 退回等待位的倒车速度：比巡航还慢，位置残差留给抓拍余量
# 车心离杆轴不足这个距离、且横坐标落在杆的跨度内时，仍算"这段还在杆口上"。
# 少了这条余量，车心压在杆轴附近时 crosses_boom 会随位姿噪声一秒放行一秒拦，
# 而放行那一秒正好是车头顶着放平的杆灌油的秒（#77 的弹射就是这么攒出来的）。
# 它必须大于车沿行进方向的半长（摆渡车 1.6 m 长 → 0.8 m）：车心刚过杆轴 0.4 m 时
# 车尾还压在杆上方，这时候落杆就是砸车，停等也必须继续停着。
BOOM_CLEARANCE = 1.00
# 顶住重试与连续作废的封顶。没封顶时 watch_stuck 会无限"松 3 s 再试"，
# 日志里读起来就是车在核验位前永远怠速；上限到了只能停死并点名要人工介入。
MAX_STUCK_TRIES = 3
MAX_ABORT_STREAK = 3
# 旋转净空判据的余量：扫掠圆半径之外再留 0.10 m。不留余量的话，判据会在
# "间隙正好等于扫掠半径"那一瞬放行旋转，而 ODE 的接触恢复会把车直接顶进设施。
SPIN_MARGIN = 0.10
# 直行脱困的速度与行程上限。上限必须有：没有它，"前不空就退、退不动就再前"
# 会在两个方向之间来回抖，看着像车在自救，实际是在把轮位误差往两边同时攒。
# 0.08→0.12：原来这个数落在 MIN_V 死区以下，发出去等于"指令看着在、车身不动"，
# 正是 154 轮 5.5 min 只挪 0.014 m 的那个形态；取到死区之上才真的能把车挪开。
ESCAPE_V = 0.12
ESCAPE_MAX = 1.20
# 脱困的**位移进展**下限：STUCK_S 窗口里挪不到这个数就算"发了油门却挪不动"。
# 只有行程封顶（ESCAPE_MAX）是不够的 —— 154 轮实测：车楔在亭子角上、轮子打滑，
# 0.08 m/s 的指令 5.5 分钟只挪了 0.014 m，按 1.20 m 的行程封顶要磨 40 分钟才挂起，
# 日志里读起来就是"车在原地怠速、节点还活着"。进展判据把这一段压到 3 次 × 5 s。
ESCAPE_PROGRESS = 0.02
# ── 指令整形：不漂移、不画圈先要满足的执行层三条 ────────────────────────────
# 为什么写在常量里而不是随手改增益：这三条管的是"发出去的数物理上能不能被执行"，
# 增益管的是"该发多少"，两件事混在一起就会把执行层欠账当成控制律没调好。
#  * RATE_V / RATE_W —— world 里 diff_drive 的 max_wheel_acceleration 是 6 rad/s²，
#    轮半径 0.10 m → 轮面线加速度 0.6 m/s²；轮距 0.70 m → 绕轮轴的角加速度上限
#    0.6/0.35 = 1.71 rad/s²，取 1.5 留余量。原来 20 Hz 直接发阶跃（一拍从 0 顶到
#    0.45 m/s、或从 0 顶到 ±0.63 rad/s），插件追不上这个加速度，ODE 就把轮子推进
#    打滑：打滑那一段轮速积在插件里而车身没走，攒够了就是一次 30 rad/s 量级的弹射
#    （155 轮 car_2 干净平地上实测：反向第一脚 6 s 位移 0.40 m、转角 +1.077 rad、
#    轮子峰值 31.76 rad/s，随后对任何指令全无声响）。斜坡限住之后不打滑，横向漂移
#    的来源就只剩几何本身。
#  * MIN_V / MIN_W —— 死区下限。低于这个量的指令车不动，但插件照样把轮位目标往前
#    推，等于纯攒欠账（154 轮实测 0.08 m/s 的脱困指令 5.5 min 只挪 0.014 m）。
#    要么给到能动起来的数，要么老实停住，不发"看着像在工作"的指令。
#  * SPIN_BUDGET_RAD —— 单个航点的原地转预算。"转→被甩偏→再转"一旦成环，车就在
#    原地画圈而日志一切正常；预算到顶直接挂起点名，宁可停驶也不给无限转向。
RATE_V = 0.60
RATE_W = 1.50
MIN_V = 0.10
MIN_W = 0.15
SPIN_BUDGET_RAD = 2.0 * math.pi
# 转向的微分阻尼：纯 P（k_ang=1.2）转到最后一格会过冲两三度再来回修，来回修在画面
# 上就是"车头左右摆"，读起来跟画圈没法区分。用实测角速度反向抵一拍即可收敛。
K_DAMP = 0.60
# 收尾段的角速度上限。为什么要有这一段：反向指令从 +0.98 拉到 -0.98 要走完 RATE_W
# 斜坡，加上 ODE 的执行滞后，实测**刹车行程**约 0.7 rad（166 轮逐拍表 /tmp/pk166_status308.log
# 的 sim 1792123974.3→1792123975.6：指令已经反向饱和，车头还在原方向上又走了 0.71 rad）。
# 0.7 rad 是 drive_tol 0.30 的两倍多，
# 于是每一格"回修"都以同样的动量冲到对面去，幅度越荡越大，最后转满一圈把
# SPIN_BUDGET_RAD 烧光被挂起 —— 看起来像画圈，其实是刹不住。进了 SPIN_SETTLE_BAND
# 就把角速度顶到 SPIN_SETTLE_RATE：这一档的刹车行程 0.35²/(2·1.5)=0.04 rad，
# 乘上同样的滞后仍在带内，回修不再自激。代价是每个转向点多花约 2 s。
SPIN_SETTLE_BAND = 1.00
SPIN_SETTLE_RATE = 0.35
# "来回修"计数门限。不能用 ALIGN_TOL 当门限：MIN_W=0.15 rad/s 是死区下限，最后一格的
# 修正天生按 0.1 rad 量级台阶走，±0.1 rad 的量化残差不是"来回修"，是执行层的分辨率。
# 门限卡在 0.05 时车会在到位圈外反复触发计数，5 次就挂起（166 轮的另一种挂起形态）。
HUNT_FLIP_FLOOR = 0.15
# 制动提前量：从"这一拍把 ω 归零"到"车头真停下"，中间还会多转过的路分两段 ——
# BRAKE_LAG·ω 是执行滞后（diff_drive 把轮速目标拉到 0、ODE 才把力卸掉，166 轮逐拍
# 表实测 ≈0.4 s），ω²/(2·RATE_W) 是按斜坡斜率刹掉已有角速度的行程。
# 为什么 K_DAMP 顶不住这一段：阻尼项把抵消点算在 |err|=K_DAMP·ω/k_ang=0.6·0.35/1.2
# =0.175 rad，刹车行程 0.4·0.35+0.35²/3=0.181 rad —— 两个数本来就挨在一起，而
# yaw_rate 是位姿流低通后的滞后量，只要它比真值小一点，指令就直接翻负；反向指令又
# 要在 RATE_W 斜坡上走 0.2 s 才生效，等生效时车头已经冲过目标 0.2 rad，正好越过
# HUNT_FLIP_FLOOR —— 每格回修都被记一次"来回修"，5 次挂起（1016 轮 exit#3 实测
# 横向 -0.61 m 告警后 ±0.37 rad 翻转 5 次就是这个形态）。改成主动按行程归零：剩下
# 的差错不过刹车行程就不再发转，让动量自己走完，误差就不会翻到对面去。
BRAKE_LAG = 0.40
# 横向偏差的告警门与挂起门（相对"上一航点轮轴→本航点轮轴"这条车道线，单位 m）。
# 0.60 m 只是提示（车队会偶尔把等待位上的车顶偏一点），1.20 m 说明车已经不在自己
# 那条车道上，此时再原地转，扫掠圆就会扫到邻道设施 —— 停驶比试错便宜。
LAT_WARN = 0.60
LAT_PARK = 1.20

STATE_NAME = {BarrierState.STATE_CLOSED: 'CLOSED', BarrierState.STATE_OPENING: 'OPENING',
              BarrierState.STATE_OPEN: 'OPEN', BarrierState.STATE_CLOSING: 'CLOSING',
              BarrierState.STATE_FAULT: 'FAULT'}


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clip(v, lim):
    return max(-lim, min(lim, v))


class ShuttleNode(Node):
    def __init__(self, **kw):
        # **kw 给自测留的口子：合成位姿流直接喂 on_models，不用起 Gazebo。
        super().__init__('parking_shuttle', **kw)
        self.declare_parameter('config_file', '')
        self.declare_parameter('model', '')
        path = self.get_parameter('config_file').value or os.path.join(
            get_package_share_directory('v2x_demo'), 'config', 'parking_zones.yaml')
        pk = self.load_config(path)
        sh = pk['shuttle']
        self.model = self.get_parameter('model').value or sh['model']
        self.cruise = float(sh['cruise_speed'])
        self.turn = float(sh['turn_speed'])
        self.radius = float(sh['arrive_radius'])
        # 受控点是轮轴中点而不是底盘原点（见 parking_zones.yaml 里 vehicle.axle_ahead
        # 那段实测）。不补偿这一项时，一次 90° 原地转会把底盘甩出 0.578 m，比航点间距
        # 还大：车永远进不了 0.12 m 的到位圈，在"转→甩偏→再转"里绕圈。
        self.axle = float(pk['vehicle']['axle_ahead'])
        self.dwell_s = float(sh['stall_dwell_s'])
        self.period_s = float(sh['loop_period_s'])
        # 原地旋转用 P 控制而不是恒定角速度：恒定 0.35 rad/s 转到最后一格会过冲两三度，
        # 然后来回修。增益 1.2（10° 误差给 0.21 rad/s），顶到 turn_speed 为止。
        self.k_ang = 1.2

        # 分队航线：shuttle.fleets.<模型名> 存在就整段覆盖上面的默认三段。
        # 覆盖是**整段**而不是逐点合并 —— 逐点合并会让 entry 用 A 队、exit 用 B 队，
        # 出场从哪个位走就看谁先改过，两条链路的几何互相咬不上。
        fl = (sh.get('fleets') or {}).get(self.model) or {}
        self.routes = {'entry': [tuple(p) for p in (fl.get('entry_route') or sh['entry_route'])],
                       'exit': [tuple(p) for p in (fl.get('exit_route') or sh['exit_route'])],
                       'reset': [tuple(p) for p in (fl.get('reset_route') or sh['reset_route'])]}
        # 摊平成 [(路线名, 段内序号, x, y, 朝向)]；"该在车位停 12 s"这种判断按
        # (路线, 段内序号) 查，不比对坐标，改点位不会把停留点改丢。
        self.seq = [(r, i, x, y, h) for r in ('entry', 'exit', 'reset')
                    for i, (x, y, h) in enumerate(self.routes[r])]
        self.dwell_at = {('entry', len(self.routes['entry']) - 1)}

        # 哪条航段会越过杆轴：由闸机几何算，不写死（改点位只改 yaml）。
        # 杆的横越方向与 gen_parking_world.py 同一个规矩：入口杆向 +x、出口杆向 -x。
        self.booms = []
        for g in pk['gates']:
            px, py = g['boom']['pivot']
            sign = 1.0 if g['direction'] == 'in' else -1.0
            ex = px + sign * float(g['boom']['length'])
            self.booms.append((g['id'], py, min(px, ex), max(px, ex)))

        # 原地转的净空判据：车体绕轮轴扫出来的圆，半径 = hypot(半长 + 轮轴前移量, 半宽)。
        # 1.60 m 长的车在闸厅这种窄走廊里"原地转"不是无条件动作 —— 152 轮 car_6 就是
        # 顶在 parking_booth 的角上转不动（棱边间隙 0.020 m），而按模型中心距筛障碍时
        # 它显示成"1.5 m 外"。扫掠圆和 keepout 表一起用，见 spin_clear。
        bl, bw = [float(v) for v in sh.get('body', [1.60, 0.80])]
        self.sweep_r = math.hypot(bl / 2.0 + self.axle, bw / 2.0)
        self.keepout = [tuple(float(v) for v in k) for k in sh.get('keepout', [])]

        self.pose = None            # (x, y, yaw, 收讫墙钟)
        self.gates = {}             # gid -> (BarrierState, 收讫墙钟)
        self.sim_now = 0.0
        self.idx = 0
        self.stage = 'approach'     # approach / align / hold / back
        self.hold_until = 0.0
        self.hold_started = 0.0
        self.hold_gate = None
        self.cycle = 0
        self.cycle_start = None
        self.aborted = 0
        self.rest_until = 0.0
        self.ref = None
        self.ref_at = 0.0
        self.aref = None                # 原地转（v=0）专用的朝向参考，见 watch_spin
        self.aref_at = None
        self.escaping = 0               # 旋转净空不足时的直行脱困方向：0 无 / +1 前 / -1 后
        self.escape_from = None         # 脱困起点，用来封顶脱困距离
        self.escape_now = False         # 由 watch_spin 装填：实测转不动了才直着挪，见 escape_tick
        self.escape_ref = None          # 脱困进展参考点，用来封顶"发了油门挪不动"（轮子打滑）
        self.escape_ref_at = None
        self.back_to = None
        self.back_index = 0             # 退回之后从哪个航点接着跑（不再硬跳 entry#0）
        self.abort_streak = 0           # 连续作废轮数，跑完整一轮才清零
        self.stuck_tries = 0            # 连续"顶住→松开"次数，走到下一航点就清零
        self.parked = False             # 封顶到了：只发零速，等人工介入
        self.park_reason = ''           # 停驶理由要一直挂在状态行上，面板才读得出为什么不动
        self.back_gate = None           # 倒车段要越的那根杆（放平时不许倒回去）
        self.back_gate_at = None
        # 新进程的第一帧位姿到手时按车位实位置选起点，只做一次（测试里手工摆好车与
        # 航点序号时，把它置 True 就跳过）。
        self.seated = False
        # 本段要越的那根杆。hold 阶段等的是"下一段"，而下一段走到一半杆可能落下来
        # （manager 的保持窗、或人工落杆）—— 没有这个字段就只能一路顶到杆上。
        self.pending_boom = None
        # 已过杆轴而杆落下时只能停等（倒回去等于从放平的杆下穿过去）。无声的永远等待
        # 在日志里读成死机，所以记下起点，超过 GATE_WAIT_S 就点名"在等人工抬杆"。
        self.boom_stall = None
        self.boom_stall_logged = False
        # ── 指令整形与转向预算的状态（见上面 RATE_*/MIN_*/SPIN_BUDGET 那段）──
        self.cmd_v = 0.0              # 上一拍真正发出去的数，斜坡从这里往上走
        self.cmd_w = 0.0
        self.tick_at = None           # 上一拍的仿真钟，dt 只用 /clock 不用墙钟
        self.dt = 0.05                # 本拍的 dt，publish 里累加转向预算用
        self.yaw_meas = 0.0           # 上一拍实测车头朝向
        self.yaw_meas_ok = False      # 首帧不差分，见 measure_yaw
        self.yaw_rate = 0.0           # 低通后的实测角速度，给微分阻尼用
        self.spin_rad = 0.0           # 本航点已下发的原地转指令总量（rad）
        self.turn_side = 0.0          # 上一次误差符号，翻转计数就是"来回修"
        self.hunt_flips = 0
        self.lat_warned = False

        qos = QoSProfile(depth=10)
        self.pub_cmd = self.create_publisher(Twist, '/%s/cmd_vel' % self.model, 10)
        self.pub_status = self.create_publisher(String, '/parking/shuttle_status', qos)
        self.create_subscription(
            ModelStates, '/model_states', self.on_models,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for gid, _py, _x0, _x1 in self.booms:
            self.create_subscription(BarrierState, '/parking/gate/%s/state' % gid,
                                     lambda m, k=gid: self.on_gate(k, m), qos)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            '[shuttle] %s 就绪：航点 %d 个（进场 %d / 出场 %d / 复位 %d），'
            '巡航 %.2f m/s、原地转 %.2f rad/s、到位半径 %.2f m、车位停 %.1f s、'
            '周期 %.0f s｜杆轴 %s｜配置 %s' % (
                self.model, len(self.seq), len(self.routes['entry']),
                len(self.routes['exit']), len(self.routes['reset']),
                self.cruise, self.turn, self.radius, self.dwell_s, self.period_s,
                '/'.join('%s@y=%.2f' % (b[0], b[1]) for b in self.booms), path))

    @staticmethod
    def load_config(path):
        import yaml
        with open(path, encoding='utf-8') as fh:
            return yaml.safe_load(fh)['parking']

    # ── 输入 ──────────────────────────────────────────────────────────────
    def on_models(self, m):
        if self.model not in m.name:
            # 车不在位姿表里 = world 里没这个模型（gen_parking_world.py 没重跑）。
            # 只报一次：200 Hz 的位姿流刷屏会把日志里真正有用的那几行冲掉。
            if not getattr(self, '_no_model', False):
                self._no_model = True
                self.get_logger().error(
                    '[shuttle] /model_states 里没有 %s，停车等待。摆渡车由 '
                    'tools/gen_parking_world.py 内联进 world，改完要重跑并整栈重启。'
                    % self.model)
            return
        self._no_model = False
        p = m.pose[m.name.index(self.model)]
        q = p.orientation
        self.pose = (p.position.x, p.position.y,
                     math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
                     time.monotonic())

    def on_gate(self, gid, s):
        self.gates[gid] = (s, time.monotonic())

    def on_clock(self, m):
        self.sim_now = m.clock.sec + m.clock.nanosec * 1e-9

    # ── 判定 ──────────────────────────────────────────────────────────────
    def boom_side(self, gid, y):
        """车在这根杆的哪一侧（沿本段行进方向）：+1 已过杆，-1 还没到，0 查不到这根杆。
        行进方向由"上一个航点→当前目标"的 y 差决定 —— 过杆之后再倒车等于从杆下面穿回去，
        那比停着等更糟，所以这个符号决定中途落杆时是退回还是停等。"""
        for gid_b, py, _x0, _x1 in self.booms:
            if gid_b == gid:
                r, i, tx, ty, th = self.seq[self.idx]
                prev = self.seq[self.idx - 1] if self.idx > 0 else self.seq[self.idx]
                sgn = 1.0 if ty >= prev[3] else -1.0
                return sgn * (1.0 if y > py else -1.0)
        return 0.0

    def crosses_boom(self, a, b):
        """航段 a→b（各为 (x, y)）是否穿过某道闸横放的杆。返回 gate_id 或 None。"""
        for gid, py, x0, x1 in self.booms:
            if (a[1] - py) * (b[1] - py) >= 0.0:     # 两端同侧，不穿越
                continue
            t = (py - a[1]) / (b[1] - a[1])
            if x0 <= a[0] + t * (b[0] - a[0]) <= x1:
                return gid
        return None

    def live_boom(self, x, y, tx, ty):
        """**当前位置→目标**这条实际线段要不要看某道闸的脸色。
        比原来"只在 next_point 里按相邻航点算一次"强的地方：进程刚起来、或作废退回后
        车物理上不在路线上（本轮实测停在通道带 (8.24,-9.51) 而航点跳到 entry#0），
        第一脚油门要越杆时原逻辑根本不知道有杆。"""
        gid = self.crosses_boom((x, y), (tx, ty))
        if gid:
            return gid
        for gid_b, py, x0, x1 in self.booms:
            if x0 <= x <= x1 and abs(y - py) < BOOM_CLEARANCE:
                return gid_b
        return None

    def boom_up(self, gid):
        """杆是不是真的抬到位了。读插件实测状态，不读 manager 的逻辑态 ——
        条目 3 的取证要看的就是"指令角 vs 实测角"。"""
        got = self.gates.get(gid)
        if got is None:
            return False, '没收到 %s 的状态（插件没加载或 gzserver 没起）' % gid
        s, wall = got
        if time.monotonic() - wall > 3.0:
            return False, '状态 %.1f s 未更新' % (time.monotonic() - wall)
        if s.state == BarrierState.STATE_OPEN and s.at_target:
            return True, 'OPEN %.2f rad' % s.angle_rad
        return False, '%s 实测 %.2f 目标 %.2f fault=%d' % (
            STATE_NAME.get(s.state, '?'), s.angle_rad, s.target_rad, s.fault)

    # ── 主控 ──────────────────────────────────────────────────────────────
    def seat_at(self, x, y):
        """新进程按车位实位置选起步航点，只在本进程第一次拿到位姿时调用。

        为什么必须有：上一版一律从 entry#0 起步，而 entry#0 在 gate_in 杆轴的**南**侧。
        进程重启时车往往还在场里（本轮实测停在 (8.83,-8.07)，在杆轴北侧 2.98 m），
        于是第一脚油门的线段正好压着放平的杆 —— live_boom 认得出这根杆并停等，可
        manager 只在车到达核验位之后才抬杆，两边互等就是死锁：车等杆、杆等车。
        停在原地不动不算安全，它只是把死锁写得像"节点坏了"。按最近航点重新选路之后，
        车从自己脚下这一段接着往下跑，不需要谁先让步。
        """
        j = min(range(len(self.seq)),
                key=lambda k: math.hypot(self.seq[k][2] - x, self.seq[k][3] - y))
        d = math.hypot(self.seq[j][2] - x, self.seq[j][3] - y)
        if j != self.idx or self.stage != 'approach':
            self.get_logger().info(
                '[shuttle] 按当前位置 (%.2f,%.2f) 重新选路：从 %s#%d 起步'
                '（离它 %.2f m；原计划 %s#%d）' % (
                    x, y, self.seq[j][0], self.seq[j][1], d,
                    self.seq[self.idx][0], self.seq[self.idx][1]))
        self.idx = j
        self.stage = 'approach'
        # 这些都是"上一段"的遗留判定，重新选路之后必须清空，否则会拿旧段的答案
        # 去判新段该不该等杆。
        self.hold_gate = self.pending_boom = self.back_gate = None
        self.boom_stall = None
        self.boom_stall_logged = False
        self.back_gate_at = None
        self.ref = None
        self.rest_until = 0.0
        self.stuck_tries = 0
        self.cycle_start = None
        self.reset_leg_stats()

    def pivot_now(self, x, y, yaw):
        """轮轴中点（原地转的瞬时中心）现在的实际位置。"""
        return (x + self.axle * math.cos(yaw), y + self.axle * math.sin(yaw))

    def pivot_want(self, tx, ty, th):
        """要让底盘停在 (tx,ty) 且车头朝向 th，轮轴中点必须落在哪儿。
        关系是 底盘 = 轮轴 − axle·û(朝向)（轮轴在车头方向前方 0.40 m，实测端点
        (8.863,-7.472)@1.57 → (9.108,-6.761)@-2.24 与这个式子差 0.004 m），
        所以反解出轮轴目标 = 航点 + axle·û(th)。转向在轮轴上原地做（轮轴不动、
        底盘绕它摆），轮轴一到位、车头一转正，底盘就正好落在航点上。"""
        return (tx + self.axle * math.cos(th), ty + self.axle * math.sin(th))

    def brake_travel(self):
        """这一拍把 ω 归零后，车头还会多转过的角度（rad）。见 BRAKE_LAG 那段。"""
        w = abs(self.yaw_rate)
        return BRAKE_LAG * w + w * w / (2.0 * RATE_W)

    def turn_cmd(self, err, cap=None):
        """把"还差多少朝向"换成角速度指令：P + 实测角速度微分阻尼 + 收尾降档。

        阻尼项用 /model_states 的实测 yaw 差分而不是 odom：这一路 odom 是转发给
        gazebo_ros 的底盘量，卡死时它和 cmd 一起变哑，只有位姿流是真值。低通取
        0.5/0.5 是为了压掉 200 Hz 位姿流在差分上的量化毛刺。

        在阻尼之前先过一道刹车行程门：已经在朝目标转、剩下的差错不过这一拍的刹车
        行程时归零不发，让动量走完。少了这道门，指令会在 |err|=0.175 处翻负去"回修"
        一段本来就不该修的量，回修生效后又是同样的过冲 —— 误差符号每来回一次就被
        spin_guard 记一次翻转，5 次直接挂起。
        """
        if err * self.yaw_rate > 0.0 and abs(err) <= self.brake_travel():
            return 0.0
        want = self.k_ang * err - K_DAMP * self.yaw_rate
        # 收尾带内换成"刹得住"的档位（见 SPIN_SETTLE_BAND 那段）
        lim = (self.turn if cap is None else cap) if abs(err) >= SPIN_SETTLE_BAND \
            else SPIN_SETTLE_RATE
        return clip(want, lim)

    def measure_yaw(self, yaw, dt):
        """每拍更新实测角速度（低通一阶），供 turn_cmd 的微分阻尼用。

        两个必须：
        1) 首帧不差分。初始 0.0 和真实车头之间是天生的差，拿它除一个 tick 会得到
           三四十 rad/s 的假角速度，阻尼项直接把第一次转向压成反向。
        2) 一拍内转过 4·turn_speed·dt 的量判成"位姿瞬移"（人工摆位、respawn、
           或弹射那一拍），角速度归零而不是当成真的在转 —— 把摆位当成转动喂给
           阻尼，下一拍的指令就是拿一个不存在的角速度在刹车。
        """
        if not self.yaw_meas_ok:
            self.yaw_meas, self.yaw_rate, self.yaw_meas_ok = yaw, 0.0, True
            return
        if dt > 0.0:
            d = wrap(yaw - self.yaw_meas)
            if abs(d) > 4.0 * self.turn * dt:
                self.yaw_rate = 0.0
            else:
                self.yaw_rate = 0.5 * self.yaw_rate + 0.5 * (d / dt)
        self.yaw_meas = yaw

    def spin_guard(self, err):
        """本航点的原地转预算与"来回修"计数。返回 None 表示可以继续转，
        返回字符串就是理由，调用方据此挂起。

        两条判据都要：预算管的是"一个方向一直转却转不完"（限环、被顶住后继续灌），
        翻转计数管的是"转过头再倒回来、来回修却不收敛"。后者位移和转角都好看，
        只有误差符号的翻转次数能读出来。计数门限用 HUNT_FLIP_FLOOR 而不是 ALIGN_TOL：
        死区下限 MIN_W 让最后一格按 0.1 rad 的台阶走，那种残差是执行层的分辨率，
        不是"来回修"。
        """
        side = 1.0 if err > 0.0 else -1.0
        if self.turn_side and side != self.turn_side and abs(err) > HUNT_FLIP_FLOOR:
            self.hunt_flips += 1
        self.turn_side = side
        if self.spin_rad > SPIN_BUDGET_RAD:
            return ('本航点已累计下发 %.1f rad 原地转指令（预算 %.1f rad）却还没转到位：'
                    '位置 (%.2f,%.2f) 车头 %+.2f —— 原地画圈，挂起等人工，不再加码' % (
                        self.spin_rad, SPIN_BUDGET_RAD, self.pose[0], self.pose[1],
                        self.pose[2]))
        if self.hunt_flips > 4:
            return ('同一航点方向误差已翻转 %d 次（%.2f rad）却没收敛 —— '
                    '过冲—回修的死循环，需要人工看 k_ang/阻尼与地面摩擦' % (
                        self.hunt_flips, err))
        return None

    def reset_leg_stats(self):
        """换航点（推进、重新选路、作废退回）时清掉本航点的转向预算与横向告警。

        预算必须**按航点**清零而不是按阶段：配对转向点的 approach 转向和 align
        转向属于同一个航点，按阶段清零等于给画圈开了两倍额度。
        """
        self.spin_rad = 0.0
        self.hunt_flips = 0
        self.turn_side = 0.0
        self.lat_warned = False

    def cross_track(self, px, py):
        """轮轴到"上一航点轮轴→本航点轮轴"这条车道线的垂距（带符号，+ 在左侧）。

        漂移要能报数才能证明"没漂"：状态行里带上这个数，看板和取证脚本读的是
        同一个口径（相对车道线而不是相对某个航点）。上一航点与本航点重合时
        （配对转向点）车道线退化，就按车头方向的垂线算，仍然是横向偏差。
        """
        r, i, tx, ty, th = self.seq[self.idx]
        prev = self.seq[self.idx - 1] if self.idx > 0 else self.seq[self.idx]
        ax, ay = self.pivot_want(prev[2], prev[3], prev[4])
        bx, by = self.pivot_want(tx, ty, th)
        dx, dy = bx - ax, by - ay
        ln = math.hypot(dx, dy)
        if ln < 1e-6:
            # 车道线退化（配对转向点：相邻两格同坐标）：改按"过本航点、沿该点朝向
            # 的那条线"算垂距，仍然是横向偏差，口径不变。
            return -(px - bx) * math.sin(th) + (py - by) * math.cos(th)
        return ((px - ax) * dy - (py - ay) * dx) / ln

    def tick(self):
        if self.pose is None or time.monotonic() - self.pose[3] > 2.0:
            self.stop('没拿到 %s 的位姿（gzserver 没起或模型名不对）' % self.model)
            return
        if self.parked:
            # 封顶之后只剩这一条出路：连发零速把轮位目标钉死，别再往前顶。
            # 状态行每次都带上停驶理由 —— 面板只看得见最后一行，理由只打一次
            # 就等于"车莫名其妙不干了"。
            return self.publish(Twist(), '停驶挂起：%s' % self.park_reason)
        x, y, yaw, _ = self.pose
        # dt 只用仿真钟，不用墙钟：RTF 不等于 1 时两个轴的量级差会影响斜坡斜率，
        # 而物理积分只跟着 /clock 走（同 rest_until 那条教训）。首拍没有参考，按
        # 定时器周期 0.05 s 起算。
        dt = 0.05 if self.tick_at is None else clip(
            self.sim_now - self.tick_at, 0.10)
        if dt <= 0.0:
            dt = 0.001
        self.tick_at = self.sim_now
        self.dt = dt
        self.measure_yaw(yaw, dt)
        if not self.seated:
            self.seated = True
            self.seat_at(x, y)
        # 松开时长必须走仿真钟：rest_until 是 watch_stuck 用 sim_now 设的，物理积分也只
        # 跟着 sim_now 走。拿墙钟去比 sim_now 的时间戳，两个轴的量级差九个数量级，
        # 条件恒假 —— 松开段永远不会生效，车会一直顶着杆灌油（正是 #77 的成因）。
        # 时钟停摆时这一支也不会误判：/clock 一停，/model_states 同源自 gzserver 也会停，
        # 上面那条位姿过期分支就先接管了，watch_stuck 更不会因为"时间没走"判车卡住。
        if self.sim_now < self.rest_until:
            self.stop('顶住了，松开 %.1f s' % (self.rest_until - self.sim_now))
            return

        r, i, tx, ty, th = self.seq[self.idx]
        # 到位判定与行车误差都按轮轴中点算：见 pivot_want 那段。底盘离航点还差多少
        # 由"轮轴到位 + 车头转正"两件事共同决定，缺了这一步就是绕圈（#89 实测）。
        fx, fy = self.pivot_want(tx, ty, th)
        px, py = self.pivot_now(x, y, yaw)
        dist = math.hypot(fx - px, fy - py)
        cmd = Twist()
        note = ''

        if self.stage == 'approach':
            # 中途落杆的处理必须在"到没到航点"之前判：这一段要越的杆一旦放平，往前顶
            # 就是撞杆，往后退又要看是不是已经过了杆轴 —— 两个分支都得先于行车指令。
            # 要越哪根杆每 tick 按**实际线段**重算，不用 next_point 里预先记下的那根：
            # 进程起来时车不在路线上、或作废退回后航点跳回序列头，线段照样可能压着杆。
            self.pending_boom = self.live_boom(x, y, tx, ty)
            if self.pending_boom:
                up, why = self.boom_up(self.pending_boom)
                if up:
                    self.boom_stall = None
                    self.boom_stall_logged = False
                elif self.boom_side(self.pending_boom, y) > 0.0:
                    # 车已在杆轴另一侧：倒车等于从放平的杆下面穿回去，比停着更危险，
                    # 所以这一支只能等 —— 等 manager 的安全侧或人工把杆重新抬起来。
                    if self.boom_stall is None:
                        self.boom_stall = self.sim_now
                    stall = self.sim_now - self.boom_stall
                    if stall > GATE_WAIT_S and not self.boom_stall_logged:
                        self.boom_stall_logged = True
                        self.get_logger().warn(
                            '[shuttle] 已过 %s 杆轴而杆落下（%s），停等人工抬杆已 %.0f s'
                            % (self.pending_boom, why, stall))
                    return self.publish(
                        Twist(), '已过 %s 杆轴而它落下了（%s），停等抬杆 %.0f s'
                                 % (self.pending_boom, why, stall))
                else:
                    return self.begin_back('%s 在本段中途落下（%s）' % (
                        self.pending_boom, why), '杆中途落下，准备退回')
            if dist <= self.radius:
                self.stage = 'align'
                self.ref = None
                self.stuck_tries = 0        # 走到底了，之前的顶住次数不算了
                note = '到位 %.2f m' % dist
            else:
                path_err = wrap(math.atan2(fy - py, fx - px) - yaw)
                # 不允许"带着饱和的角速度行车"：ω 顶到 turn_speed 时车走的是半径
                # v/ω = 0.25/0.35 = 0.71 m 的圆，只要目标点比这个半径近，车就绕着它
                # 转圈而永远贴不上（bearing 误差会稳定在 e*=asin(ωd/2v) 那个极限环上）。
                # 行车支的增益是 1.5，所以 |path_err|>turn/1.5 就是饱和点 —— 把它当成
                # 真正的"边走边修"上限，超过就停下原地转（原地转时轮轴不动，bearing
                # 恒定，e 单调收敛）。2026-10-13 08:52 那次绕圈就是 e≈0.30 落在
                # PATH_ALIGN_TOL 0.30 以内却已经饱和，两边都没管。
                drive_tol = min(PATH_ALIGN_TOL, self.turn / 1.5)
                lat = self.cross_track(px, py)
                if abs(lat) > LAT_PARK:
                    return self.park(
                        '轮轴已偏离本段车道 %.2f m（挂起上限 %.2f m），位置 (%.2f,%.2f)：'
                        '继续行车或原地转都会扫到邻道设施，请确认是不是被其他车顶出车道' % (
                            lat, LAT_PARK, x, y))
                if abs(lat) > LAT_WARN and not self.lat_warned:
                    self.lat_warned = True
                    self.get_logger().warn(
                        '[shuttle] 横向偏差 %.2f m 超过告警门 %.2f m（本段 %s#%d），'
                        '车正在被带偏，先报一次' % (lat, LAT_WARN, r, i))
                # 圆弧可行性（#90 的现场：reset#2 绕着轮轴目标转圈，转 6.3 rad 被预算挂起）。
                # pure pursuit 的 ω = 2v·sinα/dist 对应的圆弧半径 R = dist/(2 sinα)，这条弧
                # 天生过目标点。可 ω 一旦顶到 turn，实际半径就变成 v/turn、和 R 脱钩：
                # v/turn > dist 时车绕着目标点画圆而永远贴不上去，bearing 误差稳定在
                # α* = asin(turn·dist/2v) 这个极限环上 —— 而 α* 常常正好跨在 drive_tol 0.30
                # 两侧，于是"行车绕圈 / 原地转收敛一点"来回切，原地转的额度就是这么烧光的。
                # 所以先算"ω 不饱和的最高车速" v_cap = turn·dist/(2|sinα|)，行车指令不许
                # 超过它（超过就是绕圈）。连死区下限 MIN_V 都会让 ω 饱和（v_cap < MIN_V）
                # 时，这一拍只剩原地转一条路：原地转绕的就是轮轴本身，轮轴不动、dist 不变，
                # α 单调收敛到 v_cap 重新够得着 MIN_V，行车支自己恢复几何可行。
                v_cap = self.turn * dist / max(
                    2.0 * abs(math.sin(path_err)), 1e-3)
                if abs(path_err) > drive_tol or v_cap < MIN_V:
                    # 这一支的 v=0、只有 ω，正好整个落在 watch_stuck 的盲区里，所以
                    # 转不动的看门狗是 watch_spin；净空不足时由 watch_spin 装填
                    # escape_now，下一拍先直着挪到转得开的地方（见 escape_tick）。
                    # 为什么不在这里做**预测式**净空检查（spin_clear 一失败就不让转）：
                    # 153 轮离线审计把 17 个航点按 arrive_radius 0.12 一圈采了样 —— 调好的
                    # 路线里 entry#3/#4 的扫掠圆对入口铰点柱只剩 0.03 m，硬卡预测会把
                    # 这两个真 90° 转直接否掉，车改走直行脱困，把上一段调出来的抓拍几何
                    # 莫名改坏。预测只用来**选脱困方向**，不用来否决转向。
                    if self.escape_now and self.escape_tick(x, y, yaw):
                        return
                    why = self.spin_guard(path_err)
                    if why:
                        return self.park(why)
                    cmd.angular.z = self.turn_cmd(path_err)
                    note = ('方向差 %.2f rad，原地转（横向 %.2f m）' % (
                        path_err, lat) if abs(path_err) > drive_tol else
                        '距 %.2f m 内车速下限也会绕圈，原地转收方向（横向 %.2f m）' % (
                            dist, lat))
                    if self.watch_spin(x, y, yaw, cmd.angular.z):
                        return self.publish(Twist(), '原地转卡住次数到上限，停驶')
                else:
                    # 接近航点降速：满速 0.45 m/s 配 0.12 m 的到位半径会冲过去，
                    # 冲过头再倒回来，核验位上的位置残差就超出抓拍余量了。下限取
                    # MIN_V 而不是原来的 0.06：低于死区的速度只会让插件空推轮位目标。
                    # ω 用运动学一致的 2v·sinα/dist 而不是定增益 1.5α：这个形式和上面
                    # 的 v_cap 是同一件事的两半——按 v 反解出的 ω 落在弧上，饱和判据才成立。
                    cmd.linear.x = max(MIN_V, min(
                        self.cruise, max(MIN_V, dist / 0.6), v_cap))
                    cmd.angular.z = clip(
                        2.0 * cmd.linear.x * math.sin(path_err) / max(dist, 1e-3),
                        self.turn)
                    note = '行车 距 %.2f m 横向 %.2f m' % (dist, lat)
                    if self.watch_stuck(x, y, cmd.linear.x):
                        return self.publish(Twist(), '顶住次数到上限，停驶')
        elif self.stage == 'align':
            err = wrap(th - yaw)
            if abs(err) <= ALIGN_TOL:
                self.enter_hold()
                note = '朝向到位，转入 hold'
            else:
                # 同样只挂反应式脱困，不做预测式否决：核验位 (6.80,-9.95) 和入口排队位
                # (12.00,-12.21) 按整圆判据本来就在柱子的扫掠阴影里（余量 -0.67/-0.63 m），
                # 而这两处只需要 10° 级的小修正 —— 预测式守卫会让车在等杆时不停挪窝，
                # 抓拍位置就没了。小角度修正扫的是扇形不是整圆，整圆判据只配用来选方向。
                if self.escape_now and self.escape_tick(x, y, yaw):
                    return
                why = self.spin_guard(err)
                if why:
                    return self.park(why)
                cmd.angular.z = self.turn_cmd(err)
                note = '朝向差 %.2f rad 未到位，本航点已累计转向 %.1f rad' % (
                    err, self.spin_rad)
                if self.watch_spin(x, y, yaw, cmd.angular.z):
                    return self.publish(Twist(), '原地转卡住次数到上限，停驶')
        elif self.stage == 'hold':
            if self.hold_gate:
                up, why = self.boom_up(self.hold_gate)
                waited = self.sim_now - self.hold_started
                if not up:
                    if waited > GATE_WAIT_S:
                        return self.begin_back(
                            '%s 在 (%.2f,%.2f) 等 %s 抬杆 %.0f s 没等到（%s）' % (
                                self.model, tx, ty, self.hold_gate, waited, why),
                            '等杆超时，准备退回')
                    return self.publish(Twist(), '等 %s 抬杆（%s）已 %.1f s' % (
                        self.hold_gate, why, waited))
            if self.sim_now < self.hold_until:
                return self.publish(Twist(), '停留还剩 %.1f s' % (
                    self.hold_until - self.sim_now))
            return self.next_point()
        else:  # back：倒车退回上一个航点（同一条车道，位置对了就停）
            bx, by = self.back_to
            # 倒车段同样按轮轴中点判到位：倒车时车头会转正到"退回目标那一格"的朝向，
            # 那一次转向也是在轮轴上原地完成的，用底盘原点判就会倒过头再甩回来。
            bfx, bfy = self.pivot_want(bx, by, self.seq[self.back_index][4])
            px, py = self.pivot_now(x, y, yaw)
            d = math.hypot(bfx - px, bfy - py)
            if d <= self.radius:
                row = self.seq[self.back_index]
                self.get_logger().info(
                    '[shuttle] 已退回 %s#%d (%.2f,%.2f)，从这一格接着往下跑' % (
                        row[0], row[1], row[2], row[3]))
                self.idx = self.back_index
                self.stage = 'align'
                self.ref = None
                self.stuck_tries = 0
                self.back_gate = None
                self.reset_leg_stats()
                return self.publish(Twist(), '退回完成')
            # 倒车一样会越杆：杆放平时倒回去就是从放平的杆上碾过去，比往前顶更糟，
            # 所以这一支和往前开一样要先确认它抬着。
            g = self.crosses_boom((x, y), (bx, by))
            if g:
                up, why = self.boom_up(g)
                if not up:
                    if self.back_gate != g:
                        self.back_gate, self.back_gate_at = g, self.sim_now
                    waited = self.sim_now - self.back_gate_at
                    if waited > GATE_WAIT_S:
                        return self.park('倒车段要越 %s，它没抬（%s）已等 %.0f s' % (
                            g, why, waited))
                    return self.publish(Twist(), '倒车要越 %s，等它抬杆 %.1f s' % (
                        g, waited))
                self.back_gate = None
            cmd.linear.x = -BACK_SPEED
            # 倒车时"车尾对准目标"等价于"车头反着对准"，所以期望航向是 atan2 加 π。
            back_err = wrap(math.atan2(bfy - py, bfx - px) + math.pi - yaw)
            # 倒车段也守"不带着饱和角速度行车"这条：这里的 ω 上限是 turn*0.5，
            # 配 BACK_SPEED 0.15 的最小转弯半径 0.43 m，比前进段还容易绕圈。
            # 前进段那套圆弧可行性判据在这里一样成立：BACK_SPEED 是定值，所以
            # "这一拍到底转得过来"就只看 ω_cap·d/(2|sin e|) 还够不够 BACK_SPEED。
            w_cap = self.turn * 0.5
            d_ok = w_cap * d / max(2.0 * abs(math.sin(back_err)), 1e-3)
            if abs(back_err) > w_cap / 1.5 or d_ok < BACK_SPEED:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_cmd(back_err, w_cap)
                note = '倒车朝向差 %.2f rad，先原地转' % back_err
                if self.watch_spin(x, y, yaw, cmd.angular.z):
                    return self.publish(Twist(), '倒车段原地转卡住次数到上限，停驶')
            else:
                cmd.angular.z = clip(
                    2.0 * BACK_SPEED * math.sin(back_err) / max(d, 1e-3), w_cap)
                note = '倒车退回 %.2f m' % d

        return self.publish(cmd, note, x, y, yaw, dist)

    def enter_hold(self):
        self.stage = 'hold'
        self.hold_started = self.sim_now
        self.ref = None
        # 只有"下一段要越过杆轴"的航点才需要等抬杆；其余航点 dwell 为 0 直接过。
        nxt = self.seq[self.idx + 1] if self.idx + 1 < len(self.seq) else None
        self.hold_gate = self.crosses_boom(
            (self.seq[self.idx][2], self.seq[self.idx][3]),
            (nxt[2], nxt[3])) if nxt else None
        self.hold_until = self.sim_now + (
            self.dwell_s if self.seq[self.idx][:2] in self.dwell_at else 0.0)

    def next_point(self):
        """推进到下一个航点；走到序列末尾就是走完一个周期，先在等待位凑满
        loop_period_s，再从头起下一轮 —— 周期由时钟保证而不是"开完就算一轮"，
        取证脚本才能按时间轴预告第 N 次通行大概落在哪一段。"""
        # hold 阶段等的是这一段要越的杆；起步后"该看哪根杆"由 approach 每 tick
        # 按当前位置→目标的实际线段重算，这里不带旧值过去（带旧值反而会漏掉
        # 车不在路线上时的那一段）。
        self.hold_gate = None
        last = self.idx == len(self.seq) - 1
        if last:
            if self.cycle_start is None:
                # 进程重启后 seat_at 会把车接在路线中段（151 轮实测落在 reset#1），
                # 这条路径上 idx==0 那一支永远走不到，cycle_start 就永远是 None。
                # 旧写法 start = cycle_start if not None else sim_now 等于每个周期末尾
                # 都把到期时刻重设成"现在 + loop_period_s"：倒计时看着在走（实测
                # 118.0→107.5→…→25.9），走完 120 s 又凭空多出 120 s，第 2 轮永不开始。
                # 现场表现是车钉在等待位、位姿 17 s 只漂 0.01 m、日志一行不增，
                # 读起来和"节点挂了"一模一样。没起过轮就没有周期可凑：把周期起点
                # 钉在这一刻，直接开跑下一轮。
                self.cycle_start = self.sim_now
            else:
                due = self.cycle_start + self.period_s
                if self.sim_now < due:
                    self.stage = 'hold'
                    self.hold_until = due
                    return self.publish(Twist(), '凑周期，还剩 %.1f s' % (
                        due - self.sim_now))
            self.cycle += 1
            self.abort_streak = 0       # 跑通一整轮，之前连续作废的账翻篇
            self.get_logger().info('[shuttle] 第 %d 轮跑完（累计作废 %d 轮）' % (
                self.cycle, self.aborted))
        elif self.idx == 0:
            self.cycle_start = self.sim_now
        self.pending_boom = None
        self.boom_stall = None
        self.boom_stall_logged = False
        self.idx = (self.idx + 1) % len(self.seq)
        # 转向点在路线里写成"同一个 (x,y) 的配对点"（entry#2/#3、reset#1/#2…）。
        # 旧实现在这里直接跳进 align、把 approach 整段跳过，理由是"原地转会车心会甩开
        # 一个 0.4 m 半径的圆，而 arrive_radius 只有 0.12 m，再判一次到位就绕圈"
        # （#80/#89 实测 exit#0 绕圈 12 min）。那个绕圈的真正原因是到位判定用的是底盘
        # 原点：底盘被转向甩开，目标却不动，于是永远贴不上。现在到位判定按轮轴中点，
        # 轮轴目标 = 航点 + axle·û(th) 与当前朝向无关，转向甩不动它，所以必须回到
        # approach 老实把轮轴开过去：轮轴一到位、车头一转正，底盘正好落在航点上。
        # 留着这个跳步的后果是实测的：reset#1 到点后原地转 90°，底盘落在 (12.40,-13.20)
        # 而不是等待位 (12.00,-12.80)，横向偏 0.57 m —— 抓拍余量没这么多。
        self.stage = 'approach'
        self.ref = None
        self.reset_leg_stats()
        return self.publish(Twist(), '进入下一航点')

    def begin_back(self, why, note):
        """作废本轮：倒车退回上一个航点。两条车道的等待位都在核验位正后方（同 x），
        所以直接倒车就回得去，不需要先掉头 —— 掉头反而会把车甩进道闸机箱那一侧的碰撞件。
        连续作废到上限就不再退了：退回→再顶一次→再退，日志里读起来和死机没区别。"""
        self.aborted += 1
        self.abort_streak += 1
        self.stuck_tries = 0
        self.hold_gate = None
        self.pending_boom = None
        self.boom_stall = None
        self.boom_stall_logged = False
        self.back_gate = None
        if self.abort_streak >= MAX_ABORT_STREAK:
            self.get_logger().warn('[shuttle] %s → 本轮作废（累计 %d 轮）' % (
                why, self.aborted))
            return self.park('%s，且已连续作废 %d 轮' % (why, self.abort_streak))
        prev = self.seq[self.idx - 1] if self.idx > 0 else self.seq[self.idx]
        self.get_logger().warn(
            '[shuttle] %s → 倒车退回 %s#%d (%.2f,%.2f)，本轮作废（累计 %d 轮）' % (
                why, prev[0], prev[1], prev[2], prev[3], self.aborted))
        self.back_to = (prev[2], prev[3])
        self.back_index = max(self.idx - 1, 0)
        self.stage = 'back'
        self.reset_leg_stats()
        return self.publish(Twist(), note)

    def park(self, reason):
        """停驶挂起：之后每个 tick 只发零速。轮位目标只有靠零速指令才钉得住
        （静默不刹，#120），所以这里不能只是"不再发布"。"""
        self.parked = True
        self.park_reason = reason
        self.ref = None
        self.get_logger().error(
            '[shuttle] 停驶挂起：%s。已停止下发任何行驶指令、只连发零速钉住轮位目标；'
            '请现场确认车是不是楔在设施上，处理完重启本节点（它会按当前位置重新选路）。'
            % reason)
        return self.publish(Twist(), '停驶挂起：%s' % reason)

    def watch_stuck(self, x, y, v):
        """给了前进指令却不动 = 顶到东西了。继续灌油只会把轮位误差攒成弹射（#77），
        所以停 STUCK_REST_S 让差速器的目标轮角追平实际轮角，再重新起步。
        返回 True 表示已经到上限并转入停驶挂起。"""
        if v <= 0.01:
            return False
        if self.ref is None or math.hypot(x - self.ref[0], y - self.ref[1]) > 0.02:
            self.ref = (x, y)
            self.ref_at = self.sim_now
            return False
        if self.sim_now - self.ref_at > STUCK_S:
            self.stuck_tries += 1
            if self.stuck_tries > MAX_STUCK_TRIES:
                self.park('连续 %d 次给 %.2f m/s 都走不动（顶到闸杆或被楔住）' % (
                    self.stuck_tries, v))
                return True
            self.get_logger().warn(
                '[shuttle] 给 %.2f m/s 却 %.1f s 没挪动（顶到闸杆或被楔住），'
                '松速 %.1f s 再试（第 %d/%d 次）' % (
                    v, STUCK_S, STUCK_REST_S, self.stuck_tries, MAX_STUCK_TRIES))
            self.rest_until = self.sim_now + STUCK_REST_S
            self.ref = None
        return False

    def watch_spin(self, x, y, yaw, w):
        """给了转向指令却一点不转 = 轮子被顶住了。

        为什么必须单独有这一条：watch_stuck 的入口判据是 `v <= 0.01` 直接放过，它只
        管"给油不走"。而 approach 的"方向差太大→原地转"支和 align 相都是 v=0、只有
        ω，正好整个落在 watch_stuck 的盲区里。152 轮实测就是这个形态：car_6 停在
        (10.228,-12.566)，12 s 里收到 146 帧 ang=+0.350 的有效指令，yaw 变化 0.00000
        rad、位移 0.0000 m，而同窗口对照组 car_2/car_4 各走了 2.73/3.63 m（物理和
        位姿流都健康）。结果 shuttle 一行新日志都不出，读起来和"节点挂了"一样。

        判据用 yaw 的实际进展而不是轮速：/car_6/odom 这一轮同样读不到，只有
        /model_states 是真值源。阈值 0.02 rad 远小于 k_ang·误差 起步就会产生的量
        （0.35 rad/s × 3 s = 1.05 rad），不会误伤正常转向。
        """
        if abs(w) <= 0.01:
            return False
        if (self.aref is None or self.aref_at is None
                or abs(wrap(yaw - self.aref)) > 0.02):
            self.aref = yaw
            self.aref_at = self.sim_now
            return False
        if self.sim_now - self.aref_at > STUCK_S:
            self.stuck_tries += 1
            if self.stuck_tries > MAX_STUCK_TRIES:
                self.park('连续 %d 次给 ω=%.2f rad/s 都转不动（轮子被顶住或驱动实例卡死）'
                          % (self.stuck_tries, w))
                return True
            if not self.spin_clear(x, y, yaw):
                # 这个位置的整圆扫掠确实压到设施 —— 别再原地重复顶，装填脱困：
                # 下一拍由 escape_tick 直着挪到转得开的地方。
                self.escape_now = True
                self.get_logger().warn(
                    '[shuttle] 给 ω=%.2f rad/s 却 %.1f s 没转过，且此处扫掠圆（半径 %.2f m）'
                    '压到已登记设施 —— 松速 %.1f s 后直着挪找旋转净空（第 %d/%d 次）' % (
                        w, STUCK_S, self.sweep_r, STUCK_REST_S,
                        self.stuck_tries, MAX_STUCK_TRIES))
            else:
                # 圆内干净却转不动：要么障碍没登记进 keepout，要么 diff_drive 实例卡死。
                # 这两种都不是"挪一挪"能解的，沿用松速重试，次数到上限停死点名要人工。
                # 但这句里的"净空"必须先自证查过东西：keepout 表为空时 spin_clear 恒真，
                # 报"扫掠圆内无已登记设施"就是句假话（读的人以为核过净空，其实一项都没
                # 核），所以把表长打进消息里，空表直接点名"这条判据没查任何东西"。
                self.get_logger().warn(
                    '[shuttle] 给 ω=%.2f rad/s 却 %.1f s 没转过（%s），'
                    '松速 %.1f s 再试（第 %d/%d 次）' % (
                        w, STUCK_S,
                        'keepout 表为空，净空这条判据没查任何东西，只能疑驱动实例卡死'
                        if not self.keepout else
                        '扫掠圆内无 keepout 里的 %d 项设施，疑漏登记障碍或驱动实例卡死'
                        % len(self.keepout),
                        STUCK_REST_S, self.stuck_tries, MAX_STUCK_TRIES))
            self.rest_until = self.sim_now + STUCK_REST_S
            self.aref = None
            self.aref_at = None
        return False

    def spin_clear(self, x, y, yaw):
        """车此刻这个位置，绕轮轴原地转一圈会不会扫到 keepout 里的设施。

        扫掠圆的圆心是**轮轴**而不是底盘中心（原地转的瞬时中心在驱动轮轴上，
        axle_ahead=0.40），半径 hypot(半长+0.40, 半宽) —— 与 parking_zones.yaml 里
        手算的那两个 1.265 m 是同一个口径，只是这里改成实算、每个 tick 都核。
        """
        if not self.keepout:
            return True
        ax = x + self.axle * math.cos(yaw)
        ay = y + self.axle * math.sin(yaw)
        r = self.sweep_r + SPIN_MARGIN
        for kx, ky, khx, khy in self.keepout:
            dx = max(abs(ax - kx) - khx, 0.0)
            dy = max(abs(ay - ky) - khy, 0.0)
            if math.hypot(dx, dy) < r:
                return False
        return True

    def escape_ok(self, x, y, yaw, d):
        """朝 d（+1 前 / -1 后）直着挪满 ESCAPE_MAX 能不能同时满足两件事：
        落点转得开、且这条直线不越闸杆。

        后一条不能省：脱困往往是倒退，而摆渡车最常见的"转不开"现场就是在闸厅里
        贴着杆口 —— 152 轮的 car_6 就停在 (10.204,-12.619)，入口杆轴在它北边。放平
        的杆是硬障碍物，倒过去等于从杆底下穿回车道另一侧，那比停着更糟（车会顶杆，
        顶出来的轮位误差就是 #77 的弹射）。这里不按杆的升降态过滤：脱困位移长达
        ESCAPE_MAX，中途 manager 把杆放下来的概率不低，一律不越杆才稳。
        """
        tx = x + d * ESCAPE_MAX * math.cos(yaw)
        ty = y + d * ESCAPE_MAX * math.sin(yaw)
        if self.crosses_boom((x, y), (tx, ty)):
            return False
        return self.spin_clear(tx, ty, yaw)

    def escape_tick(self, x, y, yaw):
        """直着挪到"绕轮轴转得开"的位置，只在 watch_spin 实测到转不动之后才跑。

        返回 True 表示这一拍已经由脱困逻辑发过指令（或已转入退回/挂起），调用方直接
        return；返回 False 表示已经转到空位，调用方接着走正常原地转。
        （publish 本身返回 None，所以不能靠返回值区分，必须显式给 bool。）

        两道封顶都要有，缺一不可：
        * 位移封顶 ESCAPE_MAX —— 挪够了还转不开，说明 keepout 表漏了东西；
        * 进展封顶 ESCAPE_PROGRESS/STUCK_S —— 154 轮实测车楔在亭子角上（OBB 间隙
          -0.000 m）、轮子打滑，0.08 m/s 的指令 5.5 min 只挪 0.014 m，只按位移封顶
          要磨 40 min 才挂起。真楔死要用"发油门挪不动"这个口径快速定性并停死。
        """
        if self.spin_clear(x, y, yaw):
            # 松速那 3 s 里接触自己弹开了、或位姿流把车读回了空位：脱困结束。
            self.escape_now = False
            self.escaping = 0
            self.escape_from = None
            self.escape_ref = None
            return False
        if self.escaping == 0:
            fwd = self.escape_ok(x, y, yaw, 1)
            back = self.escape_ok(x, y, yaw, -1)
            if not fwd and not back:
                self.escape_now = False
                self.begin_back(
                    '车在 (%.2f,%.2f) 前后 %.2f m 内要么转不开（扫掠半径 %.2f m）、'
                    '要么要越过闸杆：两个脱困方向都被否' % (
                        x, y, ESCAPE_MAX, self.sweep_r),
                    '旋转净空不足，准备退回')
                return True
            self.escaping = 1 if fwd else -1
            self.escape_from = (x, y)
            self.escape_ref = (x, y)
            self.escape_ref_at = self.sim_now
        d = math.hypot(x - self.escape_from[0], y - self.escape_from[1])
        if d > ESCAPE_MAX:
            self.escape_now = False
            self.escaping = 0
            self.escape_from = None
            self.escape_ref = None
            self.park('直行脱困走满 %.2f m 仍然转不开（顶在 (%.2f,%.2f)）' % (
                ESCAPE_MAX, x, y))
            return True
        if (self.escape_ref is None or self.escape_ref_at is None
                or math.hypot(x - self.escape_ref[0], y - self.escape_ref[1])
                > ESCAPE_PROGRESS):
            self.escape_ref = (x, y)
            self.escape_ref_at = self.sim_now
        elif self.sim_now - self.escape_ref_at > STUCK_S:
            self.stuck_tries += 1
            self.escaping = 0            # 下一拍重选方向，别绑死在同一个滑不动的朝向上
            self.escape_ref = None
            if self.stuck_tries > MAX_STUCK_TRIES:
                self.escape_now = False
                self.park('脱困指令（%.2f m/s）连续 %d 次各 %.1f s 都挪不动，离脱困起点才 %.2f m '
                          '—— 轮子在打滑、车身被楔死，自行脱困无望，需要人工摆位或重 spawn'
                          % (ESCAPE_V, self.stuck_tries, STUCK_S, d))
                return True
            self.get_logger().warn(
                '[shuttle] 脱困给了 %.2f m/s 却 %.1f s 没挪动（轮子打滑/车身楔住），'
                '松速 %.1f s 后重选方向再试（第 %d/%d 次）' % (
                    ESCAPE_V, STUCK_S, STUCK_REST_S, self.stuck_tries, MAX_STUCK_TRIES))
            self.rest_until = self.sim_now + STUCK_REST_S
            return True
        cmd = Twist()
        cmd.linear.x = self.escaping * ESCAPE_V
        self.publish(cmd, '转不开，先%s %.2f m 找旋转净空' % (
            '前行' if self.escaping > 0 else '倒退', d))
        return True

    def brake_out(self):
        """退出前连发 2 s 零速。diff_drive 的 cmd_vel_timeout 实测不生效（#120），
        节点一走插件会保持着最后一条指令继续跑 —— 车要么顶到杆上把轮位误差攒成弹射，
        要么一路开出沙盘翻下去（#85 就是这么来的）。"""
        for _ in range(40):
            self.pub_cmd.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def stop(self, note):
        self.publish(Twist(), note)

    # ── 输出 ──────────────────────────────────────────────────────────────
    def shape(self, cmd):
        """把这一拍想要的速度换成物理上追得上的速度。

        两步：
        1) 死区下限 —— 想要一个非零却低于 MIN_* 的量，等于"发指令但车不动"，插件
           照样把轮位目标往前推，纯攒欠账；要么给到能动起来的数，要么就是零。
           已经在零上的指令不受影响（hold/停驶的零速永远立刻生效，刹车不排队）。
        2) 斜坡 —— 每拍最多动 RATE_*·dt。阶跃指令超过 max_wheel_acceleration 时
           ODE 直接把轮子推进打滑：轮速积在插件里、车身没走，攒够就是一次弹射。
        状态行打印的是**整形之后**的数，取证读到的就是真正发出去的那一帧。
        """
        out = Twist()
        out.linear.x = self._ramp(cmd.linear.x, RATE_V, MIN_V, 'cmd_v')
        out.angular.z = self._ramp(cmd.angular.z, RATE_W, MIN_W, 'cmd_w')
        # 原地转预算：v 为零而 ω 非零的那些拍累加，换航点时清零（见 next_point）。
        if abs(out.linear.x) < 1e-9 and abs(out.angular.z) > 0.0:
            self.spin_rad += abs(out.angular.z) * self.dt
        return out

    def _ramp(self, want, rate, mn, attr):
        cur = getattr(self, attr)
        if 0.0 < abs(want) < mn:
            want = math.copysign(mn, want)
        got = 0.0 if want == 0.0 else cur + clip(want - cur, rate * self.dt)
        setattr(self, attr, got)
        return got

    def publish(self, cmd, note, x=None, y=None, yaw=None, dist=None):
        cmd = self.shape(cmd)
        self.pub_cmd.publish(cmd)
        row = self.seq[self.idx]
        # 行首点名模型：任务五改成 car_2 / car_3 两辆摆渡车后，两个实例都往
        # /parking/shuttle_status 发，看板和取证脚本必须能从这一行读出是哪家车，
        # 否则画面上的"进度"会在两辆车之间来回跳。
        st = '%s phase=%s %d/%d %s#%d 目标=(%.2f,%.2f) 车头=%+.2f' % (
            self.model, self.stage, self.idx + 1, len(self.seq), row[0], row[1],
            row[2], row[3], row[4])
        if x is not None:
            st += ' 位置=(%.2f,%.2f) 距离=%.2f 航向=%+.2f' % (x, y, dist, yaw)
        st += ' 指令 v=%.2f w=%+.2f｜第 %d 轮%s｜%s' % (
            cmd.linear.x, cmd.angular.z, self.cycle + 1,
            '（作废 %d 轮）' % self.aborted if self.aborted else '', note)
        self.pub_status.publish(String(data=st))


def main():
    rclpy.init()
    node = ShuttleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.brake_out()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
