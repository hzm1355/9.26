#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按灯行驶节点 —— 让虚拟车辆真正遵守"红灯停、绿灯行"（协议 §三.(二)）。

  1) 车辆世界位姿取自 /model_states，不依赖各车 odom 的坐标系原点
  2) 订阅 traffic_light/<group>/state 获取各路口灯态
  3) 停车线带"来车方向 heading"：只有车头朝向与该方向一致时才受这条线约束，
     因此同一条停车线只拦迎面来车，不拦已经通过/正在驶离的车
  4) 红灯必停；黄灯在仍能安全刹住（距线 > yellow_clear_dist）时也停；绿灯放行
  5) 跟车防追尾：同车道前车慢于本车时按间距限速，形成真实排队
  6) 车体重叠/对顶或车辆被甩出场景时自动复位，避免接触求解发散把车弹飞
  7) 输出 /<car>/cmd_vel，并把每 tick 的位姿与"被谁拦住"写入 CSV 供取证

配置：config/signal_drive_params.yaml
"""
import csv
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def obstacle_offset(obstacles, clr, lookahead, lane_half,
                    ppx, ppy, ux, uy, along, lat):
    """把前方静态碰撞盒投到当前航段坐标系，返回 (期望横向偏移, 挤不下, 车已压进禁行带)。

    横向以车道中心线为 0。每个盒在横向上占 [l_lo, l_hi]，车侧边要留 clr，于是它变成
    禁行带 [l_lo - clr, l_hi + clr]。期望偏移从车道中心 0 出发反复外让：落在带里就移到
    离当前目标更近的带边，直到没有带包住目标。单趟扫掠不收敛（两根夹着路的灯杆会把解
    在两边之间弹来弹去），所以迭代到稳定，最多 8 趟；仍收敛不了就按最后结果走并标记 blocked。
    中心线本身安全时期望偏移就是 0；车漂进禁行带则靠 in_band 收紧前视点＋限速来救。
    """
    bands = []
    for x0, x1, y0, y1, _nm in obstacles:
        a_list, l_list = [], []
        for cx in (x0, x1):
            for cy in (y0, y1):
                rx, ry = cx - ppx, cy - ppy
                a_list.append(rx * ux + ry * uy)
                l_list.append(-rx * uy + ry * ux)
        a_lo, a_hi = min(a_list), max(a_list)
        if a_hi < along - 1.0 or a_lo > along + lookahead:
            continue                       # 不在这段前方视窗里
        l_lo, l_hi = min(l_list), max(l_list)
        if l_hi < -lane_half - 1.0 or l_lo > lane_half + 1.0:
            continue                       # 横向本来就远在走廊外
        bands.append((a_lo, l_lo - clr, l_hi + clr))
    if not bands:
        return 0.0, False, False
    bands.sort()
    in_band = any(lo < lat < hi for _a, lo, hi in bands)
    target = 0.0
    for _pass in range(8):
        moved = False
        for _a, lo, hi in bands:
            if lo < target < hi:
                target = lo if (target - lo) <= (hi - target) else hi
                moved = True
        if not moved:
            break
    blocked = abs(target) > lane_half or any(lo < target < hi for _a, lo, hi in bands)
    return clamp(target, -lane_half, lane_half), blocked, in_band


class CarAgent:
    """单车控制器状态：路线巡航 + 信号灯/跟车限速"""

    def __init__(self, name, cfg, node):
        self.name = name
        route = [tuple(p) for p in cfg.get('route', [])]
        if len(route) < 2:
            raise ValueError(f'{name}: route 至少两个航点')
        self.route = route
        self.cyclic = str(cfg.get('mode', 'loop')) != 'shuttle'
        self.step = 1
        self.idx = int(cfg.get('start_idx', 0)) % len(route)
        if not self.cyclic and self.step < 0:
            self.idx = len(route) - 1
        self.obey = cfg.get('obey') or []
        self.cruise = float(cfg.get('cruise_speed', 0.7))
        # 同环判据 + 弧长定位：leg j = route[j-1] -> route[j]，off[j] 为其起点弧长
        self.ring = tuple(route)
        self.off = [0.0]
        for j in range(1, len(route) + 1):
            p, t = route[(j - 1) % len(route)], route[j % len(route)]
            self.off.append(self.off[-1] + math.hypot(t[0] - p[0], t[1] - p[1]))
        self.perim = self.off[-1] or 1.0
        self.s = 0.0                # 沿路线的弧长位置（m）
        self.x = self.y = self.yaw = self.speed = 0.0
        self.vmag = 0.0               # 世界系合速度（model_states 的 twist 是世界速度）
        # 实测角速度：被别的车顶住打转时，绕接触点的 ω 会远超指令上限（审计抓到 1.5 rad/s
        # 而 max_angular 只有 0.45）。只看线速度看不到这件事，车会在打转中横滑出车道。
        self.wmag = 0.0
        self.yaw_prev = None
        self.t_prev = 0.0
        self.spin_ticks = 0           # 实测超速转的持续 tick 数
        self.off_lane = False         # 当前是否已离开车道走廊
        self.off_lane_ticks = 0
        self.reacquire_try = 0        # 连续"按位姿重取航段"的次数
        self.lat_target = 0.0         # 绕开静态碰撞盒所需的车道横向偏移（平滑后）
        self.seen = False
        self.arrive_radius = float(cfg.get('arrive_radius', 1.2))
        self.lane_half = float(cfg.get('lane_half', 2.4))   # 入线判定的横向容差
        # 出界判据按**这台车自己的路线包络**算，不再用全体共用、且和路线脱节的写死方框：
        # 原 oob_x 22.0 对着西环中心线 x=-21.8 只剩 0.2 m，正常压线就判出界 → 复位 →
        # 摆位弹射 4~8 m/s → 真被弹出台面，风暴自持（不带覆盖实测 122 s 内 68 次复位，
        # 其中 42 次是"出界"）。余量 = lane_half + 1.2（走廊半宽 + 一个换段余量），
        # 并夹到台面 25 m 内侧，保留"被弹出台面必须复位"这一条。
        self.env_x = min(24.4, max(abs(p[0]) for p in route) + self.lane_half + 1.2)
        self.env_y = min(24.4, max(abs(p[1]) for p in route) + self.lane_half + 1.2)
        self.wp_timeout = float(cfg.get('wp_timeout', 45.0))
        self.last_wp_t = node.get_clock().now().nanoseconds / 1e9
        self.t_seeded = False         # 首个 tick 起算航点超时
        self.contact = False          # 车体与其他车重叠
        self.conflict = False         # 对向来车冲突
        self.stuck_ticks = 0          # 车体重叠持续tick数
        self.held_ticks = 0           # 被其他车拦停且不动的持续tick数
        self.conflict_ticks = 0       # 对向顶住且不动的持续tick数
        self.wedge_ticks = 0          # 给油不走（顶到障碍物）持续tick数
        self.escape_left = 0          # 倒车脱困剩余tick
        self.escape_try = 0           # 已尝试脱困次数
        self.cmd_prev = 0.0           # 上一周期下发的线速度
        self.stopped = False          # 当前被信号灯要求停住
        self.last_reset = 0.0
        # 复位不再是一步摆位：1=原地零速刹轮，2=落位后零速静默，0=正常行驶。
        self.reset_phase = 0
        self.reset_left = 0
        self.reset_reason = ''
        self.pub = node.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        self.status = ''
        self.stopped_ticks = 0

    def update_pose(self, x, y, yaw, speed, vmag=None):
        self.x, self.y, self.yaw, self.speed = x, y, yaw, speed
        self.vmag = abs(speed) if vmag is None else vmag
        self.seen = True

    def update_arc(self):
        """刷新沿路线的弧长位置，供同环两车判断谁前谁后"""
        n = len(self.route)
        j = self.idx % n
        px, py = self.route[j - 1]
        tx, ty = self.route[j]
        dx, dy = tx - px, ty - py
        L = math.hypot(dx, dy) or 1.0
        a = ((self.x - px) * dx + (self.y - py) * dy) / L
        self.s = self.off[j] + clamp(a, 0.0, L)

    def _next_waypoint(self, node, now, timed_out):
        if not timed_out:
            if self.cyclic:
                self.idx = (self.idx + 1) % len(self.route)
            else:
                self.idx += self.step
                if self.idx >= len(self.route):
                    self.step = -1
                    self.idx = len(self.route) - 2
                elif self.idx < 0:
                    self.step = 1
                    self.idx = 1
            self.last_wp_t = now
            return
        # 超时绝不能跳到"下一个"航点：它可能在车后方，会让车原地掉头撞上后车。
        # 只选仍然在前方、且离当前目标最近角度的航点。
        node.get_logger().warn(f'{self.name}: 航点 {self.idx} 超时，重选前方航点')
        n = len(self.route)
        best, best_err = None, 1.4
        for k in range(1, n):
            j = (self.idx + k) % n
            tx, ty = self.route[j]
            dx, dy = tx - self.x, ty - self.y
            if math.hypot(dx, dy) < self.arrive_radius:
                continue
            err = abs(wrap(math.atan2(dy, dx) - self.yaw))
            if err < best_err:
                best, best_err = j, err
        if best is not None:
            self.idx = best
        self.last_wp_t = now

    def snap_to_route(self, node):
        """首个 tick 按实际位姿入线。回放/中断恢复后车可能停在路网任意位置，
        仍按 start_idx 行驶会让它斜穿街区撞楼，这里改成"接着当前航段往下开"。"""
        n = len(self.route)
        best = None
        for j in range(n):
            p = self.route[(j - 1) % n]
            t = self.route[j]
            dx, dy = t[0] - p[0], t[1] - p[1]
            L = math.hypot(dx, dy) or 1.0
            ux, uy = dx / L, dy / L
            rx, ry = self.x - p[0], self.y - p[1]
            along = rx * ux + ry * uy
            lat = abs(-rx * uy + ry * ux)
            if not 0.0 <= along <= L or lat > self.lane_half:
                continue
            if abs(wrap(math.atan2(uy, ux) - self.yaw)) > 1.0:
                continue          # 车头朝向与该航段相反，不算入线
            score = lat + 0.02 * (L - along)
            if best is None or score < best[0]:
                best = (score, j, along, L)
        if best is not None:
            _, j, along, L = best
            self.idx = j
            node.get_logger().info(
                f'{self.name}: 按位姿入线 -> 航段→{j} 剩余 {L - along:.1f} m')


class SignalDriveNode(Node):
    def __init__(self):
        super().__init__('signal_drive_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('turn_p_gain', 1.1)
        self.declare_parameter('max_angular', 0.45)
        self.declare_parameter('turn_lead', 1.2)   # 进角前多少米换段（配合进角减速，太大会斜切内弯）
        # 穿障修复两护栏，判据来自 tools/analyze_deviation.py 的 3 小时轨迹审计：
        # 最大横偏 car_1 +5.37 m / car_5 +5.04 m，且审计抓到实测 |ω|≈1.5 rad/s（指令上限只有 0.45）。
        # 换段那一瞬，车对**新**航段的合法横偏本来就等于 turn_lead(1.2) 加上转弯半径余量，
        # 所以 1.5 会把每一次正常过 90° 角都数进"离道"，4 s 后重取航段、三次就复位。
        # 2.6 = lead 1.2 + 进角限速下的半径余量 1.4；真正的索引跑偏实测 3.8~15.8 m 照样拦得住。
        self.declare_parameter('lane_guard', 2.6)       # |横偏| 超过此值判为离开车道走廊
        self.declare_parameter('off_lane_speed', 0.28)  # 离道时的限速，逼车先回正再提速
        self.declare_parameter('off_lane_reacquire_seconds', 4.0)
        self.declare_parameter('reacquire_max', 3)
        # 阈值取 1.0 而不是指令上限 0.45：probe_fleet.py 实测正常跑（含进弯）|ω| 就到 0.87，
        # 卡在 0.9 会把正常进弯误判成打转。
        self.declare_parameter('spin_rate', 1.0)       # 实测 |ω| 上限，超过＝绕着别的东西打转
        self.declare_parameter('spin_seconds', 0.4)
        # 绕开实体碰撞盒：审计里 tl_j3_ns_ped 的盒与车体外接盒相交 185 格，灯杆离车道
        # 中心线只有 1.40~2.00 m，车宽 0.8 m，沿中心线走必然刮杆；刮杆把车楔住打转，
        # 打转再把车甩出车道去穿路边的无碰撞设施。所以几何避让才是这一串的源头。
        self.declare_parameter('obstacle_file', '')
        self.declare_parameter('obstacle_clear', 0.65)      # 车侧边到盒面的留白
        self.declare_parameter('obstacle_lookahead', 6.0)   # 只看前方这么远内的盒
        self.declare_parameter('detour_speed', 0.45)
        self.declare_parameter('stop_gain', 0.5)
        self.declare_parameter('yellow_clear_dist', 4.0)
        self.declare_parameter('follow_gap', 2.6)
        self.declare_parameter('contact_dist', 1.7)
        self.declare_parameter('conflict_half', 1.1)
        self.declare_parameter('oob_x', 22.0)
        self.declare_parameter('oob_y', 19.0)
        # 复位高度：实测静止值，不是 world 的 spawn 高度（见 reset_car 里的说明）
        self.declare_parameter('reset_z', 6.70)
        # 摆位弹射的隔离实验（tools/place_eject_probe.py → _eject77.txt）：
        # 同一台车、同一个落点，摆位前全程零速 ⇒ 摆位后 6 s 实测 mv 恒 0.00（不弹）；
        # 摆位时持续给油 0.70 ⇒ 立刻 0.27→0.55→0.83→1.11→1.38（弹）。
        # 现场更狠：车楔住"给油不走"时差速控制器的轮位误差能攒十几秒，摆位一次性放掉，
        # 实测沿车头方向以 ≈2.8 m/s² 冲到 4~8.5 m/s（指令上限只有 0.70，期间还在发 -0.10 刹车），
        # 直冲出路网边界 —— 这就是用户看到的"车穿过建筑物"。
        # 所以复位必须"先刹轮放掉误差 → 再落位 → 落位后再零速压一小段"。
        self.declare_parameter('reset_brake_seconds', 1.5)
        self.declare_parameter('reset_quiet_seconds', 1.0)
        self.declare_parameter('stuck_seconds', 4.0)
        self.declare_parameter('conflict_seconds', 3.0)
        self.declare_parameter('wedge_seconds', 2.0)
        self.declare_parameter('held_seconds', 15.0)
        self.declare_parameter('escape_seconds', 2.0)
        self.declare_parameter('escape_max', 2)
        self.declare_parameter('path_csv', '/tmp/signal_drive_path.csv')
        self.declare_parameter('enable', True)

        path = self.get_parameter('config_file').value
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'signal_drive_params.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        root = cfg.get('signal_drive', cfg)
        if 'ros__parameters' in root:
            root = dict(root['ros__parameters'], **{k: v for k, v in cfg.items() if k != 'signal_drive'})
        self.cfg = root

        self.rate = float(self.get_parameter('publish_rate').value)
        self.k_turn = float(self.get_parameter('turn_p_gain').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        self.turn_lead = float(self.get_parameter('turn_lead').value)
        self.lane_guard = float(self.get_parameter('lane_guard').value)
        self.off_lane_speed = float(self.get_parameter('off_lane_speed').value)
        self.off_lane_reacquire_ticks = int(
            self.rate * float(self.get_parameter('off_lane_reacquire_seconds').value))
        self.reacquire_max = int(self.get_parameter('reacquire_max').value)
        self.spin_rate = float(self.get_parameter('spin_rate').value)
        self.spin_ticks_limit = int(self.rate * float(self.get_parameter('spin_seconds').value))
        self.obstacle_clear = float(self.get_parameter('obstacle_clear').value)
        self.obstacle_lookahead = float(self.get_parameter('obstacle_lookahead').value)
        self.detour_speed = float(self.get_parameter('detour_speed').value)
        self.obstacles = self.load_obstacles(
            str(self.get_parameter('obstacle_file').value))
        self.k_stop = float(self.get_parameter('stop_gain').value)
        self.yellow_clear = float(self.get_parameter('yellow_clear_dist').value)
        self.follow_gap = float(self.get_parameter('follow_gap').value)
        self.contact_dist = float(self.get_parameter('contact_dist').value)
        self.conflict_half = float(self.get_parameter('conflict_half').value)
        self.oob_x = float(self.get_parameter('oob_x').value)
        self.oob_y = float(self.get_parameter('oob_y').value)
        self.reset_z = float(self.get_parameter('reset_z').value)
        self.reset_brake_ticks = int(
            self.rate * float(self.get_parameter('reset_brake_seconds').value))
        self.reset_quiet_ticks = int(
            self.rate * float(self.get_parameter('reset_quiet_seconds').value))
        self.stuck_ticks_limit = int(self.rate * float(self.get_parameter('stuck_seconds').value))
        self.conflict_ticks_limit = int(self.rate * float(self.get_parameter('conflict_seconds').value))
        self.wedge_ticks_limit = int(self.rate * float(self.get_parameter('wedge_seconds').value))
        self.held_ticks_limit = int(self.rate * float(self.get_parameter('held_seconds').value))
        self.escape_ticks = int(self.rate * float(self.get_parameter('escape_seconds').value))
        self.escape_max = int(self.get_parameter('escape_max').value)
        self.csv_path = str(self.get_parameter('path_csv').value)
        self.enable = bool(self.get_parameter('enable').value)

        self.states = {}          # group -> RED/YELLOW/GREEN
        self.agents = {}
        for name, ccfg in (self.cfg.get('cars') or {}).items():
            if not ccfg.get('drive', True):
                # 任务五把 car_2/car_3 抽去停车场当摆渡车，路线定义原样留着（随时归队），
                # 但绝不能给它发 cmd_vel：两个发布者同时驱动同一辆差速车，谁赢看 DDS 时序。
                self.get_logger().info('跳过 %s（drive: false，不在车流里）' % name)
                continue
            self.agents[name] = CarAgent(name, ccfg, self)
        if not self.agents:
            self.get_logger().error('signal_drive_params.yaml 未配置任何车辆')

        groups = set()
        for ag in self.agents.values():
            for line in ag.obey:
                groups.add(line['group'])
        for g in groups:
            self.states[g] = 'GREEN'
            self.create_subscription(
                String, f'traffic_light/{g}/state',
                lambda msg, gg=g: self.on_state(msg, gg), 10)

        self.create_subscription(ModelStates, '/model_states', self.on_models, 10)
        self.status_pub = self.create_publisher(String, '/signal_drive/status', 10)
        self.set_state_cli = None

        self.fh = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.w = csv.writer(self.fh)
        self.w.writerow(['t', 'car', 'x', 'y', 'yaw', 'speed', 'limited_by', 'phase'])
        self.t0 = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info(
            'SignalDriveNode: cars=%s groups=%s cfg=%s' % (
                list(self.agents), sorted(groups), path))

    def on_state(self, msg, group):
        self.states[group] = msg.data.strip().upper()

    def on_models(self, msg):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            ag = self.agents.get(name)
            if ag is None:
                continue
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            ag.update_pose(pose.position.x, pose.position.y, yaw,
                           twist.linear.x * math.cos(yaw) + twist.linear.y * math.sin(yaw),
                           math.hypot(twist.linear.x, twist.linear.y))

    def speed_limit(self, ag):
        """返回 (限速, 拦停者名字)：信号灯优先，其次跟车"""
        limit, who, phase = ag.cruise, '', ''
        ag.contact = False
        ag.conflict = False
        ag.stopped = False
        ux_all, uy_all = math.cos(ag.yaw), math.sin(ag.yaw)

        for line in ag.obey:
            head = float(line.get('heading', 0.0))
            ux, uy = math.cos(head), math.sin(head)
            rx, ry = float(line['x']) - ag.x, float(line['y']) - ag.y
            along = rx * ux + ry * uy        # 到停车线的纵向距离（车心）
            lateral = abs(-rx * uy + ry * ux)
            # 解除约束的判据是"车体已完全越过停车线"（along < -0.9 ≈ 半车长+余量），
            # 而不是"贴近线 0.3 m"。旧阈值会让恰好停在线前的车在红灯下被放行，
            # 审计抓到过 car_4 以 0.55 m/s 直接穿过红灯。
            if along < -0.9 or along > 25.0 or lateral > float(line.get('lane_half', 6.0)):
                continue                     # 已过线 / 太远 / 不在该车道
            ph = self.states.get(line['group'], 'GREEN')
            must_stop = ph == 'RED' or (ph == 'YELLOW' and along > self.yellow_clear)
            if not must_stop:
                continue
            if abs(wrap(ag.yaw - head)) > 1.0:
                continue                     # 车头不朝这条线（横向/对向腿）就不受它管
            remain = along - float(line.get('standoff', 2.2))
            v = 0.0 if remain <= 0.05 else max(0.0, self.k_stop * remain)
            if v < limit:
                limit, who, phase = v, line['group'], ph
                if v <= 0.02:
                    ag.stopped = True

        for other in self.agents.values():
            if other is ag or not other.seen:
                continue
            rx, ry = other.x - ag.x, other.y - ag.y
            along = rx * ux_all + ry * uy_all
            lateral = abs(-rx * uy_all + ry * ux_all)
            if along <= -1.0 or along > 10.0:
                continue
            if lateral > 1.8:
                continue
            if other.ring == ag.ring and ag.cyclic and other.cyclic:
                # 同环两车是单车道纵队，只按"弧长先后"处理：后车让前车，前车绝不因后车停住。
                # 角点处两车朝向接近垂直，落到下面的 cross 分支会双向互锁——实测 car_2/car_5
                # 在 (1.8,-14.6) 顶住 30 s 不动，且 limit=0 时 wedge 检测（要求 cmd>0.15）
                # 和 conflict 检测（要求 dy_cos<-0.2）都不会计数，看门狗救不了。
                if (other.s - ag.s) % ag.perim > ag.perim * 0.5:
                    continue                                 # 它在后面，不拦前车
                if 0.0 < along < self.contact_dist:
                    ag.contact = True
                if other.speed > ag.speed + 0.05:
                    continue                                 # 前车在加速离开
                if ag.stopped and other.stopped and along > self.contact_dist:
                    continue
                v = max(0.0, self.k_stop * (along - self.follow_gap))
                if v < limit:
                    limit, who = v, f'follow:{other.name}'
                continue
            dy_cos = math.cos(ag.yaw - other.yaw)
            if dy_cos < -0.2:
                if lateral > self.conflict_half:
                    continue                 # 并行的对向车道，各行其道，不算冲突
                # 正面对向来车（单向环线本不该出现）：停住别顶，交给看门狗复位
                ag.conflict = True
                if along > -0.5 and limit > 0.0:
                    limit, who = 0.0, f'conflict:{other.name}'
                continue
            if dy_cos < 0.3:
                # 近乎垂直＝路口交叉：按灯通行，只在贴得很近时按距离减速，避免互锁
                if along > 3.0:
                    continue
                v = max(0.0, self.k_stop * (along - 1.6))
                if v < limit:
                    limit, who = v, f'cross:{other.name}'
                continue
            if 0.0 < along < self.contact_dist:
                ag.contact = True              # 车体重叠
            if other.speed > ag.speed + 0.05:
                continue                       # 前车在加速离开，不用管
            if ag.stopped and other.stopped and along > self.contact_dist:
                continue                       # 两车都被红灯停住且未接触，各自守线
            v = max(0.0, self.k_stop * (along - self.follow_gap))
            if v < limit:
                limit, who = v, f'follow:{other.name}'
        return limit, who, phase

    def load_obstacles(self, path):
        """读入静态碰撞盒足迹（tools/gen_obstacle_config.py 从 world 生成）。

        清单缺失时返回空表，车就退回"只守车道走廊"的行为，不会拒绝行驶。
        """
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'obstacles.yaml')
        if not os.path.exists(path):
            self.get_logger().warn(f'障碍物清单不存在：{path}（跳过几何避让）')
            return []
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        out = []
        for o in cfg.get('obstacles') or []:
            r = o.get('rect')
            if not r or len(r) != 4:
                continue
            # yaml 里是 [x0, y0, x1, y1]（min/max 习惯写法），函数签名要 x0,x1,y0,y1
            out.append((float(r[0]), float(r[2]), float(r[1]), float(r[3]),
                        str(o.get('name', '?'))))
        self.get_logger().info(f'已载入静态碰撞盒 {len(out)} 个：{path}')
        return out

    def avoid_offset(self, ag, ppx, ppy, ux, uy, along, lat):
        return obstacle_offset(self.obstacles, self.obstacle_clear,
                               self.obstacle_lookahead, ag.lane_half,
                               ppx, ppy, ux, uy, along, lat)

    def on_timer(self):
        if not self.enable:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        for a in self.agents.values():
            if a.seen:
                a.update_arc()
        status = []
        for ag in self.agents.values():
            if not ag.seen:
                status.append(f'{ag.name}=未见到')
                continue
            if not ag.t_seeded:                 # 首个有效 tick 起算航点超时，并按实际位姿入线
                ag.t_seeded = True
                ag.last_wp_t = now
                ag.snap_to_route(self)
            if ag.yaw_prev is None:
                ag.yaw_prev, ag.t_prev = ag.yaw, now
            elif now - ag.t_prev >= 0.6 / self.rate:
                # 用实测 yaw 差分而不是指令 ω：被对向车楔住绕接触点转时，物理引擎给的
                # 角速度是指令值的三倍，只看 cmd_vel 永远看不见。dt 过小的 tick 直接跳过
                # 不更新基准（仿真时钟会把相邻 tick 量化成同一时刻）。
                ag.wmag = wrap(ag.yaw - ag.yaw_prev) / (now - ag.t_prev)
                ag.yaw_prev, ag.t_prev = ag.yaw, now
            if ag.reset_phase:
                # 复位途中只做一件事：持续零速。出界判定、倒车脱困、跟车让路全都别叠指令，
                # 阶段 1 就是靠"原地不动"把轮位误差放掉的。事故窗口也要写进 CSV，
                # 否则审计里复位前后看起来是"车没动"，正好把弹射藏起来。
                ag.reset_left -= 1
                ag.pub.publish(Twist())
                ag.cmd_prev = 0.0
                if ag.reset_left <= 0:
                    if ag.reset_phase == 1:
                        if self.place_car(ag):
                            ag.reset_phase = 2
                            ag.reset_left = self.reset_quiet_ticks
                        else:
                            ag.reset_phase = 0
                    else:
                        ag.reset_phase = 0
                tag = ('复位刹轮' if ag.reset_phase == 1
                       else '复位静默' if ag.reset_phase == 2 else '复位结束')
                status.append(f'{ag.name}={tag} 剩{max(ag.reset_left, 0) / self.rate:.1f}s')
                self.w.writerow([round(now - self.t0, 2), ag.name, round(ag.x, 3),
                                 round(ag.y, 3), round(ag.yaw, 3), round(ag.speed, 3),
                                 ag.reset_reason or 'reset', tag])
                continue
            if (abs(ag.x) > max(self.oob_x, ag.env_x)
                    or abs(ag.y) > max(self.oob_y, ag.env_y)):
                # 阈值取"参数"和"这台车路线包络"里更宽的那个：参数还能当现场收紧的下限用，
                # 但绝不能再比路线本身更紧 —— 那正是把正常行驶判成出界的那颗地雷。
                self.reset_car(ag, now, '出界')
                status.append(f'{ag.name}=出界已复位')
                continue
            if ag.escape_left > 0:              # 倒车脱困中：不推进航点、不做跟车判断
                ag.escape_left -= 1
                t = Twist()
                t.linear.x = -0.22
                t.angular.z = 0.5 * (1 if ag.escape_try % 2 else -1)
                ag.pub.publish(t)
                ag.cmd_prev = t.linear.x
                status.append(f'{ag.name}=倒车脱困{ag.escape_left / self.rate:.1f}s')
                continue
            if ag.cmd_prev > 0.15 and ag.vmag < 0.05:
                ag.wedge_ticks += 1             # 给了油却几乎不动 → 顶到障碍物
            elif (ag.off_lane and not ag.stopped and ag.vmag < 0.05
                    and abs(ag.wmag) < 0.05):
                # 车头楔进路缘/桩列之后，控制器只会走"边刹边转/钉停"那两条分支，线速度
                # 常年 0 或 -0.10，上面那条 cmd_prev>0.15 永不成立 → 顶住计数恒 0 → 车能
                # 在这儿一直坐到航点超时，只能靠"离道 4s→重取航段"来回打转：本轮 car_1 在
                # (-0.6,-18.9) 反复"第1次"、car_5 在 (23.4,-19.3) 全程 0.00 m 都是这个死区。
                # 补一条与油门无关的判据：离道 + 不动 + 不转，且不是被红灯要求停着，就按
                # 顶住处理，走同一套倒车脱困（顶住计数→escape→封顶后才复位）。
                ag.wedge_ticks += 1
            else:
                ag.wedge_ticks = 0
                if ag.vmag > 0.15:
                    ag.escape_try = 0
            if ag.wedge_ticks > self.wedge_ticks_limit:
                ag.wedge_ticks = 0
                ag.escape_try += 1
                if ag.escape_try > self.escape_max:
                    self.reset_car(ag, now, '顶住障碍卡死')
                    ag.escape_try = 0
                    status.append(f'{ag.name}=卡死已复位')
                    continue
                ag.escape_left = self.escape_ticks
                self.get_logger().warn(
                    f'{ag.name}: 在({ag.x:.1f},{ag.y:.1f})给油不走，倒车脱困 第{ag.escape_try}次')
                continue
            if ag.off_lane_ticks > self.off_lane_reacquire_ticks:
                # 索引跑偏（车实际在 A 段、控制器还按 B 段算）时，横向误差极大而航向误差接近 0，
                # 于是车朝着"远处的错误航段"直着爬，最后楔进路边灯杆被弹飞（本轮 car_5 在
                # (5.1,2.0) 顶住 tl_j3_ns_ped，复位时被甩到 x=32.2 出界）。先按当前位姿重取
                # 航段：只换索引、不改位姿，轨迹上不留瞬移；连续几次都回不来才复位。
                ag.off_lane_ticks = 0
                ag.reacquire_try += 1
                t = Twist()
                ag.pub.publish(t)
                ag.cmd_prev = 0.0
                if ag.reacquire_try > self.reacquire_max:
                    ag.reacquire_try = 0
                    self.reset_car(ag, now, '离道无法回线')
                    status.append(f'{ag.name}=离道已复位')
                    continue
                ag.snap_to_route(self)
                ag.last_wp_t = now
                ag.lat_target = 0.0
                self.get_logger().warn(
                    f'{ag.name}: 离道 {self.off_lane_reacquire_ticks / self.rate:.0f}s '
                    f'仍在({ag.x:.1f},{ag.y:.1f})，按位姿重取航段 第{ag.reacquire_try}次')
                status.append(f'{ag.name}=重取航段{ag.reacquire_try}')
                continue
            for _ in range(3):
                # 前视跟踪：在当前航段上取前视点，而不是直冲航点终点
                tx, ty = ag.route[ag.idx]
                dx, dy = tx - ag.x, ty - ag.y
                ppx, ppy = ag.route[(ag.idx - 1) % len(ag.route)]
                legx, legy = tx - ppx, ty - ppy
                leg_len = math.hypot(legx, legy) or 1.0
                along = ((ag.x - ppx) * legx + (ag.y - ppy) * legy) / leg_len
                # 换段点尽量贴近角点：提前太多会让前视点落到新航段上，车斜切内弯刮到路侧建筑
                lead = self.turn_lead
                if math.hypot(dx, dy) < ag.arrive_radius or along > leg_len - lead:
                    # 投影越过航点即切换，并用新航段重算前视点（否则会沿旧航段直冲出图）
                    ag._next_waypoint(self, now, False)
                    continue
                if now - ag.last_wp_t > ag.wp_timeout:
                    # 超时绝不跳点：下一个航点可能在侧后方，斜穿街区会撞楼、撞对向车
                    ag.last_wp_t = now
                    self.get_logger().warn(
                        f'{ag.name}: 航点 {ag.idx} 超时，保持当前目标（交给脱困/复位）')
                break
            ux, uy = legx / leg_len, legy / leg_len
            lat = -(ag.x - ppx) * uy + (ag.y - ppy) * ux
            want, blocked, in_band = self.avoid_offset(
                ag, ppx, ppy, ux, uy, along, lat)
            # 目标偏移做一阶平滑：禁行带会在换段瞬间出现/消失，不平滑会让前视点横跳。
            ag.lat_target += 0.15 * (want - ag.lat_target)
            detour = abs(ag.lat_target) > 0.05
            ag.off_lane = abs(lat - ag.lat_target) > self.lane_guard
            if ag.off_lane:
                ag.off_lane_ticks += 1
            else:
                ag.off_lane_ticks = 0
                ag.reacquire_try = 0
            narrow = ag.off_lane or in_band
            # 前视点按"沿路径的弧长"取，且允许 along 为负：刚换段时车还差 1.2 m 才到角点，
            # 若把 along 夹到 0，前视点会一下跳到角点后 2.5 m，航向误差被放大到 1.1~1.6 rad，
            # 车带着 0.7 m/s 冲过角点刮到路侧建筑甚至出图。
            if narrow:
                # 离道/压进禁行带：前视点收到 1.2 m，横向误差才能换成足够大的回正角。
                # 保持远处前视点只会让车以小夹角一路斜穿，审计里"沿路缘磨过去"就是这种。
                la = min(leg_len, max(along + 1.2, 0.6))
            else:
                la = min(leg_len, max(along + max(2.5, ag.cruise * 3.0), 0.6))
            lx = ppx + ux * la - uy * ag.lat_target
            ly = ppy + uy * la + ux * ag.lat_target   # 前视点：贴着当前车道纵向位置取目标
            err = wrap(math.atan2(ly - ag.y, lx - ag.x) - ag.yaw)

            ag.update_arc()             # 航点可能刚推进，重算弧长再判同环先后
            limit, who, phase = self.speed_limit(ag)
            if detour and limit > self.detour_speed:
                limit, who = self.detour_speed, (
                    'obstacle:' + ('blocked' if blocked else 'detour'))
            if narrow and limit > self.off_lane_speed:
                # 只在收紧限速是最紧约束时才改写 limited_by，别把跟车/红灯的归因盖掉
                limit, who = self.off_lane_speed, ('inband' if in_band else 'offlane')

            # 进角减速：转弯半径 r=v/ω，所以速度必须被"离角点多近"压住。
            # 原来这里用的是 leg_len - along（当前航段剩余里程），但换段发生在角点**之前**
            # lead=1.2 m，换完段车在**新**段上的 along≈-1.2，于是 to_corner=leg_len+1.2，
            # 这条限速恰好在"正要开始转"的那一帧自己失效：车带着 cruise 0.7 进 90° 弯
            # （r=1.56 m、转完要 2.45 m 弧长）→ 冲过角点楔进南侧 c80 路缘/桩列 → 离道 →
            # 复位 → 弹射，风暴自持。改成"到最近角点顶点的欧氏距离"（正在绕的这个角=本段
            # 起点 route[idx-1]，下一个角=route[idx]，取小），限速就横跨角的两侧，换段清不掉。
            cx, cy = ag.route[(ag.idx - 1) % len(ag.route)]
            to_corner = min(math.hypot(ag.x - tx, ag.y - ty),
                            math.hypot(ag.x - cx, ag.y - cy))
            if to_corner < 2.2:
                limit = min(limit, max(0.12, self.max_ang * max(to_corner, 0.25)))

            # 曲率可行性上限：纯追踪要的曲率是 2·sin|err|/la，而轮子最多给 ω/v，所以
            # v 还得 <= ω·la/(2 sin|err|)。换段那一瞬 err≈47°、la≈1.3 m → 要的半径只有
            # 0.88 m，而 cruise 0.70 能实现的半径是 v/ω=1.56 m，转不过来就带着大横偏甩
            # 外切 —— 本轮实测车队贴在 y≈-18.9 那条带（配置中心线 y=-16.8 南 2.1 m），
            # 车头正好楔进 c80 路缘/桩列。上面那道 ω·d 在同一处给的是 0.54，比这里要的
            # 0.40 还宽一档，两道都要：d 管"离角多远"，err 管"当下这条视线要多急的转"。
            # err 夹在 0.12~1.5 rad：大角度掉头由下面 |err|>1.6 的边刹边转分支接管。
            ea = min(max(abs(err), 0.12), 1.5)
            limit = min(limit, max(0.12, self.max_ang * la / (2.0 * math.sin(ea))))

            if ag.contact:
                ag.stuck_ticks += 1
            else:
                ag.stuck_ticks = 0
            if ag.conflict and ag.vmag < 0.05:
                ag.conflict_ticks += 1
            else:
                ag.conflict_ticks = 0

            # 被"车"拦停且长时间不动 = 互锁（被灯拦停不算，那本来就该等）。
            # wedge 检测要求 cmd>0.15、conflict 检测要求对向，两者都覆盖不到 limit=0 的互锁。
            if limit <= 0.02 and ag.vmag < 0.05 and who.startswith(('follow:', 'cross:')):
                ag.held_ticks += 1
            else:
                ag.held_ticks = 0
            if ag.held_ticks > self.held_ticks_limit:
                ag.held_ticks = 0
                ag.escape_try += 1
                if ag.escape_try > self.escape_max:
                    self.reset_car(ag, now, who)
                    ag.escape_try = 0
                    status.append(f'{ag.name}=互锁已复位({who})')
                    continue
                ag.escape_left = self.escape_ticks
                self.get_logger().warn(
                    f'{ag.name}: 被 {who} 拦停 {self.held_ticks_limit / self.rate:.0f}s 不动，'
                    f'倒车脱困 第{ag.escape_try}次')
                continue

            if ag.stuck_ticks > self.stuck_ticks_limit:
                self.reset_car(ag, now, who)
                status.append(f'{ag.name}=车体重叠已复位({who})')
                continue
            if ag.conflict_ticks > self.conflict_ticks_limit:
                # 对向顶住：先倒车让离，多次仍不通再复位。直接复位会在轨迹上留下瞬移。
                ag.conflict_ticks = 0
                ag.escape_try += 1
                if ag.escape_try > self.escape_max:
                    self.reset_car(ag, now, who)
                    ag.escape_try = 0
                    status.append(f'{ag.name}=对向顶死已复位({who})')
                    continue
                ag.escape_left = self.escape_ticks
                self.get_logger().warn(
                    f'{ag.name}: 与 {who} 对向顶住({ag.x:.1f},{ag.y:.1f})，倒车让离 第{ag.escape_try}次')
                continue

            if abs(ag.wmag) > self.spin_rate and (ag.contact or ag.conflict or ag.off_lane):
                ag.spin_ticks += 1
            else:
                ag.spin_ticks = 0
            if ag.spin_ticks > self.spin_ticks_limit:
                # 被顶住打转：实测 ω 远大于指令上限，说明车在绕着别的东西转而不是在转向。
                # 这时倒车只会越转越偏（审计里 car_1 被转到 1.5 rad/s、横滑 1.4 m，
                # 脱困指令又叠上去），所以零速钉住，把让开的责任交给对方车的脱困逻辑。
                ag.spin_ticks = 0
                t = Twist()
                ag.pub.publish(t)
                ag.cmd_prev = 0.0
                status.append(f'{ag.name}=打转钉停 ω={ag.wmag:.2f} 横偏{lat:+.1f}')
                continue

            t = Twist()
            if abs(err) > 1.6:
                # 大角度偏差（掉头/被甩偏）：边转边主动刹。diff_drive 没有刹车，
                # 旧写法在这里把 ω 也归零"等惯性滑停"，结果车带着 0.7 m/s 直冲出路线：
                # car_5 就是这么穿过 16.8,4.6 角点跑到 x=22 出界的。
                t.linear.x = -0.10 if ag.speed > 0.12 else 0.0
                t.angular.z = clamp(self.k_turn * err, -self.max_ang, self.max_ang)
            elif limit <= 0.02:
                # 停车等待：给 0 只是不再加油，80 kg 车体还会滑 1.5~2 m，车头会压过停车线
                # （审计里表现为"红灯越线停车"）。用小幅反推把它钉在 standoff 处。
                t.linear.x = -0.12 if ag.speed > 0.06 else 0.0
                t.angular.z = clamp(0.4 * err, -0.08, 0.08)
                ag.stopped_ticks += 1
            else:
                t.linear.x = min(limit, ag.cruise if abs(err) < 0.3 else ag.cruise * 0.45)
                t.angular.z = clamp(self.k_turn * err, -self.max_ang, self.max_ang)
                ag.stopped_ticks = 0
            ag.pub.publish(t)
            ag.cmd_prev = t.linear.x
            tag = who or '-'
            if phase:
                tag += f'({phase})'
            ag.status = (f'{ag.name}=({ag.x:.1f},{ag.y:.1f}) v={ag.speed:.2f} '
                         f'横偏{lat:+.1f} 限{limit:.2f} {tag}')
            status.append(ag.status.rstrip())
            self.w.writerow([round(now - self.t0, 2), ag.name, round(ag.x, 3), round(ag.y, 3),
                             round(ag.yaw, 3), round(ag.speed, 3), who, phase])

        s = String()
        s.data = ' | '.join(status)
        self.status_pub.publish(s)

    def reset_car(self, ag, now, reason):
        """复位请求：先零速刹轮，到点由 tick 循环的阶段 2 真正摆位。

        旧写法在这里直接 SetEntityState，于是把差速控制器攒下的轮位误差连同车一起搬到
        新位置释放出来（车沿车头冲到 4~8.5 m/s，见 reset_brake_seconds 的说明）。
        车辆被物理引擎甩出场景/互相顶死时，先刹停再放回当前航段前的车道。
        """
        if ag.reset_phase or now - ag.last_reset < 5.0:
            return
        ag.last_reset = now
        ag.reset_phase = 1
        ag.reset_left = self.reset_brake_ticks
        ag.reset_reason = reason
        ag.escape_left = 0
        ag.wedge_ticks = 0
        ag.cmd_prev = 0.0
        self.get_logger().warn(
            f'{ag.name}: 请求复位（{reason}）@({ag.x:.1f},{ag.y:.1f})，'
            f'先零速刹轮 {self.reset_brake_ticks / self.rate:.1f}s')

    def landing_clear(self, x, y):
        """落点要与每个静态盒 footprint 都留出 obstacle_clear + 车半宽（在盒内算 0）"""
        need = self.obstacle_clear + 0.55
        for x0, x1, y0, y1, _nm in self.obstacles:
            dx = max(x0 - x, 0.0, x - x1)
            dy = max(y0 - y, 0.0, y - y1)
            if math.hypot(dx, dy) < need:
                return False
        return True

    def reset_landing(self, ag):
        """在当前航段上往后退，找一个既没别的车、也没顶进碰撞盒的落点"""
        tx, ty = ag.route[ag.idx]
        px, py = ag.route[(ag.idx - 1) % len(ag.route)]
        dx, dy = tx - px, ty - py
        d = math.hypot(dx, dy) or 1.0
        ux, uy = dx / d, dy / d
        yaw = math.atan2(uy, ux)
        for back in (3.0, 5.5, 8.0, 11.0, 14.0):
            x, y = tx - ux * back, ty - uy * back
            if all(math.hypot(x - o.x, y - o.y) > 4.5
                   for o in self.agents.values() if o is not ag and o.seen) \
                    and self.landing_clear(x, y):
                return x, y, yaw
        # 五个候选位都不干净（比如整段走廊被灯杆挤满）：仍按最近的落位，但要留话，
        # 让审计能看出这次复位是"勉强落地"，落点本身可能还会刮到东西。
        self.get_logger().warn(f'{ag.name}: 复位落点周围车/盒都不干净，按 3.0 m 候选强落')
        return tx - ux * 3.0, ty - uy * 3.0, yaw

    def place_car(self, ag):
        """执行摆位（复位阶段 2 的入口）。返回 False 表示服务未就绪、本次复位作废"""
        if self.set_state_cli is None:
            self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        if not self.set_state_cli.service_is_ready():
            self.get_logger().warn(f'{ag.name}: /set_entity_state 未就绪，暂无法复位')
            return False
        ox, oy = ag.x, ag.y
        x, y, yaw = self.reset_landing(ag)
        req = SetEntityState.Request()
        req.state.name = ag.name
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        # 用实测静止高度 6.70 而不是 world 的 spawn 高度 7.0：那 0.3 m 的自由落体
        # 会让复位后的车头带着 2.4 m/s 的垂直速度砸回路面，既把卡口"实测速度"污染成
        # 2~5 m/s，也容易把车弹飞出界（复位连环触发的一个来源）。
        req.state.pose.position.z = self.reset_z
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'
        req.state.twist.linear.x = 0.0     # 复位必须清零速度：带着被甩出的动能继续跑会连环出界
        req.state.twist.angular.z = 0.0
        self.set_state_cli.call_async(req)
        ag.update_pose(x, y, yaw, 0.0)
        ag.stuck_ticks = 0
        ag.conflict_ticks = 0
        ag.wedge_ticks = 0
        ag.escape_left = 0
        ag.escape_try = 0
        ag.cmd_prev = 0.0
        ag.stopped_ticks = 0
        ag.spin_ticks = 0
        ag.off_lane_ticks = 0
        ag.reacquire_try = 0
        ag.wmag = 0.0
        ag.yaw_prev = None          # 复位是瞬移，yaw 差分会算出巨大的假角速度
        ag.lat_target = 0.0
        self.get_logger().warn(
            f'{ag.name}: 复位到({x:.1f},{y:.1f}) 原点({ox:.1f},{oy:.1f}) 原因={ag.reset_reason}')
        return True

    def destroy_node(self):
        try:
            self.fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SignalDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
