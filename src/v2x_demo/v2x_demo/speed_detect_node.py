#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务四（三）.(四) 条目 1~4：测速路段仿真 —— 点位、雷达测速、阈值判定、流量统计。

  输入  /model_states      gazebo_msgs/ModelStates   全部模型的位姿+速度（实测 ~97 Hz）
  输入  /clock           rosgraph_msgs/Clock       仿真时钟（实测 ~10 Hz，BEST_EFFORT）
  输入  /plate/number      std_msgs/String           车辆真值车牌（易失，缺则用 YAML 兜底）
  服务  /speed/zones       v2x_msgs/SetSpeedZone     点位与限速阈值热改
  输出  /speed/capture     v2x_msgs/SpeedCapture     定点瞬时测速抓拍
  输出  /speed/segment     v2x_msgs/SpeedSegment     区间平均测速结果
  输出  /speed/alert       v2x_msgs/SpeedAlert       超速告警（瞬时/区间两类）
  输出  /speed/stats       v2x_msgs/SpeedStats       每点位一条流量统计

测速的两条通道都只吃位姿：
  * speed_pose_kmh —— |Δp|/Δt，位姿差分的真值参照；
  * speed_kmh      —— 把 Δp/Δt 投影到雷达波束轴上再叠高斯噪声，即"雷达测速效果"。
投影这一步是真实固定测速雷达的**余弦误差**：波束轴与行车方向夹角 θ 时只能读到 v·cosθ。
点位装的是车道名义方向，车切弯/斜穿时 θ≠0，读数就偏低 —— 这个偏低是物理正确的，
不是 bug，所以两条通道一起上报，界面与导出都能看见差值。判定取"过车期间最近Approach
时刻的雷达读数"，峰值另存一列，两者不等价于彼此。
"""
import math
import os
import random
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from v2x_msgs.msg import SpeedAlert, SpeedCapture, SpeedSegment, SpeedStats
from v2x_msgs.srv import SetSpeedZone

MS2KMH = 3.6
BEAM_AUTO = 999.0


def stamp_of(sec):
    """把仿真秒（float）写成 builtin_interfaces/Time，供 header.stamp 用。"""
    t = Time()
    t.sec = int(sec)
    t.nanosec = int(round((sec - int(sec)) * 1e9)) % 1_000_000_000
    return t


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Zone:
    """一个测速点位。instant 只有 center；segment 有 entry/exit 两个断面。"""

    def __init__(self, d):
        self.id = d['id']
        self.mode = 1 if str(d.get('mode', 'instant')).startswith('seg') else 0
        self.road = d.get('road', '')
        self.limit = float(d.get('limit_kmh', 0.0))
        r = float(d.get('radius', 0) or 0)
        self.radius = r if r > 0 else 1.6
        c = d.get('center') or d.get('entry') or [0.0, 0.0]
        self.center = (float(c[0]), float(c[1]))
        e = d.get('entry') or c
        x = d.get('exit') or c
        self.entry = (float(e[0]), float(e[1]))
        self.exit = (float(x[0]), float(x[1]))
        r = d.get('radar')
        self.radar = (float(r[0]), float(r[1])) if r else self.center
        b = d.get('beam', BEAM_AUTO)
        self.beam = None if abs(float(b) - BEAM_AUTO) < 1e-6 else math.radians(float(b))
        self.length = math.hypot(self.exit[0] - self.entry[0],
                                 self.exit[1] - self.entry[1])

    def beam_dir(self):
        """波束轴单位向量。
        区间点位缺省按 entry->exit（车道名义方向，正对来车，夹角 0）；
        瞬时点位没有通行方向信息，缺省只能按"雷达指向点位"的视线轴装 —— 侧装读数会
        按余弦大幅偏低，所以 YAML 里的瞬时点位都显式写了 beam（沿车道方向）。"""
        if self.beam is not None:
            return (math.cos(self.beam), math.sin(self.beam))
        if self.mode:
            ax, ay = self.exit
            bx, by = self.entry
        else:
            ax, ay = self.center
            bx, by = self.radar
        d = math.hypot(ax - bx, ay - by)
        return ((ax - bx) / d, (ay - by) / d) if d > 1e-6 else (1.0, 0.0)

    def line(self):
        return ('%s %s limit=%.2f km/h r=%.1f %s' % (
            self.id,
            '区间' if self.mode else '瞬时',
            self.limit, self.radius,
            ('entry(%.1f,%.1f)->exit(%.1f,%.1f) L=%.1fm'
             % (self.entry[0], self.entry[1], self.exit[0], self.exit[1], self.length))
            if self.mode else 'center(%.1f,%.1f)' % self.center))


class Encounter:
    """一次过车过程的累积量：最近 approach 样本 + 峰值 + 首次进入时刻。"""
    __slots__ = ('dist', 'radar', 'pose_kmh', 'peak', 'ts', 'pose', 'heading',
                 'beam_deg', 'range_m', 'noise')

    def __init__(self):
        self.dist = None
        self.radar = 0.0
        self.pose_kmh = 0.0
        self.peak = 0.0
        self.ts = 0.0
        self.pose = None
        self.heading = 0.0
        self.beam_deg = 0.0
        self.range_m = 0.0
        self.noise = 0.0


class SpeedDetectNode(Node):
    def __init__(self):
        super().__init__('speed_detect_node')
        self.declare_parameter('config_file', '')
        self.declare_parameter('vehicle_prefix', 'car_')
        # use_sim_time 由 rclpy 自己声明（launch/ros2 run 的 -p 走的是已声明参数），
        # 这里再 declare 会抛 ParameterAlreadyDeclaredException —— 见 traffic_light_controller 的同款注释。

        path = self.get_parameter('config_file').value
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'speed_zones.yaml')
        cfg = self.load_config(path)
        sp = cfg.get('speed', {})
        self.noise = float(sp.get('radar_noise_kmh', 0.3))
        self.min_pass = float(sp.get('min_pass_kmh', 0.3))
        self.ttl = float(sp.get('segment_ttl_s', 300.0))
        self.vel_window = float(sp.get('vel_window_s', 0.30))
        # 物理上限：车队巡航只有 0.7 m/s = 2.52 km/h，窗口弦速超过它不可能是行驶。
        # 5 m 的弦长护栏拦不住小跳变——1 m 的复位跳变除以 0.2 s 窗口就是 18 km/h。
        self.max_sane = float(sp.get('max_sane_kmh', 12.0))
        self.window = float(sp.get('stats_window_s', 120.0))
        self.prefix = self.get_parameter('vehicle_prefix').value

        self.yaml_zones = [dict(z) for z in sp.get('zones', [])]
        self.zones = [Zone(z) for z in self.yaml_zones]
        self.plates = dict(sp.get('plates') or {})
        self.enc = {}        # (vehicle, zone_id) -> Encounter，瞬时点位用
        self.open_seg = {}   # (vehicle, zone_id) -> 进入断面后的累积，区间用
        self.inside = {}     # 圈内外状态（瞬时=(车,点位)，区间=('e'/'x',车,点位)），做边沿触发
        # vehicle -> (x, y, 原始 pose)：位姿流只负责"最新值"，评估节奏交给 /clock
        self.latest = {}
        # vehicle -> deque[(时钟, x, y)]：速度用窗口弦长/窗口时长，不做逐帧差分
        self.hist = defaultdict(lambda: deque(maxlen=16))
        self.samples = {}    # zone_id -> [(ts, kmh, over)]
        self.total = {}      # zone_id -> [flow, over]
        self.seq = 0
        self.err_count = 0
        self.jump_drop = 0     # 因超物理上限而作废的帧数
        self.seeded = False
        self.last_tick_wall = time.monotonic()
        self.last_clock_warn = 0.0

        self.pub_cap = self.create_publisher(SpeedCapture, '/speed/capture', 50)
        self.pub_seg = self.create_publisher(SpeedSegment, '/speed/segment', 50)
        self.pub_alert = self.create_publisher(SpeedAlert, '/speed/alert', 50)
        self.pub_stats = self.create_publisher(SpeedStats, '/speed/stats', 50)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 20)
        # /clock 的发布侧是 BEST_EFFORT，默认 reliable 订阅收不到（实测 gz 11 就是这样，
        # 用默认 QoS 订阅会得到 "offering incompatible QoS ... RELIABILITY" 且永远零事件）。
        self.create_subscription(
            Clock, '/clock', self.on_clock,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/plate/number', self.on_plate, 10)
        self.create_service(SetSpeedZone, '/speed/zones', self.on_zones)
        self.create_timer(float(sp.get('stats_period_s', 10.0)), self.publish_stats)
        self.create_timer(5.0, self.check_clock)

        self.get_logger().info('[speed] %s：%d 个点位（配置 %s），速度基线 %.2f s' % (
            '测速路段节点启动', len(self.zones), path, self.vel_window))
        for z in self.zones:
            self.get_logger().info('[speed]   ' + z.line())

    @staticmethod
    def load_config(path):
        import yaml
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f) or {}

    def on_plate(self, m):
        parts = m.data.split('|')
        if len(parts) >= 4 and parts[3]:
            self.plates[parts[0]] = parts[3]

    # ------------------------------------------------------------------ 主循环
    def on_states(self, msg):
        """位姿流只刷新最新值，不做差分（实测 ~97 Hz）。
        在本回调里算速度就得拿节点时钟（读数是上一格 /clock，最多已经过了 0.1 s）去配
        这一帧的位置，dt 被时钟量化到 ±0.05 s；窗口只有 0.3 s 时这就是 ±17%，
        实测读数散到 3.1~6.5 km/h（真值 2.52）。所以差分挪到 on_clock。"""
        for i, name in enumerate(msg.name):
            if name.startswith(self.prefix):
                self.latest[name] = msg.pose[i]

    def on_clock(self, msg):
        """评估节奏 = 仿真时钟节奏（实测 ~10 Hz）：每格给每辆车记一个样本，
        窗口跨度是整格数，dt 与两个位置同源，量化误差只剩"车在窗口两端各差半格"的 ~3%。"""
        self.last_tick_wall = time.monotonic()
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if not self.seeded:
            if not self.latest:
                return
            self.seed(now)
            return
        for name, pose in self.latest.items():
            p = pose.position
            dq = self.hist[name]
            dq.append((now, p.x, p.y))
            # 丢掉超出窗口的旧样本：判据要落在 dq[0] 上（"这段已经太老"），
            # 落在 dq[1] 上会把窗口一路缩到只剩两个点，跨度永远到不了判据。
            # 下界写成 4：中位数至少要 3 个子区间。/clock 实测 9.8 Hz（格长 0.102 s），
            # 若允许缩到 3 个样本，窗口就长期只剩 2 个子区间，中位数会挑到最快的那格。
            while len(dq) > 4 and now - dq[0][0] > self.vel_window:
                dq.popleft()
            if len(dq) < 2 or now - dq[0][0] < self.vel_window * 0.6:
                continue
            dt = now - dq[0][0]
            dx, dy = p.x - dq[0][1], p.y - dq[0][2]
            if dt <= 1e-6:
                continue
            if dt > 2.0:
                # 时钟自己跳了（暂停/回放/复位）：这段历史不能当行驶用，重新开始攒窗口。
                dq.clear()
                continue
            if math.hypot(dx, dy) > 5.0:
                # 复位/瞬移不是行驶：把该车的在途过程全部作废，否则区间均速会算出
                # 几十 km/h 的假告警（signal_drive 卡死时会 teleport 回复位点）。
                self.drop_vehicle(name, '中途复位/瞬移，本次通行作废', p)
                continue
            vx, vy = dx / dt, dy / dt
            sub = self.window_velocity(dq)
            if sub is None:
                continue          # 窗口还没攒够 3 个子区间，中位数没有意义
            vx, vy = sub
            pose_kmh = math.hypot(vx, vy) * MS2KMH
            if pose_kmh > self.max_sane:
                # 窗口里混进了非行驶跳变：这一帧不能进任何点位的判定，
                # 否则就会用假速度告警（实测 2 m 跳变 / 0.2 s = 36 km/h，
                # 线上冒出过 20.08 km/h 的假告警和一个 44.77 km/h 的假峰）。
                self.jump_drop += 1
                if self.jump_drop <= 5 or self.jump_drop % 100 == 0:
                    self.get_logger().warn(
                        '[speed] %s 窗口中位弦速 %.1f km/h 超物理上限 %.1f（第 %d 帧，整窗位移 %.2f m/%.2f s）'
                        '%s' % (name, pose_kmh, self.max_sane, self.jump_drop,
                                math.hypot(dx, dy), dt,
                                '，在途区间通行一并作废' if math.hypot(dx, dy) > 3.0 else
                                '，只跳过该帧（刮蹭抖动不该毁掉整趟通行）'))
                if math.hypot(dx, dy) > 3.0:
                    self.drop_vehicle(name, '测速窗口内出现跳变', p)
                else:
                    dq.clear()      # 窗口重头攒，瞬时抓拍与区间在途都保留
                continue
            heading = math.degrees(math.atan2(vy, vx))
            for z in self.zones:
                # 一个点位抛异常会把整个回调打死、节点随之退出（任务三实录：识别器被
                # 一张图带崩，两轮各 10 min 的光照对照只打出零行的表）。点位之间隔离掉。
                try:
                    self.step(z, name, pose, vx, vy, pose_kmh, heading, now)
                except Exception as exc:
                    self.err_count += 1
                    if self.err_count <= 5:
                        self.get_logger().error('[speed] %s 处理 %s 失败：%r' % (
                            z.id, name, exc))
        self.expire_segments(now)

    def window_velocity(self, dq):
        """窗口内逐子区间算速度，取**中位数**那一格的向量（m/s），不用整窗弦长/整窗时长。

        弦长法会把一次单格抖动摊进整窗：0.5 m 的刮蹭抖动落在 0.2 s 的窗里就是 9 km/h，
        而车队巡航只有 2.5~3.9 km/h，界面上就成了一条假超速。中位数对单格离群免疫，
        子区间的 dt 与两端位置同样都来自 /clock 那把尺，不引入新的口径错配。
        至少要 3 个子区间：两格里一动一静时"中位"会挑到动的那格。"""
        cand = []
        for i in range(1, len(dq)):
            sdt = dq[i][0] - dq[i - 1][0]
            if sdt <= 1e-6 or sdt > 2.0:
                continue
            sx, sy = dq[i][1] - dq[i - 1][1], dq[i][2] - dq[i - 1][2]
            cand.append((math.hypot(sx, sy) / sdt, sx / sdt, sy / sdt))
        if len(cand) < 3:
            return None
        cand.sort()
        return cand[len(cand) // 2][1], cand[len(cand) // 2][2]

    def check_clock(self):
        """/clock 一断就是一条事件都不会有，而"零事件"和"没有车超速"在探针里长得一样。
        所以停摆要主动喊出来（任务三教训：零行的表是故障，不是结果）。"""
        gap = time.monotonic() - self.last_tick_wall
        if gap > 4.5 and time.monotonic() - self.last_clock_warn > 30.0:
            self.last_clock_warn = time.monotonic()
            self.get_logger().warn(
                '[speed] /clock 已 %.1f s 没有更新，测速判定停摆'
                '（查 gzserver 是否活着、use_sim_time 是否传了）' % gap)

    def seed(self, now):
        """首个时钟格只把"车辆在圈内外"的初值填满，不做任何判定。
        否则冷启动时正好压在出口断面里的那辆车会被当成"刚离开出口"，白报一条反向通过。"""
        for name, pose in self.latest.items():
            self.seed_flags(name, pose.position.x, pose.position.y)
        self.seeded = True
        self.get_logger().info('[speed] 已播种 %d 辆车的断面状态（仿真 %.1f s）' % (
            len(self.latest), now))

    def seed_flags(self, name, x, y):
        for z in self.zones:
            self.inside[(name, z.id)] = \
                math.hypot(x - z.center[0], y - z.center[1]) <= z.radius
            self.inside[('e', name, z.id)] = \
                math.hypot(x - z.entry[0], y - z.entry[1]) <= z.radius
            self.inside[('x', name, z.id)] = \
                math.hypot(x - z.exit[0], y - z.exit[1]) <= z.radius

    def step(self, z, name, pose, vx, vy, pose_kmh, heading, now):
        if z.mode == 0:
            self.step_instant(z, name, pose, vx, vy, pose_kmh, heading, now)
        else:
            self.step_segment(z, name, pose, vx, vy, now)

    @staticmethod
    def in_circle(c, p, r):
        return math.hypot(p.x - c[0], p.y - c[1]) <= r

    def radar_read(self, z, vx, vy):
        """把位姿速度投影到波束轴 + 噪声 = 雷达读数（km/h）。"""
        bx, by = z.beam_dir()
        radial = max(0.0, (vx * bx + vy * by)) * MS2KMH
        noise = random.gauss(0.0, self.noise) if self.noise > 0 else 0.0
        return max(0.0, radial + noise), noise, radial

    # ---------------------------------------------------------- 定点瞬时测速
    def step_instant(self, z, name, pose, vx, vy, pose_kmh, heading, now):
        p = pose.position
        key = (name, z.id)
        inside = self.in_circle(z.center, p, z.radius)
        if inside and not self.inside.get(key):
            self.enc[key] = Encounter()
            self.inside[key] = True
        elif not inside and self.inside.get(key):
            self.inside[key] = False
            e = self.enc.pop(key, None)
            if e is not None and e.pose is not None:
                self.emit_capture(z, name, e)
            return
        if not inside:
            return
        e = self.enc.get(key)
        if e is None:
            return
        dist = math.hypot(p.x - z.center[0], p.y - z.center[1])
        rng = math.hypot(p.x - z.radar[0], p.y - z.radar[1])
        radar, noise, radial = self.radar_read(z, vx, vy)
        if e.dist is None or dist <= e.dist:
            cos = radial / (pose_kmh + 1e-9) if pose_kmh > 1e-6 else 1.0
            e.dist, e.radar, e.pose_kmh, e.ts = dist, radar, pose_kmh, now
            e.pose = pose
            e.heading, e.range_m, e.noise = heading, rng, noise
            e.beam_deg = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        e.peak = max(e.peak, radar)

    def emit_capture(self, z, name, e):
        """header.stamp 用**最近 approach 那一帧**的时刻，不是离开圆那一刻：
        抓拍编号要能对上"超速时间/位置"，而判定样本就是那次最近点。"""
        self.seq += 1
        cid = 'cap_%s_%05d' % (z.id, self.seq)
        over = e.radar > z.limit
        m = SpeedCapture()
        m.header.stamp = stamp_of(e.ts)
        m.capture_id = cid
        m.zone_id = z.id
        m.vehicle_id = name
        m.plate_number = self.plates.get(name, '')
        m.speed_kmh = round(e.radar, 3)
        m.speed_pose_kmh = round(e.pose_kmh, 3)
        m.speed_peak_kmh = round(e.peak, 3)
        m.limit_kmh = round(z.limit, 3)
        m.over_kmh = round(e.radar - z.limit, 3)
        m.over = over
        m.heading_deg = round(e.heading, 2)
        m.beam_angle_deg = round(e.beam_deg, 2)
        m.radar_range_m = round(e.range_m, 2)
        m.noise_kmh = round(e.noise, 3)
        m.pose = e.pose
        self.pub_cap.publish(m)
        self.record(z, e.ts, e.radar, over, name)
        if over:
            self.raise_alert(SpeedAlert.KIND_INSTANT, cid, z, name, m.plate_number,
                             e.radar, e.pose, e.ts)
        self.get_logger().info(
            '[speed] 瞬时 %s %s 牌=%s 雷达%.2f 位姿%.2f 峰%.2f 夹角%.1f° 阈值%.2f %s' % (
                z.id, name, m.plate_number or '-', e.radar, e.pose_kmh, e.peak,
                e.beam_deg, z.limit, '**超速**' if over else '正常'))

    # ------------------------------------------------------------ 区间平均测速
    def step_segment(self, z, name, pose, vx, vy, now):
        p = pose.position
        key = (name, z.id)
        ke, kx = ('e',) + key, ('x',) + key
        ein = self.in_circle(z.entry, p, z.radius)
        xin = self.in_circle(z.exit, p, z.radius)
        was_e = self.inside.get(ke, False)
        was_x = self.inside.get(kx, False)
        self.inside[ke] = ein
        self.inside[kx] = xin
        radar, _, _ = self.radar_read(z, vx, vy)
        if ein and not was_e:
            # 再次进入入口断面（掉头/绕回）：旧的一次通行没有意义，直接覆盖
            self.open_seg[key] = {
                't0': now, 'p0': pose, 'v0': radar, 'max': radar, 'n': 1}
            return
        if xin and not was_x:
            o = self.open_seg.pop(key, None)
            if o is None:
                if not ein:
                    self.get_logger().warn(
                        '[speed] %s %s 反向通过区间（先见到出口断面），不计' % (z.id, name))
                return
            self.emit_segment(z, name, o, now, pose, radar)
            return
        o = self.open_seg.get(key)
        if o is not None:
            o['max'] = max(o['max'], radar)
            o['n'] += 1

    def emit_segment(self, z, name, o, now, exit_pose, exit_radar):
        tt = now - o['t0']
        if tt <= 0.5:
            return
        avg = z.length / tt * MS2KMH
        self.seq += 1
        sid = 'seg_%s_%05d' % (z.id, self.seq)
        over = avg > z.limit
        m = SpeedSegment()
        m.header.stamp = stamp_of(now)
        m.segment_id = sid
        m.zone_id = z.id
        m.vehicle_id = name
        m.plate_number = self.plates.get(name, '')
        m.entry_ts = o['t0']
        m.exit_ts = now
        m.travel_time_s = round(tt, 3)
        m.length_m = round(z.length, 3)
        m.avg_kmh = round(avg, 3)
        m.limit_kmh = round(z.limit, 3)
        m.over_kmh = round(avg - z.limit, 3)
        m.over = over
        m.max_kmh = round(o['max'], 3)
        m.entry_speed_kmh = round(o['v0'], 3)
        m.exit_speed_kmh = round(exit_radar, 3)
        m.entry_pose = o['p0']
        m.exit_pose = exit_pose
        self.pub_seg.publish(m)
        self.record(z, now, avg, over, name)
        if over:
            self.raise_alert(SpeedAlert.KIND_SEGMENT, sid, z, name, m.plate_number,
                             avg, m.exit_pose, now)
        self.get_logger().info(
            '[speed] 区间 %s %s 牌=%s L=%.1fm t=%.1fs 均速%.2f 峰%.2f 阈值%.2f %s' % (
                z.id, name, m.plate_number or '-', z.length, tt, avg, o['max'],
                z.limit, '**超速**' if over else '正常'))

    def drop_vehicle(self, name, note, pos=None):
        """瞬移/复位：在途的瞬时累积与区间通行全部作废，并按新位置重建圈内外标志。
        不重建的话标志会一直停在 True，车辆复位后重新进入入口断面时边沿不再触发（区间永久失效）；
        直接清空又会让"复位点正好落在断面里"变成一次伪进入。"""
        dropped = sum(1 for k in list(self.open_seg) if name in k)
        for k in [k for k in list(self.open_seg) if name in k]:
            self.open_seg.pop(k)
        for k in [k for k in list(self.enc) if name in k]:
            self.enc.pop(k)
        for k in [k for k in list(self.inside) if name in k]:
            self.inside.pop(k)
        self.hist.pop(name, None)          # 跨瞬移的弦长不能当作行驶
        if pos is not None:
            self.seed_flags(name, pos.x, pos.y)
        if dropped:
            self.get_logger().warn('[speed] %s %s，作废 %d 次在途区间通行' % (
                name, note, dropped))

    def expire_segments(self, now):
        for key in [k for k, o in self.open_seg.items() if now - o['t0'] > self.ttl]:
            o = self.open_seg.pop(key)
            self.get_logger().warn('[speed] %s 在途 %.0fs 未离开，按超时丢弃' % (
                key[1], now - o['t0']))

    # ---------------------------------------------------------------- 告警/统计
    def raise_alert(self, kind, ref, z, name, plate, kmh, pose, ts):
        a = SpeedAlert()
        a.header.stamp = stamp_of(ts)
        a.kind = kind
        a.ref_id = ref
        a.zone_id = z.id
        a.vehicle_id = name
        a.plate_number = plate
        a.speed_kmh = round(kmh, 3)
        a.limit_kmh = round(z.limit, 3)
        a.over_kmh = round(kmh - z.limit, 3)
        a.ts = ts
        a.pose = pose
        a.note = '瞬时' if kind == SpeedAlert.KIND_INSTANT else '区间平均'
        self.pub_alert.publish(a)

    def record(self, z, ts, kmh, over, vehicle=''):
        if kmh < self.min_pass:
            return
        self.samples.setdefault(z.id, []).append((ts, kmh, over, vehicle))
        t = self.total.setdefault(z.id, [0, 0])
        t[0] += 1
        t[1] += over

    def publish_stats(self):
        now = self.get_clock().now().nanoseconds / 1e9
        for z in self.zones:
            rows = self.samples.get(z.id, [])
            rows = [r for r in rows if now - r[0] <= self.window]
            self.samples[z.id] = rows
            t = self.total.get(z.id, [0, 0])
            m = SpeedStats()
            m.header.stamp = stamp_of(now)
            m.zone_id = z.id
            m.mode = z.mode
            m.road = z.road
            m.limit_kmh = round(z.limit, 3)
            m.window_s = self.window
            m.flow = len(rows)
            m.vehicles = len({r[3] for r in rows if r[3]})
            m.avg_kmh = round(sum(r[1] for r in rows) / len(rows), 3) if rows else 0.0
            m.max_kmh = round(max(r[1] for r in rows), 3) if rows else 0.0
            m.min_kmh = round(min(r[1] for r in rows), 3) if rows else 0.0
            m.over_count = sum(1 for r in rows if r[2])
            m.over_ratio = round(m.over_count / float(m.flow), 4) if m.flow else 0.0
            m.total_flow = t[0]
            m.total_over = t[1]
            m.pending_segment = sum(1 for k in self.open_seg if k[1] == z.id)
            self.pub_stats.publish(m)

    # ------------------------------------------------------------------ 热配置
    def on_zones(self, req, resp):
        """服务回调里抛出的异常会顺着 spin 把整个节点带走 —— 实测常量名写错成
        `SetSpeedZone.OP_LIST`（这些常量挂在 Request 上）就跑了一轮热配置、把节点打死在
        验证脚本第 3 步。外部请求是不可信的，这里兜住并回失败，不让节点陪葬。"""
        try:
            ok = self.apply_zones(req, resp)
        except Exception as exc:
            ok, resp.message = False, 'op=%d 处理失败：%r' % (req.op, exc)
            self.get_logger().error('[speed] /speed/zones ' + resp.message)
        resp.success = ok
        return self.finish(resp)

    def apply_zones(self, req, resp):
        ok = True
        if req.op == SetSpeedZone.Request.OP_LIST:
            resp.message = '当前 %d 个点位' % len(self.zones)
        elif req.op == SetSpeedZone.Request.OP_SET_LIMIT:
            z = self.find(req.zone_id)
            if z is None:
                ok, resp.message = False, '找不到点位 %s' % req.zone_id
            else:
                z.limit = float(req.limit_kmh)
                resp.message = '%s 阈值已改为 %.2f km/h' % (z.id, z.limit)
        elif req.op == SetSpeedZone.Request.OP_SET_GLOBAL_LIMIT:
            for z in self.zones:
                z.limit = float(req.limit_kmh)
            resp.message = '全部 %d 个点位阈值改为 %.2f km/h' % (
                len(self.zones), req.limit_kmh)
        elif req.op == SetSpeedZone.Request.OP_ADD:
            if req.mode:
                d = {'id': req.zone_id, 'mode': 'segment',
                     'road': req.road or '运行期新增',
                     'limit_kmh': req.limit_kmh, 'radius': req.radius,
                     'center': [req.entry_x, req.entry_y],
                     'entry': [req.entry_x, req.entry_y],
                     'exit': [req.exit_x, req.exit_y]}
            else:
                d = {'id': req.zone_id, 'mode': 'instant',
                     'road': req.road or '运行期新增',
                     'limit_kmh': req.limit_kmh, 'radius': req.radius,
                     'center': [req.center_x, req.center_y]}
            if req.radar_x > -9000:
                d['radar'] = [req.radar_x, req.radar_y]
            if abs(req.beam_deg - BEAM_AUTO) > 1e-6:
                d['beam'] = req.beam_deg
            z = Zone(d)
            self.zones = [q for q in self.zones if q.id != z.id] + [z]
            self.reset_state()
            resp.message = '点位 %s 已写入（%s）' % (z.id, z.line())
        elif req.op == SetSpeedZone.Request.OP_REMOVE:
            n = len(self.zones)
            self.zones = [q for q in self.zones if q.id != req.zone_id]
            self.reset_state()
            ok = n != len(self.zones)
            resp.message = '已删除 %d 个点位' % (n - len(self.zones))
        elif req.op == SetSpeedZone.Request.OP_CLEAR:
            self.zones = [Zone(d) for d in self.yaml_zones]
            self.reset_state()
            resp.message = '运行期点位已清空，回到 YAML 基线 %d 个点位' % len(self.zones)
        else:
            ok, resp.message = False, '未知 op=%d' % req.op
        return ok

    def find(self, zone_id):
        for z in self.zones:
            if z.id == zone_id:
                return z
        return None

    def reset_state(self):
        """点位集合变更后清掉全部在途累积与统计，再按上一帧位置重建圈内外标志。"""
        self.enc.clear()
        self.open_seg.clear()
        self.inside.clear()
        self.samples.clear()
        self.total.clear()
        for name, pose in self.latest.items():
            self.seed_flags(name, pose.position.x, pose.position.y)

    def finish(self, resp):
        resp.zones = [z.line() for z in self.zones]
        self.get_logger().info('[speed] /speed/zones -> ' + resp.message)
        return resp


def main():
    rclpy.init()
    node = SpeedDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
