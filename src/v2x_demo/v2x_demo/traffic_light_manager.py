#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多路口信号灯协调管理器 —— 协议 §三.(二) 信号灯配时仿真 + 多路口联动。

功能：
  1) 机动车灯态循环（GREEN→YELLOW→RED）+ 行人灯态（WALK→FLASHING→DONT_WALK）
  2) 多信号组联动：complementary 反相、offset 绿波协调
  3) 配时预设切换（peak/offpeak/night/default）+ 时段自动切换
  4) 事件触发：emergency 全绿、forced_green 单方向强制放行、
     traffic_light/emergency_groups 急救走廊（走廊绿灯、横向禁行）
  5) 统计输出：通过 traffic_light/<group>/stats 发布通行统计
  6) 自适应接口：订阅 traffic_light/<group>/adaptive_factor 动态调整配时
  7) 运行时手动配时：traffic_light/config 按组覆盖红/黄/绿时长与相位差

配置来源：traffic_light_params.yaml
"""
import json
import os
import time as wall_time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBool

try:
    from v2x_msgs.msg import TrafficLightState, TrafficLightStats
    HAS_V2X_MSGS = True
except ImportError:
    HAS_V2X_MSGS = False


class PedestrianPhase:
    """行人信号相位管理器 —— 独立子周期时序。

    时序规则（对齐国标 GB 14886）：
      - 机动车 GREEN / YELLOW → 行人 DONT_WALK（红灯 steady）
      - 机动车 RED 期间 → 行人独立子周期：WALK → FLASHING → DONT_WALK
        · WALK     ：绿灯行走人（= walk_duration 秒）
        · FLASHING ：闪烁警告（= flashing_duration 秒）
        · DONT_WALK：红灯 steady 直到机动车红转绿
    """
    WALK = 'WALK'
    FLASHING = 'FLASHING'
    DONT_WALK = 'DONT_WALK'

    def __init__(self, walk_duration=5.0, flashing_duration=3.0):
        self.walk_duration = walk_duration
        self.flashing_duration = flashing_duration

    def sub_durations(self, red_dur=None):
        """行人子周期时长：总和不超出本周期红灯时长，否则闪烁/清空会被截断"""
        walk, flash = self.walk_duration, self.flashing_duration
        if red_dur and red_dur > 0:
            flash = min(flash, max(1.0, 0.4 * red_dur))
            walk = min(walk, max(1.0, red_dur - flash))
        return walk, flash

    def get_phase(self, vehicle_phase, elapsed_in_red, red_dur=None):
        if vehicle_phase != 'RED':
            return self.DONT_WALK
        walk, flash = self.sub_durations(red_dur)
        if elapsed_in_red < walk:
            return self.WALK
        if elapsed_in_red < walk + flash:
            return self.FLASHING
        return self.DONT_WALK


class TrafficLightManager(Node):
    def __init__(self):
        super().__init__('traffic_light_manager')

        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_period', 0.2)
        self.declare_parameter('force_republish', 1.0)
        self.declare_parameter('enable_statistics', True)
        self.declare_parameter('enable_time_schedule', True)

        path = self.get_parameter('config_file').value
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'traffic_light_params.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        if 'groups' not in cfg and 'traffic_light_controller' in cfg:
            cfg = dict(cfg['traffic_light_controller'].get('ros__parameters', {}), **cfg)

        self.presets = cfg.get('presets') or {}
        self.groups = self._norm_groups(cfg.get('groups') or {})
        self.pedestrian = PedestrianPhase(
            walk_duration=float(cfg.get('walk_duration', 5.0)),
            flashing_duration=float(cfg.get('flashing_duration', 3.0)),
        )

        base = cfg.get('durations', {})
        self.declare_parameter('green_duration', float(base.get('green', cfg.get('green_duration', 10.0))))
        self.declare_parameter('yellow_duration', float(base.get('yellow', cfg.get('yellow_duration', 3.0))))
        self.declare_parameter('red_duration', float(base.get('red', cfg.get('red_duration', 10.0))))
        self.green = float(self.get_parameter('green_duration').value)
        self.yellow = float(self.get_parameter('yellow_duration').value)
        self.red = float(self.get_parameter('red_duration').value)

        self.preset_name = cfg.get('preset', 'default')
        self._apply_preset(self.preset_name, log=False)

        self.default_group = cfg.get('default_group', '')
        if not self.default_group and self.groups:
            self.default_group = next(iter(self.groups))

        self.time_schedule = cfg.get('time_schedule', {})
        self.enable_time_schedule = bool(self.get_parameter('enable_time_schedule').value)

        self.pubs = {}
        self.ped_pubs = {}
        self.state_pub = {}
        self.stats_pub = {}
        for gid in self.groups:
            self.pubs[gid] = self.create_publisher(String, f'traffic_light/{gid}/state', 10)
            self.ped_pubs[gid] = self.create_publisher(String, f'traffic_light/{gid}/pedestrian', 10)
            if HAS_V2X_MSGS:
                self.state_pub[gid] = self.create_publisher(
                    TrafficLightState, f'traffic_light/{gid}/state_detail', 10)
                self.stats_pub[gid] = self.create_publisher(
                    TrafficLightStats, f'traffic_light/{gid}/stats', 10)

        self.legacy_pub = self.create_publisher(String, 'traffic_light/state', 10)

        self.create_subscription(String, 'traffic_light/preset', self.on_preset, 10)
        self.create_subscription(String, 'traffic_light/config', self.on_config, 10)
        self.create_subscription(String, 'traffic_light/forced_green_group',
                                 self.on_forced_green_group, 10)
        self.create_subscription(String, 'traffic_light/emergency_groups',
                                 self.on_emergency_groups, 10)
        self.config_ack_pub = self.create_publisher(String, 'traffic_light/config_ack', 10)
        self.create_service(SetBool, '~/emergency', self.on_emergency)
        self.create_service(SetBool, '~/forced_green', self.on_forced_green)
        self.add_on_set_parameters_callback(self.on_params)

        for gid in self.groups:
            self.create_subscription(
                String, f'traffic_light/{gid}/adaptive_factor',
                lambda msg, g=gid: self.on_adaptive_factor(msg, g), 10)

        self.emergency = False
        self.emergency_groups = None      # 急救走廊：列表内绿灯，列表外禁行
        self.forced_green_group = None
        self.adaptive_factors = {gid: 1.0 for gid in self.groups}
        self.dur_latch = {}                   # 本周期生效时长锁（见 _group_durations）
        self.phase_state = {}                 # gid -> {phase, start, dur} 相位状态机

        self.last_phase = {gid: None for gid in self.groups}
        self.last_ped_phase = {gid: None for gid in self.groups}
        self.last_force = {gid: -1e9 for gid in self.groups}

        self.stats = {}
        for gid in self.groups:
            self.stats[gid] = {
                'vehicle_count': 0, 'total_wait': 0.0, 'max_wait': 0.0,
                'total_vehicles': 0, 'total_wait_all': 0.0, 'cycle_count': 0,
                'cycle_start': 0.0, 'last_green_start': 0.0,
                'queue_length': 0,
            }

        self.vehicles_near_light = {gid: [] for gid in self.groups}
        self.veh_stop = {}
        self.create_subscription(ModelStates, '/model_states',
                                 self._on_model_states, 10)

        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.sim_start = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(float(self.get_parameter('publish_period').value),
                                       self.on_timer)
        self.get_logger().info(
            'TrafficLightManager: groups=%s preset=%s G=%s Y=%s R=%s pedestrian=1 (cfg=%s)' % (
                list(self.groups), self.preset_name, self.green, self.yellow, self.red, path))

    @staticmethod
    def _norm_groups(raw):
        groups = {}
        for gid, g in (raw or {}).items():
            g = g or {}
            groups[gid] = {
                'offset': float(g.get('offset', 0.0)),
                'complementary': bool(g.get('complementary', False)),
                'anti_phase_of': str(g.get('anti_phase_of', '')) or None,
                'green': g.get('green'), 'yellow': g.get('yellow'), 'red': g.get('red'),
                'stop_x': float(g.get('stop_x', 0.0)),
                'stop_y': float(g.get('stop_y', 0.0)),
                'queue_radius': float(g.get('queue_radius', 30.0)),
            }
        if not groups:
            groups = {'default': {'offset': 0.0, 'complementary': False,
                                  'green': None, 'yellow': None, 'red': None,
                                  'stop_x': 0.0, 'stop_y': 0.0, 'queue_radius': 30.0}}
        return groups

    def _apply_preset(self, name, log=True):
        p = (self.presets or {}).get(name)
        if p:
            self.green = float(p.get('green', self.green))
            self.yellow = float(p.get('yellow', self.yellow))
            self.red = float(p.get('red', self.red))
            self.preset_name = name
            if log:
                self.get_logger().info(f'preset -> {name} (G={self.green} Y={self.yellow} R={self.red})')
        elif log and name:
            self.get_logger().warn(f'未知预设 "{name}"，沿用当前配时')

    def _group_durations(self, g):
        gid = g.get('_gid', 'default')
        # 绿/黄期间锁住本周期时长：自适应系数或预设中途改变会移动模数边界，
        # 那会让灯态跳过黄灯（GREEN 直接变 RED）。新配时从下一个周期起生效。
        if self.last_phase.get(gid) in ('GREEN', 'YELLOW') and gid in self.dur_latch:
            return self.dur_latch[gid]
        factor = self.adaptive_factors.get(gid, 1.0)
        G = float(g['green']) if g.get('green') is not None else self.green
        Y = float(g['yellow']) if g.get('yellow') is not None else self.yellow
        R = float(g['red']) if g.get('red') is not None else self.red
        G = max(3.0, G * factor)
        R = max(3.0, R * factor)
        self.dur_latch[gid] = (G, Y, R)
        return G, Y, R

    def _effective_offset(self, g):
        if g.get('complementary'):
            _G, _Y, R = self._group_durations(g)
            return R
        apo = g.get('anti_phase_of')
        if apo and apo in self.groups and apo != g.get('_gid'):
            base = self.groups[apo]
            base['_gid'] = apo
            bG, bY, _bR = self._group_durations(base)
            return self._effective_offset(base) + bG + bY
        return float(g.get('offset', 0.0))

    def _seed_state(self, gid, now):
        """按 offset 把相位对齐到主时钟网格，作为状态机初值"""
        g = self.groups[gid]
        g['_gid'] = gid
        G, Y, R = self._group_durations(g)
        cycle = G + Y + R
        if cycle <= 0:
            return {'phase': 'RED', 'start': now, 'dur': 1.0}
        lt = (now + self._effective_offset(g)) % cycle
        if lt < G:
            return {'phase': 'GREEN', 'start': now - lt, 'dur': G}
        if lt < G + Y:
            return {'phase': 'YELLOW', 'start': now - (lt - G), 'dur': Y}
        return {'phase': 'RED', 'start': now - (lt - G - Y), 'dur': R}

    def phase_for(self, gid, now):
        """相位状态机：只在相位自然结束时切换，配时/预设/自适应改动从下一周期生效。

        纯模数模型在时长中途变化时会整体平移相位边界，出现 GREEN 直接跳 RED（漏黄灯）；
        这里改为按当前相位剩余时间推进，时钟回退（回放/重启）时重新按 offset 播种。
        """
        g = self.groups[gid]
        g['_gid'] = gid
        st = self.phase_state.get(gid)
        if st is None or now < st['start'] - 1e-6:
            self.dur_latch.pop(gid, None)   # 全新播种：用最新配时，不吃上一轮的时长锁
            st = self.phase_state[gid] = self._seed_state(gid, now)
            return st['phase']
        order = {'GREEN': 'YELLOW', 'YELLOW': 'RED', 'RED': 'GREEN'}
        for _ in range(64):
            if now < st['start'] + st['dur']:
                break
            G, Y, R = self._group_durations(g)
            st['start'] += st['dur']
            st['phase'] = order[st['phase']]
            st['dur'] = {'GREEN': G, 'YELLOW': Y, 'RED': R}[st['phase']]
        return st['phase']

    def _check_time_schedule(self):
        if not self.enable_time_schedule or not self.time_schedule:
            return
        now_wall = wall_time.localtime()
        current_minutes = now_wall.tm_hour * 60 + now_wall.tm_min
        for slot in self.time_schedule.get('slots', []):
            start_h, start_m = map(int, slot['start'].split(':'))
            end_h, end_m = map(int, slot['end'].split(':'))
            start_min = start_h * 60 + start_m
            end_min = end_h * 60 + end_m
            # 支持跨午夜时段（如 19:00-07:00）
            if start_min <= end_min:
                in_slot = start_min <= current_minutes < end_min
            else:
                in_slot = current_minutes >= start_min or current_minutes < end_min
            if in_slot:
                preset = slot.get('preset', 'default')
                if preset != self.preset_name:
                    self._apply_preset(preset)
                    self.get_logger().info(f'时段自动切换: {slot["start"]}-{slot["end"]} -> {preset}')
                break

    def on_timer(self):
        now = self.get_clock().now().nanoseconds / 1e9
        sim_elapsed = now - self.sim_start

        self._check_time_schedule()

        force_every = float(self.get_parameter('force_republish').value)
        corridor = self.emergency_groups
        for gid in self.groups:
            forced = (corridor is not None or self.emergency
                      or self.forced_green_group == gid)
            sched = self.phase_for(gid, sim_elapsed)   # 管制期间也推进，避免解除时跳变
            if corridor is not None:
                phase = 'GREEN' if gid in corridor else 'RED'
            elif self.emergency:
                phase = 'GREEN'
            elif self.forced_green_group == gid:
                phase = 'GREEN'
            else:
                phase = sched

            elapsed_in_red, red_dur = self._red_elapsed_dur(gid, sim_elapsed)
            g = self.groups[gid]
            g['_gid'] = gid
            ped_phase = self.pedestrian.get_phase(phase, elapsed_in_red, red_dur)
            if forced and phase == 'RED':
                ped_phase = PedestrianPhase.DONT_WALK   # 管制期间行人也不放行

            changed = (phase != self.last_phase[gid])
            ped_changed = (ped_phase != self.last_ped_phase[gid])
            if changed or (now - self.last_force[gid] >= force_every):
                msg = String()
                msg.data = phase
                self.pubs[gid].publish(msg)
                self.last_force[gid] = now

                if changed:
                    self.last_phase[gid] = phase
                    self.get_logger().info(f'{gid} -> {phase}')
                    self._update_stats(gid, phase, now)

                ped_msg = String()
                ped_msg.data = ped_phase
                self.ped_pubs[gid].publish(ped_msg)
                if ped_changed:
                    self.last_ped_phase[gid] = ped_phase
                    self.get_logger().info(f'{gid} pedestrian -> {ped_phase}')

                if HAS_V2X_MSGS and gid in self.state_pub:
                    self._publish_state_detail(gid, phase, ped_phase, sim_elapsed)

                if HAS_V2X_MSGS and gid in self.stats_pub:
                    self._publish_stats(gid, now)

        if self.default_group in self.groups:
            m = String()
            m.data = 'GREEN' if self.emergency else (self.last_phase.get(self.default_group) or 'RED')
            self.legacy_pub.publish(m)

    def _red_elapsed_dur(self, gid, sim_elapsed):
        """返回 (红灯已持续, 红灯时长)；非红灯或未建状态时按配时公式回推"""
        st = self.phase_state.get(gid)
        if st is not None:
            if st['phase'] != 'RED':
                return 0.0, st['dur']
            return max(0.0, sim_elapsed - st['start']), st['dur']
        g = self.groups[gid]
        g['_gid'] = gid
        G, Y, R = self._group_durations(g)
        cycle = G + Y + R
        if cycle <= 0:
            return 0.0, R
        lt = (sim_elapsed + self._effective_offset(g)) % cycle
        red_start = G + Y
        return (lt - red_start if lt >= red_start else 0.0), R

    def _elapsed_in_red(self, gid, sim_elapsed):
        return self._red_elapsed_dur(gid, sim_elapsed)[0]

    def _on_model_states(self, msg: ModelStates):
        import math
        for gid in self.groups:
            self.stats[gid]['queue_length'] = 0
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if not (name.startswith('v2x_') or name.startswith('smart_car_')
                    or name.startswith('car_')):
                continue
            x, y = pose.position.x, pose.position.y
            speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
            for gid in self.groups:
                g = self.groups[gid]
                stop_x, stop_y = g.get('stop_x', 0.0), g.get('stop_y', 0.0)
                queue_radius = g.get('queue_radius', 30.0)
                dist = math.hypot(x - stop_x, y - stop_y)
                if dist < queue_radius and speed < 0.5:
                    phase = self.last_phase.get(gid, 'RED')
                    if phase in ('RED', 'YELLOW'):
                        self.stats[gid]['queue_length'] += 1
                # 通行计数：路口区域内由停转走且当前绿灯，记一次通过
                was_stopped = self.veh_stop.get((gid, name), False)
                stopped_now = dist < queue_radius and speed < 0.5
                if was_stopped and not stopped_now and \
                        self.last_phase.get(gid) == 'GREEN':
                    self.stats[gid]['vehicle_count'] += 1
                    self.stats[gid]['total_vehicles'] += 1
                self.veh_stop[(gid, name)] = stopped_now

    def _update_stats(self, gid, phase, now):
        s = self.stats[gid]
        if phase == 'GREEN':
            s['last_green_start'] = now
            s['cycle_count'] += 1
        elif phase == 'RED' and s['last_green_start'] > 0:
            wait = now - s['last_green_start']
            s['total_wait_all'] += wait
            if wait > s['max_wait']:
                s['max_wait'] = wait

    def _publish_state_detail(self, gid, phase, ped_phase, sim_elapsed):
        g = self.groups[gid]
        g['_gid'] = gid
        G, Y, R = self._group_durations(g)
        st = self.phase_state.get(gid)
        if st is not None and st['phase'] == phase:
            remaining = st['start'] + st['dur'] - sim_elapsed
        else:
            cycle = G + Y + R
            lt = (sim_elapsed + self._effective_offset(g)) % cycle if cycle > 0 else 0.0
            remaining = {'GREEN': G - lt, 'YELLOW': G + Y - lt}.get(phase, cycle - lt)

        elapsed_red, red_dur = self._red_elapsed_dur(gid, sim_elapsed)
        walk, flash = self.pedestrian.sub_durations(red_dur)
        if ped_phase == 'WALK':
            ped_remaining = walk - elapsed_red
        elif ped_phase == 'FLASHING':
            ped_remaining = walk + flash - elapsed_red
        else:
            ped_remaining = max(0.0, red_dur - elapsed_red)

        msg = TrafficLightState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.light_id = gid
        msg.group_id = gid
        msg.vehicle_phase = phase
        msg.phase_remaining = max(0.0, remaining)
        msg.pedestrian_phase = ped_phase
        msg.pedestrian_remaining = max(0.0, ped_remaining)
        msg.green_duration = G
        msg.yellow_duration = Y
        msg.red_duration = R
        msg.walk_duration = walk
        msg.dont_walk_duration = flash
        if self.emergency:
            msg.control_mode = 'EMERGENCY'
        elif self.forced_green_group == gid:
            msg.control_mode = 'MANUAL'
        elif self.adaptive_factors.get(gid, 1.0) != 1.0:
            msg.control_mode = 'ADAPTIVE'
        else:
            msg.control_mode = 'FIXED'
        self.state_pub[gid].publish(msg)

    def _publish_stats(self, gid, now):
        s = self.stats[gid]
        msg = TrafficLightStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.group_id = gid
        msg.vehicle_count = s['vehicle_count']
        msg.avg_wait_time = s['total_wait_all'] / max(1, s['cycle_count'])
        msg.max_wait_time = s['max_wait']
        cycle_time = now - self.sim_start
        msg.throughput = s['total_vehicles'] / max(0.001, cycle_time / 60.0)
        msg.total_vehicles = s['total_vehicles']
        msg.total_wait_time = s['total_wait_all']
        msg.cycle_count = s['cycle_count']
        g = self.groups[gid]
        g['_gid'] = gid
        G, _, _ = self._group_durations(g)
        msg.current_green_duration = G
        msg.adaptation_factor = self.adaptive_factors.get(gid, 1.0)
        msg.queue_length = s.get('queue_length', 0)
        self.stats_pub[gid].publish(msg)

    def on_preset(self, msg):
        self._apply_preset(msg.data.strip())
        # 手动切换预设优先于时段自动调度，避免调度每秒把手动选择覆盖回去
        if self.enable_time_schedule:
            self.enable_time_schedule = False
            self.get_logger().info('手动预设切换：时段自动调度已暂停')
        self.last_force = {gid: -1e9 for gid in self.groups}

    def on_config(self, msg):
        """运行时手动配时：JSON {"group":"J2_ns"|"all","green":6,"yellow":2,"red":8,
        "offset":3,"complementary":false,"anti_phase_of":"J2_ns"}"""
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError) as e:
            self.get_logger().warn(f'配时指令解析失败: {e}')
            return
        tgt = str(d.get('group', 'all')).strip()
        gids = list(self.groups) if tgt in ('', 'all', '*') else [tgt]
        unknown = [g for g in gids if g not in self.groups]
        if unknown:
            self.get_logger().warn(f'未知信号组 {unknown}，可用: {list(self.groups)}')
            return
        for gid in gids:
            g = self.groups[gid]
            for k in ('green', 'yellow', 'red'):
                if d.get(k) is not None:
                    g[k] = float(d[k])
            if d.get('offset') is not None:
                g['offset'] = float(d['offset'])
                g['complementary'] = False
                g['anti_phase_of'] = None
            if 'complementary' in d:
                g['complementary'] = bool(d['complementary'])
            if 'anti_phase_of' in d:
                g['anti_phase_of'] = str(d['anti_phase_of']) or None
        self.last_force = {gid: -1e9 for gid in self.groups}
        ack = json.dumps({'group': tgt,
                          'applied': {k: v for k, v in d.items() if k != 'group'}},
                         ensure_ascii=False)
        self.config_ack_pub.publish(String(data=ack))
        self.get_logger().info(f'手动配时 -> {ack}')

    def on_forced_green_group(self, msg):
        gid = msg.data.strip()
        if gid in self.groups:
            self.forced_green_group = gid
            text = f'强制绿通 -> {gid}'
        else:
            self.forced_green_group = None
            text = '强制绿通已取消，恢复配时周期'
        self.last_force = {g: -1e9 for g in self.groups}
        self.get_logger().info(text)

    def on_emergency_groups(self, msg):
        """急救走廊：列表内信号组绿灯放行，其余禁行；空/none 恢复周期"""
        raw = msg.data.strip()
        if not raw or raw in ('none', '[]', 'off'):
            groups = None
        else:
            try:
                d = json.loads(raw)
                lst = d.get('groups', []) if isinstance(d, dict) else d
            except (ValueError, TypeError):
                lst = [s.strip() for s in raw.replace(';', ',').split(',') if s.strip()]
            groups = [g for g in lst if g in self.groups] or None
        self.emergency_groups = groups
        self.last_force = {g: -1e9 for g in self.groups}
        if groups:
            self.get_logger().warn(f'急救优先放行: 走廊={groups} 禁行={[g for g in self.groups if g not in groups]}')
        else:
            self.get_logger().info('急救优先放行关闭，恢复配时周期')

    def on_emergency(self, request, response):
        self.emergency = bool(request.data)
        self.last_force = {gid: -1e9 for gid in self.groups}
        response.success = True
        response.message = 'emergency all-green ON' if self.emergency else 'emergency OFF, resume schedule'
        self.get_logger().warn(response.message)
        return response

    def on_forced_green(self, request, response):
        if request.data:
            self.forced_green_group = self.default_group
            response.message = f'forced green for {self.default_group}'
        else:
            self.forced_green_group = None
            response.message = 'forced green OFF'
        self.last_force = {gid: -1e9 for gid in self.groups}
        response.success = True
        self.get_logger().info(response.message)
        return response

    def on_adaptive_factor(self, msg, gid):
        try:
            factor = float(msg.data)
            factor = max(0.5, min(2.0, factor))
            self.adaptive_factors[gid] = factor
            self.get_logger().info(f'{gid} adaptive_factor -> {factor}')
            self.last_force = {gid: -1e9 for gid in self.groups}
        except ValueError:
            pass

    def on_params(self, params):
        mapping = {'green_duration': 'green', 'yellow_duration': 'yellow', 'red_duration': 'red'}
        for p in params:
            if p.name in mapping:
                setattr(self, mapping[p.name], float(p.value))
                self.get_logger().info(f'{p.name} -> {p.value}')
        self.last_force = {gid: -1e9 for gid in self.groups}
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightManager()
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
