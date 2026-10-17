#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交通灯状态记录器 + 回放器 —— 完整录制信号灯时序变化。

功能：
  1) 录制：订阅 traffic_light/<group>/state_detail，记录每次灯态变化
  2) 录制内容：时间戳、机动车灯态、行人灯态、剩余时间、控制模式、自适应系数
  3) 录制车辆通行过程：按 vehicle_sample_period 采样 /model_states 中车辆位姿
  4) 保存为 JSON 文件（与车辆轨迹记录格式兼容）
  5) 回放：按时间序列重新发布灯态（含行人灯态），并通过 /set_entity_state
     复现车辆位姿运动，支持速度控制

JSON 结构：
  {
    "meta": {
      "start_time": 1234567890.123,
      "end_time": 1234567900.456,
      "groups": ["J1_ns", "J1_ew", "J2_ns"],
      "record_duration": 10.333,
      "event_count": 42
    },
    "events": [
      {
        "t": 0.0,
        "group": "J1_ns",
        "vehicle_phase": "GREEN",
        "pedestrian_phase": "DONT_WALK",
        "remaining": 9.8,
        "ped_remaining": 0.0,
        "control_mode": "FIXED",
        "adaptive_factor": 1.0
      },
      ...
    ]
  }

服务（std_srvs/Trigger）：
  Recorder:
    ~/start  开始记录
    ~/stop   停止记录
    ~/save   保存到文件
    ~/clear  清空缓冲区
  Player:
    ~/load   从文件加载
    ~/play   开始回放
    ~/pause  暂停/恢复回放
    ~/stop   停止回放
    ~/set_speed  设置回放速度（通过参数）
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from v2x_msgs.msg import TrafficLightState
    HAS_V2X_MSGS = True
except ImportError:
    HAS_V2X_MSGS = False


class TrafficLightRecorder(Node):
    def __init__(self):
        super().__init__('traffic_light_recorder')

        self.declare_parameter('output_file', os.path.expanduser('~/traffic_light_log.json'))
        self.declare_parameter('groups', ['J1_ns', 'J1_ew', 'J2_ns'])
        self.declare_parameter('record_detail', True)
        self.declare_parameter('record_vehicles', True)
        self.declare_parameter('vehicle_sample_period', 0.5)

        self.output_file = self.get_parameter('output_file').value
        self.groups = self.get_parameter('groups').value
        self.record_detail = bool(self.get_parameter('record_detail').value)
        self.record_vehicles = bool(self.get_parameter('record_vehicles').value)
        self.vehicle_period = float(self.get_parameter('vehicle_sample_period').value)

        self.recording = False
        self.events = []
        self.t0 = None
        self.last_veh = {}
        self.vehicles = set()

        self.subs = {}
        for gid in self.groups:
            self.subs[gid] = self.create_subscription(
                String, f'traffic_light/{gid}/state',
                lambda msg, g=gid: self.on_state(msg, g), 10)
            self.subs[f'{gid}_ped'] = self.create_subscription(
                String, f'traffic_light/{gid}/pedestrian',
                lambda msg, g=gid: self.on_pedestrian(msg, g), 10)

            if HAS_V2X_MSGS and self.record_detail:
                self.subs[f'{gid}_detail'] = self.create_subscription(
                    TrafficLightState, f'traffic_light/{gid}/state_detail',
                    lambda msg, g=gid: self.on_state_detail(msg, g), 10)

        if self.record_vehicles:
            self.subs['model_states'] = self.create_subscription(
                ModelStates, '/model_states', self.on_model_states, 10)

        self.create_service(Trigger, '~/start', self.svc_start)
        self.create_service(Trigger, '~/stop', self.svc_stop)
        self.create_service(Trigger, '~/save', self.svc_save)
        self.create_service(Trigger, '~/clear', self.svc_clear)

        self.get_logger().info(
            f'traffic_light_recorder: output={self.output_file} groups={self.groups}')

    def on_state(self, msg, group):
        if not self.recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.events.append({
            't': round(now - self.t0, 4),
            'type': 'light',
            'group': group,
            'vehicle_phase': msg.data,
            'pedestrian_phase': '',
            'remaining': 0.0,
            'ped_remaining': 0.0,
            'control_mode': '',
            'adaptive_factor': 1.0,
        })

    def on_pedestrian(self, msg, group):
        if not self.recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.events.append({
            't': round(now - self.t0, 4),
            'type': 'light',
            'group': group,
            'vehicle_phase': '',
            'pedestrian_phase': msg.data,
            'remaining': 0.0,
            'ped_remaining': 0.0,
            'control_mode': '',
            'adaptive_factor': 1.0,
        })

    def on_state_detail(self, msg, group):
        if not self.recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        self.events.append({
            't': round(now - self.t0, 4),
            'type': 'light',
            'group': group,
            'vehicle_phase': msg.vehicle_phase,
            'pedestrian_phase': msg.pedestrian_phase,
            'remaining': round(msg.phase_remaining, 2),
            'ped_remaining': round(msg.pedestrian_remaining, 2),
            'control_mode': msg.control_mode,
            'adaptive_factor': round(msg.adaptation_factor if hasattr(msg, 'adaptation_factor') else 1.0, 3),
        })

    def on_model_states(self, msg):
        if not self.recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        for name, pose in zip(msg.name, msg.pose):
            if not name.startswith(('car_', 'v2x_', 'smart_car_')):
                continue
            if now - self.last_veh.get(name, -1e9) < self.vehicle_period:
                continue
            self.last_veh[name] = now
            self.vehicles.add(name)
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.events.append({
                't': round(now - self.t0, 4),
                'type': 'vehicle',
                'group': '',
                'name': name,
                'x': round(pose.position.x, 3),
                'y': round(pose.position.y, 3),
                'z': round(pose.position.z, 3),
                'yaw': round(yaw, 3),
            })

    def svc_start(self, _req, resp):
        self.recording = True
        if self.t0 is None:
            self.t0 = self.get_clock().now().nanoseconds / 1e9
        resp.success = True
        resp.message = 'recording started'
        self.get_logger().info('recording started')
        return resp

    def svc_stop(self, _req, resp):
        self.recording = False
        resp.success = True
        resp.message = f'recording stopped, {len(self.events)} events captured'
        self.get_logger().info(resp.message)
        return resp

    def svc_save(self, _req, resp):
        data = {
            'meta': {
                'start_time': self.t0,
                'end_time': self.get_clock().now().nanoseconds / 1e9,
                'groups': self.groups,
                'record_duration': round((self.get_clock().now().nanoseconds / 1e9) - (self.t0 or 0), 3),
                'event_count': len(self.events),
                'vehicles': sorted(self.vehicles),
            },
            'events': self.events,
        }
        try:
            with open(self.output_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            resp.success = True
            resp.message = f'saved {len(self.events)} events to {self.output_file}'
            self.get_logger().info(resp.message)
        except Exception as e:
            resp.success = False
            resp.message = f'save failed: {e}'
            self.get_logger().error(resp.message)
        return resp

    def svc_clear(self, _req, resp):
        self.events.clear()
        self.t0 = None
        resp.success = True
        resp.message = 'buffer cleared'
        return resp


class TrafficLightPlayer(Node):
    """交通灯回放节点 —— 完整复现信号灯时序变化（含行人灯态）"""

    def __init__(self):
        super().__init__('traffic_light_player')

        self.declare_parameter('input_file', os.path.expanduser('~/traffic_light_log.json'))
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('loop', False)

        self.input_file = self.get_parameter('input_file').value
        self.playback_speed = float(self.get_parameter('playback_speed').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.pubs = {}
        self.ped_pubs = {}
        self.playing = False
        self.paused = False
        self.events = []
        self.play_index = 0
        self.play_start_time = None
        self.meta = {}
        self.set_state_cli = None

        self.create_service(Trigger, '~/load', self.svc_load)
        self.create_service(Trigger, '~/play', self.svc_play)
        self.create_service(Trigger, '~/pause', self.svc_pause)
        self.create_service(Trigger, '~/stop', self.svc_stop)

        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(f'traffic_light_player: input={self.input_file} speed={self.playback_speed}x')

    def svc_load(self, _req, resp):
        try:
            with open(self.input_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.events = data.get('events', [])
            self.meta = data.get('meta', {})
            groups = self.meta.get('groups', [])
            for gid in groups:
                if gid not in self.pubs:
                    self.pubs[gid] = self.create_publisher(
                        String, f'traffic_light/{gid}/state', 10)
                    self.ped_pubs[gid] = self.create_publisher(
                        String, f'traffic_light/{gid}/pedestrian', 10)
            resp.success = True
            resp.message = f'loaded {len(self.events)} events from {self.input_file}'
            self.get_logger().info(resp.message)
        except Exception as e:
            resp.success = False
            resp.message = f'load failed: {e}'
        return resp

    def svc_play(self, _req, resp):
        if not self.events:
            resp.success = False
            resp.message = 'no events loaded'
            return resp
        self.playing = True
        self.paused = False
        self.play_index = 0
        self.play_start_time = self.get_clock().now().nanoseconds / 1e9
        resp.success = True
        resp.message = f'playback started, {len(self.events)} events at {self.playback_speed}x'
        self.get_logger().info(resp.message)
        return resp

    def svc_pause(self, _req, resp):
        self.paused = not self.paused
        if not self.paused and self.playing:
            self.play_start_time = self.get_clock().now().nanoseconds / 1e9 - (
                self.play_index / max(1, len(self.events)) *
                (self.events[-1]['t'] if self.events else 0) / self.playback_speed
            )
        resp.success = True
        resp.message = 'paused' if self.paused else 'resumed'
        return resp

    def svc_stop(self, _req, resp):
        self.playing = False
        self.play_index = 0
        resp.success = True
        resp.message = 'playback stopped'
        return resp

    def _replay_vehicle(self, event):
        if self.set_state_cli is None:
            self.set_state_cli = self.create_client(SetEntityState, '/set_entity_state')
        if not self.set_state_cli.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state.name = event['name']
        req.state.pose.position.x = event['x']
        req.state.pose.position.y = event['y']
        req.state.pose.position.z = event['z']
        half = event['yaw'] / 2.0
        req.state.pose.orientation.z = math.sin(half)
        req.state.pose.orientation.w = math.cos(half)
        self.set_state_cli.call_async(req)

    def on_timer(self):
        if not self.playing or self.paused:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = (now - self.play_start_time) * self.playback_speed

        while self.play_index < len(self.events):
            event = self.events[self.play_index]
            if event['t'] > elapsed:
                break
            group = event['group']
            if event.get('type') == 'vehicle':
                self._replay_vehicle(event)
                self.play_index += 1
                continue
            if event.get('vehicle_phase') and group in self.pubs:
                msg = String()
                msg.data = event['vehicle_phase']
                self.pubs[group].publish(msg)
            if event.get('pedestrian_phase') and group in self.ped_pubs:
                ped_msg = String()
                ped_msg.data = event['pedestrian_phase']
                self.ped_pubs[group].publish(ped_msg)
            self.play_index += 1

        if self.play_index >= len(self.events):
            if self.loop:
                self.play_index = 0
                self.play_start_time = self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info('playback looped')
            else:
                self.playing = False
                self.get_logger().info('playback completed')


def main_recorder(args=None):
    rclpy.init(args=args)
    node = TrafficLightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_player(args=None):
    rclpy.init(args=args)
    node = TrafficLightPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'player':
        main_player()
    else:
        main_recorder()
