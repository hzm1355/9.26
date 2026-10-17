#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 ↔ WebSocket 桥接服务（运行在 WSL2 中）。

功能：
  1. 订阅 /model_states 获取所有车辆位姿
  2. 订阅各车传感器话题（scan/camera/imu/odom）
  3. 提供 WebSocket API 供 Windows GUI 查询/控制
  4. 转发故障注入指令到各车 fault_sim 插件

API：
  → {type: "subscribe", topics: [...]}
  ← {type: "state", vehicles: {...}, regions: [...], t: ...}
  → {type: "command", cmd: "cmd_vel", vehicle: "smart_car_1", linear: 0.5, angular: 0}
  → {type: "command", cmd: "fault", vehicle: "smart_car_1", fault_type: "lidar_off"}
  → {type: "command", cmd: "record", action: "start"|"stop"|"save"}
  → {type: "command", cmd: "playback", action: "load"|"play"|"stop", file: "..."}
"""
import asyncio
import json
import math
import os
import subprocess
import sys
import time
import threading
from collections import defaultdict
from typing import Dict, Set

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("[bridge] rclpy 不可用，使用 CLI 回退模式")

try:
    import websockets
    import websockets.server
except ImportError:
    print("[bridge] 需要 websockets: pip3 install websockets")
    sys.exit(1)


# ── 车辆状态缓存 ──────────────────────────────────────────────
class VehicleState:
    def __init__(self, name: str):
        self.name = name
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.scan_min_range = 0.0
        self.scan_ranges = []
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.last_update = 0.0

    def to_dict(self):
        return {
            'name': self.name,
            'x': round(self.x, 3),
            'y': round(self.y, 3),
            'z': round(self.z, 3),
            'yaw': round(self.yaw, 3),
            'vx': round(self.vx, 3),
            'vy': round(self.vy, 3),
            'scan_min_range': round(self.scan_min_range, 3),
            'scan_sample_count': len(self.scan_ranges),
            'imu_yaw': round(self.imu_yaw, 3),
            'odom_x': round(self.odom_x, 3),
            'odom_y': round(self.odom_y, 3),
            'odom_yaw': round(self.odom_yaw, 3),
            'odom_vx': round(self.odom_vx, 3),
            'last_update': round(self.last_update, 2),
        }


class BridgeState:
    def __init__(self):
        self.vehicles: Dict[str, VehicleState] = {}
        self.region_events = []
        self.lock = threading.Lock()
        self.sim_time = 0.0
        self.is_recording = False
        self.is_playing = False

    def get_snapshot(self) -> dict:
        with self.lock:
            return {
                'type': 'state',
                't': time.time(),
                'sim_time': round(self.sim_time, 2),
                'vehicles': {n: v.to_dict() for n, v in self.vehicles.items()},
                'region_events': self.region_events[-50:],
                'is_recording': self.is_recording,
                'is_playing': self.is_playing,
            }


STATE = BridgeState()
CLIENTS: Set = set()


# ── ROS2 节点 ─────────────────────────────────────────────────
if HAS_ROS2:
    class BridgeNode(Node):
        def __init__(self):
            super().__init__('gui_bridge_node')
            qos_states = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    history=HistoryPolicy.KEEP_LAST,
                                    durability=QoSProfile(durability=10).durability)

            self.create_subscription(ModelStates, '/model_states',
                                     self.on_model_states, 10)

            self.cmd_vel_pubs = {}
            self.get_logger().info('桥接节点启动')

        def on_model_states(self, msg: ModelStates):
            now = time.time()
            with STATE.lock:
                for i, name in enumerate(msg.name):
                    if not (name.startswith('smart_car_') or name.startswith('tb3_')):
                        continue
                    if name not in STATE.vehicles:
                        STATE.vehicles[name] = VehicleState(name)
                        self._subscribe_vehicle_sensors(name)
                    v = STATE.vehicles[name]
                    p = msg.pose[i]
                    t = msg.twist[i]
                    v.x = p.position.x
                    v.y = p.position.y
                    v.z = p.position.z
                    q = p.orientation
                    v.yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                       1 - 2*(q.y*q.y + q.z*q.z))
                    v.vx = t.linear.x
                    v.vy = t.linear.y
                    v.last_update = now
                STATE.sim_time = now

        def _subscribe_vehicle_sensors(self, name: str):
            ns = f'/{name}'
            self.create_subscription(LaserScan, f'{ns}/scan',
                                     lambda m, n=name: self.on_scan(m, n), 10)
            self.create_subscription(Imu, f'{ns}/imu/data',
                                     lambda m, n=name: self.on_imu(m, n), 10)
            self.create_subscription(Odometry, f'{ns}/odom',
                                     lambda m, n=name: self.on_odom(m, n), 10)
            self.cmd_vel_pubs[name] = self.create_publisher(Twist, f'{ns}/cmd_vel', 10)
            self.get_logger().info(f'  订阅 {name} 传感器')

        def on_scan(self, msg: LaserScan, name: str):
            with STATE.lock:
                if name in STATE.vehicles:
                    v = STATE.vehicles[name]
                    valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
                    v.scan_min_range = min(valid) if valid else msg.range_max
                    v.scan_ranges = list(msg.ranges[::max(1, len(msg.ranges)//36)])

        def on_imu(self, msg: Imu, name: str):
            with STATE.lock:
                if name in STATE.vehicles:
                    v = STATE.vehicles[name]
                    q = msg.orientation
                    v.imu_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                           1 - 2*(q.y*q.y + q.z*q.z))

        def on_odom(self, msg: Odometry, name: str):
            with STATE.lock:
                if name in STATE.vehicles:
                    v = STATE.vehicles[name]
                    v.odom_x = msg.pose.pose.position.x
                    v.odom_y = msg.pose.pose.position.y
                    q = msg.pose.pose.orientation
                    v.odom_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                            1 - 2*(q.y*q.y + q.z*q.z))
                    v.odom_vx = msg.twist.twist.linear.x

        def publish_cmd_vel(self, vehicle: str, linear: float, angular: float):
            if vehicle in self.cmd_vel_pubs:
                tw = Twist()
                tw.linear.x = linear
                tw.angular.z = angular
                self.cmd_vel_pubs[vehicle].publish(tw)
                return True
            return False

    BRIDGE_NODE = None
else:
    BRIDGE_NODE = None


# ── CLI 回退（无 rclpy 时用 ros2 CLI） ───────────────────────
def cli_get_model_states() -> dict:
    try:
        r = subprocess.run(
            ['ros2', 'topic', 'echo', '--once', '/model_states',
             '--field', 'name,pose,twist'],
            capture_output=True, text=True, timeout=3)
        return parse_model_states_cli(r.stdout)
    except Exception:
        return {}


def parse_model_states_cli(text: str) -> dict:
    vehicles = {}
    lines = text.strip().split('\n')
    current = None
    for line in lines:
        line = line.strip()
        if line.startswith('- '):
            name = line[2:].strip().strip(\"'\\\"\")
            if name.startswith('smart_car_') or name.startswith('tb3_'):
                current = name
                vehicles[name] = {'x': 0, 'y': 0, 'yaw': 0, 'vx': 0, 'vy': 0}
    return vehicles


def cli_send_cmd_vel(vehicle: str, linear: float, angular: float):
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        f'/{vehicle}/cmd_vel', 'geometry_msgs/msg/Twist',
        f'{{linear: {{x: {linear}}}, angular: {{z: {angular}}}}}'
    ], capture_output=True, timeout=3)


def cli_send_fault(vehicle: str, fault_type: str):
    srv_map = {
        'lidar_off': ('lidar', False),
        'lidar_on': ('lidar', True),
        'camera_off': ('camera', False),
        'camera_on': ('camera', True),
        'imu_drift': ('imu_drift', True),
        'odom_slip': ('odom_slip', True),
        'reset': ('reset', True),
    }
    if fault_type in srv_map:
        sensor, enabled = srv_map[fault_type]
        subprocess.run([
            'ros2', 'service', 'call',
            f'/{vehicle}/fault_sim',
            'v2x_msgs/srv/FaultCommand',
            f'{{sensor: "{sensor}", enabled: {str(enabled).lower()}}}'
        ], capture_output=True, timeout=3)


# ── WebSocket 服务 ────────────────────────────────────────────
async def handler(ws):
    CLIENTS.add(ws)
    addr = ws.remote_address
    print(f'[bridge] GUI 连接: {addr}')
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            await handle_command(msg, ws)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CLIENTS.discard(ws)
        print(f'[bridge] GUI 断开: {addr}')


async def handle_command(msg: dict, ws):
    cmd = msg.get('type', '')

    if cmd == 'ping':
        await ws.send(json.dumps({'type': 'pong', 't': time.time()}))

    elif cmd == 'get_state':
        await ws.send(json.dumps(STATE.get_snapshot()))

    elif cmd == 'subscribe':
        pass

    elif cmd == 'cmd_vel':
        vehicle = msg.get('vehicle', '')
        linear = float(msg.get('linear', 0))
        angular = float(msg.get('angular', 0))
        if BRIDGE_NODE:
            BRIDGE_NODE.publish_cmd_vel(vehicle, linear, angular)
        else:
            cli_send_cmd_vel(vehicle, linear, angular)
        await ws.send(json.dumps({'type': 'ack', 'cmd': 'cmd_vel',
                                  'vehicle': vehicle}))

    elif cmd == 'fault':
        vehicle = msg.get('vehicle', '')
        fault_type = msg.get('fault_type', '')
        cli_send_fault(vehicle, fault_type)
        await ws.send(json.dumps({'type': 'ack', 'cmd': 'fault',
                                  'vehicle': vehicle, 'fault': fault_type}))

    elif cmd == 'record':
        action = msg.get('action', 'start')
        subprocess.run(['ros2', 'service', 'call',
                        '/trajectory_recorder/start',
                        'std_srvs/srv/Trigger'],
                       capture_output=True, timeout=3)
        STATE.is_recording = (action == 'start')
        await ws.send(json.dumps({'type': 'ack', 'cmd': 'record',
                                  'action': action}))

    elif cmd == 'playback':
        action = msg.get('action', 'load')
        if action == 'load':
            f = msg.get('file', '~/smart_car_trajectory.json')
            subprocess.run(['ros2', 'service', 'call',
                            '/trajectory_player/load',
                            'std_srvs/srv/Trigger',
                            f'{{file: "{f}"}}'],
                           capture_output=True, timeout=3)
        elif action == 'play':
            subprocess.run(['ros2', 'service', 'call',
                            '/trajectory_player/play',
                            'std_srvs/srv/Trigger'],
                           capture_output=True, timeout=3)
            STATE.is_playing = True
        elif action == 'stop':
            subprocess.run(['ros2', 'service', 'call',
                            '/trajectory_player/stop',
                            'std_srvs/srv/Trigger'],
                           capture_output=True, timeout=3)
            STATE.is_playing = False
        await ws.send(json.dumps({'type': 'ack', 'cmd': 'playback',
                                  'action': action}))

    elif cmd == 'get_trajectory':
        fpath = os.path.expanduser(msg.get('file', '~/smart_car_trajectory.json'))
        data = {'type': 'trajectory', 'data': None, 'error': None}
        try:
            with open(fpath) as f:
                data['data'] = json.load(f)
        except Exception as e:
            data['error'] = str(e)
        await ws.send(json.dumps(data))

    elif cmd == 'get_config':
        cfg_path = msg.get('file', '')
        data = {'type': 'config', 'data': None}
        try:
            with open(cfg_path) as f:
                data['data'] = yaml.safe_load(f)
        except Exception:
            pass
        await ws.send(json.dumps(data))

    elif cmd == 'save_config':
        cfg_path = msg.get('file', '')
        cfg_data = msg.get('data', {})
        try:
            import yaml
            with open(cfg_path, 'w') as f:
                yaml.dump(cfg_data, f, allow_unicode=True, default_flow_style=False)
            await ws.send(json.dumps({'type': 'ack', 'cmd': 'save_config', 'ok': True}))
        except Exception as e:
            await ws.send(json.dumps({'type': 'ack', 'cmd': 'save_config',
                                      'ok': False, 'error': str(e)}))


async def broadcast_loop():
    while True:
        if CLIENTS:
            snapshot = STATE.get_snapshot()
            data = json.dumps(snapshot)
            dead = set()
            for ws in CLIENTS:
                try:
                    await ws.send(data)
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
            CLIENTS -= dead
        await asyncio.sleep(0.1)


async def cli_poll_loop():
    if HAS_ROS2:
        return
    while True:
        try:
            vehicles = cli_get_model_states()
            with STATE.lock:
                now = time.time()
                for name, data in vehicles.items():
                    if name not in STATE.vehicles:
                        STATE.vehicles[name] = VehicleState(name)
                    v = STATE.vehicles[name]
                    v.x = data.get('x', 0)
                    v.y = data.get('y', 0)
                    v.yaw = data.get('yaw', 0)
                    v.vx = data.get('vx', 0)
                    v.vy = data.get('vy', 0)
                    v.last_update = now
        except Exception:
            pass
        await asyncio.sleep(1.0)


async def main():
    host = '0.0.0.0'
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8765

    print(f'[bridge] 启动 WebSocket 服务 ws://{host}:{port}')
    print(f'[bridge] ROS2 原生模式: {"是" if HAS_ROS2 else "否（CLI 回退）"}')

    if HAS_ROS2:
        rclpy.init()
        global BRIDGE_NODE
        BRIDGE_NODE = BridgeNode()
        spin_thread = threading.Thread(target=rclpy.spin, args=(BRIDGE_NODE,),
                                       daemon=True)
        spin_thread.start()

    asyncio.create_task(broadcast_loop())
    asyncio.create_task(cli_poll_loop())

    async with websockets.serve(handler, host, port):
        print(f'[bridge] 等待 GUI 连接...')
        await asyncio.Future()


if __name__ == '__main__':
    try:
        import yaml
    except ImportError:
        yaml = None
    asyncio.run(main())
