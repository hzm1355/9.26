#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket 桥接节点 —— 将 ROS2 话题转发到 WebSocket，供 3D Web 可视化使用。

功能：
  1) 订阅交通灯状态、车辆位姿等话题
  2) 通过 WebSocket 广播到 Web 客户端
  3) 接收 Web 客户端的控制命令（切换预设、紧急模式等）

话题：
  订阅：
    traffic_light/<group>/state (std_msgs/String)
    /model_states (gazebo_msgs/ModelStates)
  发布：
    traffic_light/preset (std_msgs/String) —— 来自 Web 的预设切换
  服务：
    /traffic_light_manager/emergency (std_srvs/SetBool) —— 来自 Web 的紧急模式

WebSocket 消息格式（JSON）：
  {"type": "traffic_light", "group": "J1_ns", "phase": "GREEN", "remaining": 8.5}
  {"type": "vehicle", "name": "smart_car_1", "x": 0, "y": -10, "yaw": 1.57, "speed": 2.5}
  {"type": "command", "action": "preset", "value": "peak"}
  {"type": "command", "action": "emergency", "value": true}
"""
import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

try:
    from v2x_msgs.msg import TrafficLightState
    HAS_V2X_MSGS = True
except ImportError:
    HAS_V2X_MSGS = False

try:
    import websockets
    HAS_WEBSOCKETS = True
except ImportError:
    HAS_WEBSOCKETS = False


class WebSocketBridge(Node):
    def __init__(self):
        super().__init__('websocket_bridge')

        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('groups', ['J1_ns', 'J1_ew', 'J2_ns'])
        self.declare_parameter('vehicle_regex', '^(smart_car_|tb3_)')

        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = int(self.get_parameter('ws_port').value)
        self.groups = self.get_parameter('groups').value
        self.vehicle_regex = self.get_parameter('vehicle_regex').value

        import re
        self.vehicle_re = re.compile(self.vehicle_regex)

        self.clients = set()
        self.latest_states = {}
        self.latest_vehicles = {}

        for gid in self.groups:
            self.create_subscription(
                String, f'traffic_light/{gid}/state',
                lambda msg, g=gid: self.on_light_state(msg, g), 10)
            if HAS_V2X_MSGS:
                self.create_subscription(
                    TrafficLightState, f'traffic_light/{gid}/state_detail',
                    lambda msg, g=gid: self.on_light_state_detail(msg, g), 10)

        self.create_subscription(
            ModelStates, '/model_states', self.on_model_states, 10)

        self.preset_pub = self.create_publisher(String, 'traffic_light/preset', 10)

        self.get_logger().info(
            f'WebSocketBridge: ws://{self.ws_host}:{self.ws_port} groups={self.groups}')

        if HAS_WEBSOCKETS:
            self.ws_thread = threading.Thread(target=self.run_ws_server, daemon=True)
            self.ws_thread.start()
        else:
            self.get_logger().warn('websockets not installed, WebSocket bridge disabled')

    def on_light_state(self, msg, group):
        self.latest_states[group] = {
            'type': 'traffic_light',
            'group': group,
            'phase': msg.data,
            'remaining': 0.0,
            'pedestrian_phase': '',
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        self.broadcast(self.latest_states[group])

    def on_light_state_detail(self, msg, group):
        if group not in self.latest_states:
            self.latest_states[group] = {}
        self.latest_states[group].update({
            'type': 'traffic_light',
            'group': group,
            'phase': msg.vehicle_phase,
            'remaining': msg.phase_remaining,
            'pedestrian_phase': msg.pedestrian_phase,
            'control_mode': msg.control_mode,
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        })
        self.broadcast(self.latest_states[group])

    def on_model_states(self, msg):
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if not self.vehicle_re.match(name):
                continue
            import math
            q = pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
            self.latest_vehicles[name] = {
                'type': 'vehicle',
                'name': name,
                'x': round(pose.position.x, 2),
                'y': round(pose.position.y, 2),
                'z': round(pose.position.z, 2),
                'yaw': round(yaw, 2),
                'speed': round(speed, 2),
            }
        vehicles_msg = {
            'type': 'vehicles',
            'data': list(self.latest_vehicles.values()),
        }
        self.broadcast(vehicles_msg)

    def broadcast(self, message):
        if not self.clients or not HAS_WEBSOCKETS:
            return
        data = json.dumps(message)
        asyncio.run_coroutine_threadsafe(self._broadcast(data), self.loop)

    async def _broadcast(self, data):
        if self.clients:
            await asyncio.gather(*[client.send(data) for client in self.clients])

    async def handler(self, websocket):
        self.clients.add(websocket)
        self.get_logger().info(f'WebSocket client connected, total={len(self.clients)}')
        try:
            async for message in websocket:
                await self.handle_command(message, websocket)
        except Exception:
            pass
        finally:
            self.clients.discard(websocket)
            self.get_logger().info(f'WebSocket client disconnected, total={len(self.clients)}')

    async def handle_command(self, message, websocket):
        try:
            cmd = json.loads(message)
            if cmd.get('type') == 'command':
                action = cmd.get('action')
                value = cmd.get('value')
                if action == 'preset':
                    msg = String()
                    msg.data = str(value)
                    self.preset_pub.publish(msg)
                    self.get_logger().info(f'Web command: preset -> {value}')
                elif action == 'emergency':
                    self.get_logger().info(f'Web command: emergency -> {value}')
        except Exception as e:
            self.get_logger().error(f'handle_command error: {e}')

    async def ws_server(self):
        async with websockets.serve(self.handler, self.ws_host, self.ws_port):
            self.get_logger().info(f'WebSocket server started on ws://{self.ws_host}:{self.ws_port}')
            await asyncio.Future()

    def run_ws_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ws_server())


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridge()
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
