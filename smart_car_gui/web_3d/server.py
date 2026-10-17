#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D Web 可视化服务器（简化版）。

使用方法：
  1. 安装依赖: pip3 install -r requirements.txt
  2. 运行: python3 server.py [端口号]
  3. 浏览器打开: http://localhost:8080
"""
import asyncio
import json
import os
import sys
import time
import math
import threading
from typing import Set

try:
    import rclpy
    from rclpy.node import Node
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Twist
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("[web3d] rclpy 不可用，使用模拟数据")

try:
    import websockets
except ImportError:
    print("[web3d] 错误: 需要安装 websockets")
    print("[web3d] 请运行: pip3 install websockets")
    sys.exit(1)

try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    HAS_AIOHTTP = False
    print("[web3d] 警告: aiohttp 不可用，使用简单 HTTP 服务器")


# ── 模拟数据（无 ROS2 时） ────────────────────────────────────
class MockSimulator:
    def __init__(self):
        self.vehicles = {
            'smart_car_1': {'x': 0, 'y': -18, 'yaw': 1.57, 'vx': 0.5},
            'smart_car_2': {'x': -15, 'y': -18, 'yaw': 0, 'vx': 0.3},
            'smart_car_3': {'x': 15, 'y': -18, 'yaw': 3.14, 'vx': 0.4},
            'tb3_1': {'x': -8, 'y': -18, 'yaw': 0.5, 'vx': 0.6},
            'tb3_2': {'x': 8, 'y': -18, 'yaw': -0.5, 'vx': 0.35},
        }
        self.time_step = 0

    def update(self):
        self.time_step += 1
        for name, v in self.vehicles.items():
            v['x'] += v['vx'] * 0.1 * math.cos(v['yaw'])
            v['y'] += v['vx'] * 0.1 * math.sin(v['yaw'])
            v['yaw'] += 0.01 * math.sin(self.time_step * 0.01)

    def get_state(self):
        return {
            'type': 'state',
            't': time.time(),
            'sim_time': self.time_step * 0.1,
            'vehicles': self.vehicles.copy(),
        }


# ── ROS2 桥接节点 ─────────────────────────────────────────────
if HAS_ROS2:
    class BridgeNode(Node):
        def __init__(self):
            super().__init__('web3d_bridge')
            self.vehicles = {}
            self.create_subscription(ModelStates, '/model_states',
                                     self.on_model_states, 10)
            self.cmd_vel_pubs = {}
            self.get_logger().info('Web3D 桥接节点启动')

        def on_model_states(self, msg: ModelStates):
            for i, name in enumerate(msg.name):
                if not (name.startswith('smart_car_') or name.startswith('tb3_')):
                    continue
                p = msg.pose[i]
                t = msg.twist[i]
                self.vehicles[name] = {
                    'x': p.position.x,
                    'y': p.position.y,
                    'z': p.position.z,
                    'yaw': math.atan2(
                        2*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y),
                        1 - 2*(p.orientation.y**2 + p.orientation.z**2)
                    ),
                    'vx': t.linear.x,
                    'vy': t.linear.y,
                }
                if name not in self.cmd_vel_pubs:
                    self.cmd_vel_pubs[name] = self.create_publisher(
                        Twist, f'/{name}/cmd_vel', 10)

        def get_state(self):
            return {
                'type': 'state',
                't': time.time(),
                'sim_time': self.get_clock().now().nanoseconds / 1e9,
                'vehicles': self.vehicles.copy(),
            }

        def publish_cmd_vel(self, vehicle, linear, angular):
            if vehicle in self.cmd_vel_pubs:
                tw = Twist()
                tw.linear.x = linear
                tw.angular.z = angular
                self.cmd_vel_pubs[vehicle].publish(tw)

    BRIDGE_NODE = None
else:
    BRIDGE_NODE = None
    MOCK_SIM = MockSimulator()


# ── WebSocket 服务 ────────────────────────────────────────────
CLIENTS: Set = set()

async def ws_handler(websocket, path=None):
    """处理 WebSocket 连接"""
    CLIENTS.add(websocket)
    remote = websocket.remote_address if hasattr(websocket, 'remote_address') else 'unknown'
    print(f'[web3d] 客户端连接: {remote}')
    try:
        async for message in websocket:
            try:
                msg = json.loads(message)
                await handle_command(msg, websocket)
            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CLIENTS.discard(websocket)
        print(f'[web3d] 客户端断开: {remote}')

async def handle_command(msg: dict, websocket):
    """处理客户端命令"""
    cmd = msg.get('type', '')
    if cmd == 'cmd_vel':
        vehicle = msg.get('vehicle', '')
        linear = float(msg.get('linear', 0))
        angular = float(msg.get('angular', 0))
        if BRIDGE_NODE:
            BRIDGE_NODE.publish_cmd_vel(vehicle, linear, angular)
        await websocket.send(json.dumps({'type': 'ack', 'cmd': 'cmd_vel'}))

async def broadcast_loop():
    """广播状态到所有客户端"""
    while True:
        if CLIENTS:
            if BRIDGE_NODE:
                state = BRIDGE_NODE.get_state()
            else:
                MOCK_SIM.update()
                state = MOCK_SIM.get_state()
            data = json.dumps(state)
            for ws in CLIENTS.copy():
                try:
                    await ws.send(data)
                except:
                    CLIENTS.discard(ws)
        await asyncio.sleep(0.05)  # 20Hz


# ── HTTP 服务 ─────────────────────────────────────────────────
if HAS_AIOHTTP:
    async def http_handler(request):
        """aiohttp HTTP 处理器"""
        path = request.path
        if path == '/':
            path = '/index.html'
        
        file_path = os.path.join(os.path.dirname(__file__), path.lstrip('/'))
        if os.path.isfile(file_path):
            with open(file_path, 'rb') as f:
                content = f.read()
            content_type = 'text/html' if path.endswith('.html') else 'application/octet-stream'
            return web.Response(body=content, content_type=content_type)
        return web.Response(text='Not Found', status=404)

    async def start_server(port):
        """启动 aiohttp 服务器（HTTP + WebSocket 同端口）"""
        app = web.Application()
        app.router.add_get('/{path:.*}', http_handler)
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', port)
        await site.start()
        
        print(f'[web3d] HTTP 服务启动: http://localhost:{port}')
        return runner
else:
    from http.server import SimpleHTTPRequestHandler, HTTPServer
    import socketserver

    class QuietHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=os.path.dirname(__file__), **kwargs)
        
        def log_message(self, format, *args):
            pass

    def start_http_server_sync(port):
        with socketserver.TCPServer(("0.0.0.0", port), QuietHandler) as httpd:
            httpd.serve_forever()


async def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    ws_port = port + 1

    print(f'[web3d] ═══════════════════════════════════════════')
    print(f'[web3d] 启动 3D Web 可视化服务')
    print(f'[web3d] ═══════════════════════════════════════════')
    print(f'[web3d] HTTP 地址: http://localhost:{port}')
    print(f'[web3d] WebSocket: ws://localhost:{ws_port}')
    print(f'[web3d] ROS2: {"是" if HAS_ROS2 else "否（模拟模式）"}')
    print(f'[web3d] ═══════════════════════════════════════════')

    if HAS_ROS2:
        rclpy.init()
        global BRIDGE_NODE
        BRIDGE_NODE = BridgeNode()
        spin_thread = threading.Thread(target=rclpy.spin, args=(BRIDGE_NODE,), daemon=True)
        spin_thread.start()

    if HAS_AIOHTTP:
        # 使用 aiohttp: HTTP 和 WebSocket 可以分开端口
        http_runner = await start_server(port)
        
        # 启动 WebSocket 服务器在下一个端口
        asyncio.create_task(broadcast_loop())
        
        print(f'[web3d] 等待客户端连接...')
        print(f'[web3d] 在浏览器中打开: http://localhost:{port}')
        
        async with websockets.serve(ws_handler, '0.0.0.0', ws_port):
            await asyncio.Future()
    else:
        # 简单模式: HTTP 在后台线程，WebSocket 在主线程
        http_thread = threading.Thread(target=start_http_server_sync, args=(port,), daemon=True)
        http_thread.start()
        
        asyncio.create_task(broadcast_loop())
        
        print(f'[web3d] 等待客户端连接...')
        print(f'[web3d] 在浏览器中打开: http://localhost:{port}')
        
        async with websockets.serve(ws_handler, '0.0.0.0', ws_port):
            await asyncio.Future()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\n[web3d] 服务器已停止')
