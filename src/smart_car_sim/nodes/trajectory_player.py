#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹回放器 —— 对应协议 §三.(一).5 场景回放。

读取 trajectory_recorder 保存的 JSON，为每辆车生成一个半透明"幽灵"标记模型，
按录制时间轴用 /set_entity_state 驱动其复现历史轨迹；同时把回放位姿发布到
/v2x/playback/<name>/pose(PoseStamped) 供可视化界面绘制。

服务（std_srvs/Trigger）：~/load 载入文件并生成幽灵；~/play 开始回放；~/stop 停止。
插值 sample_at() 为纯函数，便于无 Gazebo 的合成自测。
"""
import json
import math
import os

import rclpy
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_srvs.srv import Trigger

GHOST_SDF = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <pose>0 0 0.5 0 0 0</pose>
    <link name="link">
      <visual name="v">
        <geometry><box><size>1.6 3.0 1.0</size></box></geometry>
        <material>
          <ambient>0 0.85 0.4 1</ambient><diffuse>0 0.7 0.3 1</diffuse>
          <emissive>0 0.45 0.18 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


def sample_at(samples, t):
    """在按时间升序的 samples([[t,x,y,z,qx,qy,qz,qw,...]]) 中取时刻 t 的位姿。
    位置线性插值，姿态取邻近样本（足以复现轨迹）。越界则夹紧到端点。
    返回 (x,y,z,qx,qy,qz,qw) 或 None（空数据）。"""
    if not samples:
        return None
    if t <= samples[0][0]:
        s = samples[0]
        return tuple(s[1:8])
    if t >= samples[-1][0]:
        s = samples[-1]
        return tuple(s[1:8])
    lo, hi = 0, len(samples) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    a, b = samples[lo], samples[hi]
    dt = b[0] - a[0]
    w = 0.0 if dt <= 0 else (t - a[0]) / dt
    xyz = [a[1 + i] + (b[1 + i] - a[1 + i]) * w for i in range(3)]
    # 姿态：取较近样本的四元数（回放可视化足够）
    q = a[4:8] if w < 0.5 else b[4:8]
    return (xyz[0], xyz[1], xyz[2], q[0], q[1], q[2], q[3])


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trajectory_player')
        self.declare_parameter('input_file', os.path.expanduser('~/v2x_trajectory.json'))
        self.declare_parameter('ghost_prefix', 'playback_')
        self.declare_parameter('spawn_service', '/spawn_entity')
        self.declare_parameter('set_state_service', '/set_entity_state')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('loop', False)

        self.input_file = self.get_parameter('input_file').value
        self.prefix = self.get_parameter('ghost_prefix').value
        self.loop = bool(self.get_parameter('loop').value)

        self.data = {}          # name -> samples
        self.duration = 0.0
        self.pose_pubs = {}
        self.playing = False
        self.t0 = None
        self.ghost_ready = set()   # 已成功生成的幽灵名，避免 spawn 未完成即 set_state 报错

        self.spawn_cli = self.create_client(SpawnEntity, self.get_parameter('spawn_service').value)
        self.set_cli = self.create_client(SetEntityState, self.get_parameter('set_state_service').value)
        self.create_service(Trigger, '~/load', self.svc_load)
        self.create_service(Trigger, '~/play', self.svc_play)
        self.create_service(Trigger, '~/stop', self.svc_stop)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('rate').value), self.on_tick)
        self.get_logger().info(f'trajectory_player: input={self.input_file}')

    # ---------------- 服务 ----------------
    def svc_load(self, _req, resp):
        try:
            with open(self.input_file, encoding='utf-8') as f:
                raw = json.load(f)
            self.data = {name: v['samples'] for name, v in raw.get('vehicles', {}).items()}
            self.duration = max((s[-1][0] for s in
                                 (v for v in self.data.values() if v)), default=0.0)
            self.ghost_ready.clear()
            for name in self.data:
                if name not in self.pose_pubs:
                    self.pose_pubs[name] = self.create_publisher(
                        PoseStamped, f'/v2x/playback/{name}/pose', 10)
            self.spawn_ghosts()
            resp.success = True
            resp.message = f'loaded {len(self.data)} vehicles, duration={self.duration:.1f}s'
            self.get_logger().info(resp.message)
        except Exception as e:  # noqa: BLE001
            resp.success = False
            resp.message = f'load failed: {e}'
            self.get_logger().error(resp.message)
        return resp

    def svc_play(self, _req, resp):
        if not self.data:
            resp.success = False
            resp.message = 'no data loaded (call ~/load first)'
            return resp
        self.t0 = None
        self.playing = True
        resp.success = True
        resp.message = f'playback started ({self.duration:.1f}s)'
        self.get_logger().info(resp.message)
        return resp

    def svc_stop(self, _req, resp):
        self.playing = False
        resp.success = True
        resp.message = 'playback stopped'
        return resp

    # ---------------- 幽灵生成 ----------------
    def spawn_ghosts(self):
        # 注意：本方法在服务回调(svc_load)中执行，节点已被外层 executor 旋转，
        # 不能再 spin_until_future_complete（会死锁/报错）。改为 fire-and-forget 异步调用。
        if not self.spawn_cli.service_is_ready():
            self.get_logger().warn('/spawn_entity 未就绪，跳过幽灵生成（回放位姿仍会发布）')
            return
        for name in self.data:
            gname = self.prefix + name
            req = SpawnEntity.Request()
            req.name = gname
            req.xml = GHOST_SDF.format(name=gname)
            req.robot_namespace = gname
            req.reference_frame = 'world'
            s0 = sample_at(self.data[name], 0.0)
            req.initial_pose = Pose()
            if s0:
                req.initial_pose.position = Point(x=s0[0], y=s0[1], z=s0[2])
                req.initial_pose.orientation = Quaternion(x=s0[3], y=s0[4], z=s0[5], w=s0[6])
            self.spawn_cli.call_async(req).add_done_callback(
                lambda f, n=gname: self._on_ghost_spawned(f, n))

    def _on_ghost_spawned(self, fut, gname):
        try:
            resp = fut.result()
            if resp is not None and resp.success:
                self.ghost_ready.add(gname)
            else:
                msg = getattr(resp, 'message', 'no response')
                self.get_logger().warn(f'幽灵 {gname} 生成失败：{msg}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'幽灵 {gname} 生成异常：{e}')

    # ---------------- 回放循环 ----------------
    def on_tick(self):
        if not self.playing or not self.data:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        if t > self.duration:
            if self.loop:
                self.t0 = now
                t = 0.0
            else:
                self.playing = False
                self.get_logger().info('回放结束')
                return
        stamp = self.get_clock().now().to_msg()
        for name, samples in self.data.items():
            s = sample_at(samples, t)
            if not s:
                continue
            x, y, z, qx, qy, qz, qw = s
            # 发布回放位姿（供 UI 绘制）
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'world'
            ps.pose.position = Point(x=x, y=y, z=z)
            ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            self.pose_pubs[name].publish(ps)
            # 驱动幽灵模型（仅在幽灵生成成功后，避免 set_entity_state 报错）
            if self.set_cli.service_is_ready() and (self.prefix + name) in self.ghost_ready:
                req = SetEntityState.Request()
                req.state.name = self.prefix + name
                req.state.pose = ps.pose
                req.state.reference_frame = 'world'
                self.set_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
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
