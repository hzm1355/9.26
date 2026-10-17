#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
区域触发公共底座 —— 协议 §三.(三)(四)(五)(六) 的共用感知层。

功能：
  - 从 regions.yaml 读取 3D box 区域（支持 yaw 旋转与 z 高度过滤，
    避免高架上的车误触发地面区域）；
  - 订阅 gazebo_ros_state 插件发布的 /model_states（全模型位姿+速度），
    按 vehicle_name_regex 过滤出车辆，逐区域做进出检测；
  - 发布：
      /v2x/region_events          v2x_msgs/RegionEvent  进/出事件流
      /v2x/region/<id>/state      v2x_msgs/RegionState  区域占用快照(transient_local)
                                  （话题名中 id 的非字母数字字符替换为 _）
  - 车辆被删除（respawn/清理）时对其所在区域补发 EXIT，避免占用状态泄漏。

下游模块用法示例：
  - 测速：订阅两个 speed_point 区域的 ENTER 时间差 → 区间平均速度；
  - ETC/停车场：etc_gate / parking_gate 区域 ENTER 触发识别与抬杆流程；
  - 卡口：checkpoint 区域 ENTER 时刻的 pose + 相机图像 → 抓拍。

参数：
  regions_file         区域配置 yaml 路径；留空则用本包 share/config/regions.yaml
  vehicle_name_regex   车辆模型名过滤正则（默认 ^v2x_，多车命名 v2x_prius_2 等）
  states_topic         位姿源话题（默认 /model_states）
  min_process_interval 回调最小处理间隔秒（默认 0.02，位姿源频率高时节流）
  debounce             ENTER/EXIT 判定所需连续样本数（默认 1，测速打点请保持 1）
"""
import math
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
from v2x_msgs.msg import RegionEvent, RegionState

import yaml


def state_topic(region_id):
    """区域状态话题名；ROS 话题名禁止 '-' 等字符，统一替换为 '_'。"""
    return 'v2x/region/' + re.sub(r'[^A-Za-z0-9_]', '_', region_id) + '/state'


class Region:
    """yaw 旋转的 3D box 区域；contains 做 world→local 坐标变换后判包含。"""

    __slots__ = ('id', 'type', 'cx', 'cy', 'cz', 'hx', 'hy', 'hz',
                 'cos', 'sin', 'inside')

    def __init__(self, d):
        self.id = str(d['id'])
        self.type = str(d.get('type', 'custom'))
        c, s = d['center'], d['size']
        self.cx, self.cy, self.cz = (float(c[0]), float(c[1]),
                                     float(c[2]) if len(c) > 2 else 1.0)
        self.hx, self.hy, self.hz = float(s[0]) / 2, float(s[1]) / 2, float(s[2]) / 2
        yaw = math.radians(float(d.get('yaw', 0.0)))
        self.cos, self.sin = math.cos(yaw), math.sin(yaw)
        self.inside = set()

    def contains(self, x, y, z):
        dx, dy = x - self.cx, y - self.cy
        lx = dx * self.cos + dy * self.sin
        ly = -dx * self.sin + dy * self.cos
        return (abs(lx) <= self.hx and abs(ly) <= self.hy
                and abs(z - self.cz) <= self.hz)


class RegionTriggerNode(Node):
    def __init__(self):
        super().__init__('region_trigger')
        self.declare_parameter('regions_file', '')
        self.declare_parameter('vehicle_name_regex', '^v2x_')
        self.declare_parameter('states_topic', '/model_states')
        self.declare_parameter('min_process_interval', 0.02)
        self.declare_parameter('debounce', 1)

        path = self.get_parameter('regions_file').value
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'regions.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        self.regions = [Region(d) for d in cfg.get('regions', [])]
        if not self.regions:
            self.get_logger().warn(f'regions.yaml 无区域定义: {path}')
        else:
            self.get_logger().info(
                f'区域触发底座就绪: {len(self.regions)} 个区域 <- {path} '
                + ', '.join(f'{r.id}({r.type})' for r in self.regions))

        self.vehicle_re = re.compile(self.get_parameter('vehicle_name_regex').value)
        self.min_dt = self.get_parameter('min_process_interval').value
        self.debounce = max(1, int(self.get_parameter('debounce').value))

        self.event_pub = self.create_publisher(RegionEvent, 'v2x/region_events', 100)
        self.state_pubs = {}
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               history=HistoryPolicy.KEEP_LAST,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for r in self.regions:
            pub = self.create_publisher(RegionState, state_topic(r.id), state_qos)
            self.state_pubs[r.id] = pub
            self._publish_state(r)

        # 进出判定的连续计数：(region_id, vehicle_id) -> 连续在内/在外样本数
        self.hit_count = {}
        self.last_proc = None
        topic = self.get_parameter('states_topic').value
        self.create_subscription(ModelStates, topic, self.on_states, 10)
        self.get_logger().info(f'订阅位姿源 {topic}（regex={self.vehicle_re.pattern}）')

    def _publish_state(self, region):
        msg = RegionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.region_id = region.id
        msg.region_type = region.type
        msg.occupied = bool(region.inside)
        msg.vehicle_ids = sorted(region.inside)
        self.state_pubs[region.id].publish(msg)

    def _emit(self, region, vehicle_id, event_type, pose, twist):
        msg = RegionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.region_id = region.id
        msg.region_type = region.type
        msg.vehicle_id = vehicle_id
        msg.event_type = event_type
        msg.pose = pose
        msg.speed = math.hypot(twist.linear.x, twist.linear.y)
        self.event_pub.publish(msg)
        tag = 'ENTER' if event_type == RegionEvent.EVENT_ENTER else 'EXIT'
        self.get_logger().info(
            f'{tag} region={region.id}({region.type}) vehicle={vehicle_id} '
            f'speed={msg.speed:.2f}m/s')

    def on_states(self, msg):
        now = self.get_clock().now()
        if self.last_proc is not None and (now - self.last_proc).nanoseconds < \
                self.min_dt * 1e9:
            return
        self.last_proc = now

        vehicles = [(n, p, t) for n, p, t in zip(msg.name, msg.pose, msg.twist)
                    if self.vehicle_re.match(n)]
        seen = {n for n, _, _ in vehicles}

        # 消失的车辆：对其仍占用的区域补 EXIT（位姿已不可得，发默认值）
        blank_pose, blank_twist = Pose(), Twist()
        for region in self.regions:
            gone = region.inside - seen
            for vid in gone:
                region.inside.discard(vid)
                self.hit_count.pop((region.id, vid), None)
                self._emit(region, vid, RegionEvent.EVENT_EXIT,
                           blank_pose, blank_twist)
            if gone:
                self._publish_state(region)
        if not vehicles:
            return

        for region in self.regions:
            changed = False
            for name, pose, twist in vehicles:
                inside_now = region.contains(pose.position.x,
                                             pose.position.y,
                                             pose.position.z)
                was_in = name in region.inside
                key = (region.id, name)
                if inside_now == was_in:
                    self.hit_count.pop(key, None)
                    continue
                cnt = self.hit_count.get(key, 0) + 1
                if cnt < self.debounce:
                    self.hit_count[key] = cnt
                    continue
                self.hit_count.pop(key, None)
                if inside_now:
                    region.inside.add(name)
                    self._emit(region, name, RegionEvent.EVENT_ENTER, pose, twist)
                else:
                    region.inside.discard(name)
                    self._emit(region, name, RegionEvent.EVENT_EXIT, pose, twist)
                changed = True
            if changed:
                self._publish_state(region)


def main(args=None):
    rclpy.init(args=args)
    node = RegionTriggerNode()
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
