#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多车交通流节点 —— 对应协议 §三.(一) 多车仿真 + 场景元素动态。

职责：
  1) 按 traffic_flow.yaml 生成多辆轻量交通车（planar_move 驱动的盒车），
     每辆用独立 robot_namespace -> 话题 /<name>/cmd_vel、/<name>/odom 互不干扰；
  2) 订阅 /model_states 获取各车实时位姿，用“纯跟踪(pure-pursuit)”沿配置路线往返行驶；
  3) 车辆名以 v2x_ 前缀命名，天然被区域触发底座与轨迹记录器识别。

不发布 TF（publish_odom_tf=false），避免多车 odom->base_link 帧名冲突；
轨迹记录统一走 /model_states，无需各车 TF。
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from rclpy.node import Node

# 轻量交通车 SDF 模板：{name} 唯一模型名，{color} 车身颜色，planar_move 直接施加速度。
CAR_SDF = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0.4 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>200.0</mass>
        <inertia><ixx>50</ixx><iyy>120</iyy><izz>150</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="c">
        <pose>0 0 0.4 0 0 0</pose>
        <geometry><box><size>1.6 3.0 0.8</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="body">
        <pose>0 0 0.4 0 0 0</pose>
        <geometry><box><size>1.6 3.0 0.8</size></box></geometry>
        <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>
      </visual>
      <visual name="cabin">
        <pose>0 0.2 0.95 0 0 0</pose>
        <geometry><box><size>1.3 1.4 0.5</size></box></geometry>
        <material><ambient>0.1 0.1 0.12 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material>
      </visual>
    </link>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <update_rate>50</update_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <odometry_frame>{name}_odom</odometry_frame>
      <robot_base_frame>{name}_base</robot_base_frame>
    </plugin>
  </model>
</sdf>
'''


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class TrafficFlowNode(Node):
    def __init__(self):
        super().__init__('traffic_flow')
        self.declare_parameter('config_file', '')
        self.declare_parameter('states_topic', '/model_states')
        self.declare_parameter('spawn_service', '/spawn_entity')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('goal_radius', 2.0)
        self.declare_parameter('steer_gain', 1.2)
        self.declare_parameter('max_angular', 1.0)

        path = self.get_parameter('config_file').value
        if not path:
            path = os.path.join(get_package_share_directory('v2x_demo'),
                                'config', 'traffic_flow.yaml')
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}

        self.goal_radius = float(self.get_parameter('goal_radius').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.cars = {}
        for c in cfg.get('cars', []):
            name = c['name']
            self.cars[name] = {
                'route': [[float(p[0]), float(p[1])] for p in c.get('route', [])],
                'speed': float(c.get('speed', 2.5)),
                'start': [float(c['start'][0]), float(c['start'][1]), float(c.get('start')[2] if len(c.get('start', [0, 0, 0])) > 2 else 0.0)],
                'color': c.get('color', [0.3, 0.5, 0.8]),
                'idx': 0,
                'dir': 1,          # 1 正向 / -1 反向（到端点折返）
                'cmd_pub': self.create_publisher(Twist, f'/{name}/cmd_vel', 10),
                'pose': None,
            }

        self.latest = {}   # name -> (x, y, yaw)
        self.spawn_cli = self.create_client(SpawnEntity, self.get_parameter('spawn_service').value)
        self.create_subscription(ModelStates, self.get_parameter('states_topic').value,
                                 self.on_states, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('control_rate').value),
                                       self.on_control)
        self.get_logger().info(f'traffic_flow: {len(self.cars)} 辆车 <- {path}')
        self.spawn_all()

    # ---------------- 生成 ----------------
    def spawn_all(self):
        for name, car in self.cars.items():
            if not car['route']:
                self.get_logger().warn(f'{name} 无路线，跳过')
                continue
            sx, sy, syaw = car['start']
            color = ' '.join(f'{v:.3f}' for v in car['color'])
            xml = CAR_SDF.format(name=name, color=color)
            req = SpawnEntity.Request()
            req.name = name
            req.xml = xml
            req.robot_namespace = name
            req.initial_pose = Pose()
            req.initial_pose.position = Point(x=sx, y=sy, z=0.4)
            req.initial_pose.orientation = yaw_to_quat(syaw)
            req.reference_frame = 'world'
            self._call_spawn(name, req)

    def _call_spawn(self, name, req):
        if not self.spawn_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{name}: /spawn_entity 服务不可用（gzserver 未起？）')
            return
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        try:
            res = fut.result()
            if res and res.success:
                self.get_logger().info(f'{name}: 生成成功 @ {req.initial_pose.position}')
            else:
                self.get_logger().error(f'{name}: 生成失败 {getattr(res, "status_message", "")}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'{name}: 生成异常 {e}')

    # ---------------- 反馈 ----------------
    def on_states(self, msg: ModelStates):
        for name, pose in zip(msg.name, msg.pose):
            if name in self.cars:
                q = pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.latest[name] = (pose.position.x, pose.position.y, yaw)

    # ---------------- 控制 ----------------
    def on_control(self):
        for name, car in self.cars.items():
            pose = self.latest.get(name)
            if not pose or not car['route']:
                continue
            x, y, yaw = pose
            route = car['route']
            idx = car['idx']
            gx, gy = route[idx]
            d = math.hypot(gx - x, gy - y)
            if d < self.goal_radius:
                nxt = idx + car['dir']
                if nxt >= len(route) or nxt < 0:
                    car['dir'] *= -1
                    nxt = idx + car['dir']
                car['idx'] = max(0, min(len(route) - 1, nxt))
                gx, gy = route[car['idx']]
            desired = math.atan2(gy - y, gx - x)
            err = norm_angle(desired - yaw)
            cmd = Twist()
            # 大转角时减速，避免纯跟踪甩尾
            cmd.linear.x = car['speed'] * max(0.25, math.cos(min(abs(err), math.pi / 2)))
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.steer_gain * err))
            car['cmd_pub'].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficFlowNode()
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
