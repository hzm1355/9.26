#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹记录器 —— 对应协议 §三.(一).5 场景回放的数据采集侧。

统一从 /model_states（gazebo_ros_state 发布的全模型位姿+速度）采集所有 v2x_* 车辆的
时间戳轨迹，无需各车 TF/odom，天然支持多车。

服务（std_srvs/Trigger）：
  ~/start  开始记录        ~/stop   暂停记录
  ~/save   将当前缓冲写入 output_file(JSON)   ~/clear 清空缓冲
JSON 结构：
  {"meta": {...}, "vehicles": {"<name>": {"samples": [[t,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz], ...]}}}
"""
import json
import os
import re

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from std_srvs.srv import Trigger


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('states_topic', '/model_states')
        self.declare_parameter('vehicle_name_regex', '^(v2x_|smart_car_|tb3_)')
        self.declare_parameter('output_file', os.path.expanduser('~/v2x_trajectory.json'))
        self.declare_parameter('min_interval', 0.05)   # 每车最小采样间隔(s)，限制文件体积

        self.regex = re.compile(self.get_parameter('vehicle_name_regex').value)
        self.output_file = self.get_parameter('output_file').value
        self.min_interval = float(self.get_parameter('min_interval').value)

        self.recording = True
        self.buffers = {}     # name -> list of sample tuples
        self.last_t = {}      # name -> last sample sim time
        self.t0 = None

        self.create_subscription(ModelStates, self.get_parameter('states_topic').value,
                                 self.on_states, 10)
        self.create_service(Trigger, '~/start', self.svc_start)
        self.create_service(Trigger, '~/stop', self.svc_stop)
        self.create_service(Trigger, '~/save', self.svc_save)
        self.create_service(Trigger, '~/clear', self.svc_clear)
        self.get_logger().info(f'trajectory_recorder: 记录中 -> {self.output_file} '
                               f'(regex={self.regex.pattern})')

    def on_states(self, msg: ModelStates):
        if not self.recording:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None:
            self.t0 = now
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if not self.regex.match(name):
                continue
            if name in self.last_t and (now - self.last_t[name]) < self.min_interval:
                continue
            self.last_t[name] = now
            p, o = pose.position, pose.orientation
            lv, av = twist.linear, twist.angular
            self.buffers.setdefault(name, []).append([
                round(now - self.t0, 4),
                round(p.x, 4), round(p.y, 4), round(p.z, 4),
                round(o.x, 5), round(o.y, 5), round(o.z, 5), round(o.w, 5),
                round(lv.x, 4), round(lv.y, 4), round(lv.z, 4),
                round(av.x, 4), round(av.y, 4), round(av.z, 4),
            ])

    def svc_start(self, _req, resp):
        self.recording = True
        resp.success = True
        resp.message = 'recording started'
        return resp

    def svc_stop(self, _req, resp):
        self.recording = False
        resp.success = True
        resp.message = 'recording stopped'
        return resp

    def svc_clear(self, _req, resp):
        self.buffers = {}
        self.last_t = {}
        self.t0 = None
        resp.success = True
        resp.message = 'buffer cleared'
        return resp

    def svc_save(self, _req, resp):
        try:
            n = self.save()
            resp.success = True
            resp.message = f'saved {n} vehicles -> {self.output_file}'
            self.get_logger().info(resp.message)
        except Exception as e:  # noqa: BLE001
            resp.success = False
            resp.message = f'save failed: {e}'
            self.get_logger().error(resp.message)
        return resp

    def save(self):
        total = sum(len(v) for v in self.buffers.values())
        data = {
            'meta': {
                'vehicles': len(self.buffers),
                'total_samples': total,
                'frame': 'world',
                'columns': ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                            'vx', 'vy', 'vz', 'wx', 'wy', 'wz'],
            },
            'vehicles': {name: {'samples': s} for name, s in self.buffers.items()},
        }
        d = os.path.dirname(os.path.abspath(self.output_file))
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False)
        return len(self.buffers)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.buffers:
                node.save()
                node.get_logger().info('退出前自动保存轨迹')
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
