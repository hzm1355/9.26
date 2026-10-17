#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自适应配时节点 —— 根据实时交通流数据动态调整信号灯配时。

原理：
  1) 订阅 /model_states 直接检测各方向排队区域的车辆数
  2) 订阅 /v2x/region_events 获取区域触发事件（备用）
  3) 综合排队长度 + 等待时间计算调整系数
  4) 发布到 traffic_light/<group>/adaptive_factor

算法（加权均衡）：
  - 各方向需求度 = queue_count * avg_wait_time（排队数 × 平均等待时间）
  - 调整系数 = 1.0 + 0.5 * (direction_demand / total_demand - 1/N)
  - 限制范围：[min_factor, max_factor]
  - 当总车辆 < min_vehicles 时不调整（factor=1.0）

接口：
  订阅：
    /v2x/region_events (v2x_msgs/RegionEvent) —— 区域触发事件
    /model_states (gazebo_msgs/ModelStates) —— 车辆位姿
    traffic_light/<group>/stats (v2x_msgs/TrafficLightStats) —— 灯组统计
  发布：
    traffic_light/<group>/adaptive_factor (std_msgs/String) —— 调整系数
    /traffic_light/adaptive_stats (std_msgs/String) —— 自适应统计信息
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

try:
    from v2x_msgs.msg import RegionEvent, TrafficLightStats
    HAS_V2X_MSGS = True
except ImportError:
    HAS_V2X_MSGS = False


class AdaptiveTimingNode(Node):
    def __init__(self):
        super().__init__('adaptive_timing_node')

        self.declare_parameter('update_period', 5.0)
        self.declare_parameter('min_factor', 0.5)
        self.declare_parameter('max_factor', 2.0)
        self.declare_parameter('min_vehicles_for_adapt', 2)
        self.declare_parameter('wait_weight', 0.3)

        self.update_period = float(self.get_parameter('update_period').value)
        self.min_factor = float(self.get_parameter('min_factor').value)
        self.max_factor = float(self.get_parameter('max_factor').value)
        self.min_vehicles = int(self.get_parameter('min_vehicles_for_adapt').value)
        self.wait_weight = float(self.get_parameter('wait_weight').value)

        self.declare_parameter('direction_zones', '')
        zones_str = self.get_parameter('direction_zones').value
        self.direction_zones = self._parse_zones(zones_str)

        self.queue_counts = {}
        self.direction_groups = {}
        self.group_stats = {}
        for direction, zones in self.direction_zones.items():
            group = zones.get('group', direction)
            self.direction_groups[direction] = group
            for zone in zones.get('zones', []):
                self.queue_counts[zone] = 0
            if group not in self.group_stats:
                self.group_stats[group] = {
                    'queue_length': 0, 'avg_wait': 0.0, 'throughput': 0.0,
                }

        if HAS_V2X_MSGS:
            self.create_subscription(RegionEvent, '/v2x/region_events',
                                     self.on_region_event, 10)
            for group in set(self.direction_groups.values()):
                self.create_subscription(
                    TrafficLightStats, f'traffic_light/{group}/stats',
                    lambda msg, g=group: self.on_stats(msg, g), 10)

        self.create_subscription(ModelStates, '/model_states',
                                 self.on_model_states, 10)

        self.pubs = {}
        for direction, group in self.direction_groups.items():
            topic = f'traffic_light/{group}/adaptive_factor'
            if group not in self.pubs:
                self.pubs[group] = self.create_publisher(String, topic, 10)

        self.stats_pub = self.create_publisher(String, '/traffic_light/adaptive_stats', 10)

        self.timer = self.create_timer(self.update_period, self.on_timer)

        self.get_logger().info(
            f'AdaptiveTimingNode: zones={list(self.direction_zones.keys())} '
            f'period={self.update_period}s factor=[{self.min_factor},{self.max_factor}]')

    def _parse_zones(self, zones_str):
        if not zones_str:
            return {
                'NS': {'group': 'J1_ns', 'zones': ['J1_ns_zone'],
                       'area': {'x': 0.0, 'y_min': -30.0, 'y_max': -8.0}},
                'EW': {'group': 'J1_ew', 'zones': ['J1_ew_zone'],
                       'area': {'x': 0.0, 'x_min': -30.0, 'x_max': -8.0, 'y': 0.0}},
            }
        import yaml
        try:
            return yaml.safe_load(zones_str)
        except Exception:
            return {}

    def on_region_event(self, msg):
        zone = msg.region_id
        if zone not in self.queue_counts:
            return
        if msg.event_type == RegionEvent.EVENT_ENTER:
            self.queue_counts[zone] = self.queue_counts.get(zone, 0) + 1
        elif msg.event_type == RegionEvent.EVENT_EXIT:
            self.queue_counts[zone] = max(0, self.queue_counts.get(zone, 0) - 1)

    def on_stats(self, msg, group):
        if group in self.group_stats:
            self.group_stats[group]['queue_length'] = msg.queue_length
            self.group_stats[group]['avg_wait'] = msg.avg_wait_time
            self.group_stats[group]['throughput'] = msg.throughput

    def on_model_states(self, msg: ModelStates):
        for direction, info in self.direction_zones.items():
            area = info.get('area')
            if not area:
                continue
            count = 0
            for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
                if not (name.startswith('v2x_') or name.startswith('smart_car_')
                        or name.startswith('car_')):
                    continue
                x, y = pose.position.x, pose.position.y
                speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
                if speed > 1.0:
                    continue
                in_area = False
                if 'x_min' in area and 'x_max' in area:
                    if 'y_min' in area and 'y_max' in area:
                        in_area = (area['x_min'] <= x <= area['x_max'] and
                                   area['y_min'] <= y <= area['y_max'])
                    else:
                        in_area = area['x_min'] <= x <= area['x_max']
                elif 'y_min' in area and 'y_max' in area:
                    in_area = area['y_min'] <= y <= area['y_max']
                if in_area:
                    count += 1
            for zone in info.get('zones', []):
                self.queue_counts[zone] = count

    def on_timer(self):
        direction_demand = {}
        direction_counts = {}
        total_vehicles = 0

        for direction, info in self.direction_zones.items():
            zone_count = sum(self.queue_counts.get(z, 0) for z in info.get('zones', []))
            group = self.direction_groups[direction]
            stats = self.group_stats.get(group, {})
            queue_len = stats.get('queue_length', zone_count)
            count = max(queue_len, zone_count)
            direction_counts[direction] = count
            total_vehicles += count

            avg_wait = stats.get('avg_wait', 0.0)
            demand = count * (1.0 + self.wait_weight * avg_wait)
            direction_demand[direction] = demand

        if total_vehicles < self.min_vehicles:
            for group in set(self.direction_groups.values()):
                msg = String()
                msg.data = '1.0'
                if group in self.pubs:
                    self.pubs[group].publish(msg)
            return

        total_demand = sum(direction_demand.values())
        n_directions = len(direction_demand)
        if total_demand <= 0:
            total_demand = 1.0

        factors = {}
        for direction, demand in direction_demand.items():
            group = self.direction_groups[direction]
            share = demand / total_demand
            fair_share = 1.0 / n_directions
            imbalance = share - fair_share
            factor = 1.0 + 1.5 * imbalance
            factor = max(self.min_factor, min(self.max_factor, factor))

            if group not in factors or factor > factors[group]:
                factors[group] = factor

        for group, factor in factors.items():
            if group in self.pubs:
                msg = String()
                msg.data = f'{factor:.2f}'
                self.pubs[group].publish(msg)

        stats = {
            'direction_counts': direction_counts,
            'direction_demand': {d: round(v, 2) for d, v in direction_demand.items()},
            'total_vehicles': total_vehicles,
            'factors': {g: round(f, 3) for g, f in factors.items()},
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
        }
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)

        self.get_logger().debug(
            f'adaptive: counts={direction_counts} demand={direction_demand} factors={factors}')


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveTimingNode()
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
