#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车避障节点 —— 基于 LiDAR 的实时避障控制。

算法：
  1. 订阅 LiDAR 扫描数据
  2. 将扫描范围分为 5 个扇区：左前、正前、右前、左、右
  3. 检测各扇区最近障碍物距离
  4. 根据障碍物位置决策：
     - 正前有障碍：停止并转向空旷侧
     - 侧方有障碍：向反方向微调
     - 无 obstacle：直线前进
  5. 发布速度指令到 cmd_vel

用法：
    ros2 run smart_car_sim obstacle_avoidance.py
    ros2 run smart_car_sim obstacle_avoidance.py --ros-args -p car_name:=smart_car_1
    ros2 run smart_car_sim obstacle_avoidance.py --ros-args -p max_speed:=0.5 -p stop_distance:=0.8
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    """基于 LiDAR 的避障控制器"""

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # 参数声明
        self.declare_parameter('car_name', 'smart_car_1')
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('slow_distance', 1.5)
        self.declare_parameter('scan_timeout', 1.0)

        self.car_name = self.get_parameter('car_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        # 话题名称
        self.scan_topic = f'/{self.car_name}/scan'
        self.cmd_topic = f'/{self.car_name}/cmd_vel'

        # 发布者与订阅者
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        # 状态变量
        self.scan_ranges = []
        self.has_scan = False
        self.last_scan_time = self.get_clock().now()

        # 定时器：10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'避障节点启动: {self.car_name}\n'
            f'  话题: scan={self.scan_topic}, cmd={self.cmd_topic}\n'
            f'  参数: max_speed={self.max_speed}, stop_distance={self.stop_distance}'
        )

    def scan_callback(self, msg):
        """接收 LiDAR 扫描数据"""
        self.scan_ranges = msg.ranges
        self.has_scan = True
        self.last_scan_time = self.get_clock().now()

    def get_sector_min(self, ranges, start_idx, end_idx):
        """获取扇区内最小距离（过滤无效值）"""
        if not ranges:
            return float('inf')
        sector = []
        for i in range(start_idx, end_idx):
            if i < len(ranges):
                r = ranges[i]
                if 0.01 < r < 100.0:  # 过滤无效值
                    sector.append(r)
        return min(sector) if sector else float('inf')

    def analyze_scan(self):
        """分析扫描数据，返回各扇区距离"""
        if not self.scan_ranges:
            return None

        n = len(self.scan_ranges)
        if n < 10:
            return None

        # 扇区划分（假设 LiDAR 从 -π 到 +π，索引 0 在正前方）
        # 实际取决于 LiDAR 配置，这里假设标准配置
        front_sector = int(n * 0.15)  # 正前 ±27°
        side_sector = int(n * 0.25)   # 侧方 ±45°

        # 各扇区索引范围
        front_left_start = n // 2 - front_sector
        front_left_end = n // 2
        front_start = n // 2 - front_sector // 2
        front_end = n // 2 + front_sector // 2
        front_right_start = n // 2
        front_right_end = n // 2 + front_sector
        left_start = n // 2 - side_sector
        left_end = n // 2 - front_sector
        right_start = n // 2 + front_sector
        right_end = n // 2 + side_sector

        return {
            'front_left': self.get_sector_min(self.scan_ranges, front_left_start, front_left_end),
            'front': self.get_sector_min(self.scan_ranges, front_start, front_end),
            'front_right': self.get_sector_min(self.scan_ranges, front_right_start, front_right_end),
            'left': self.get_sector_min(self.scan_ranges, left_start, left_end),
            'right': self.get_sector_min(self.scan_ranges, right_start, right_end),
        }

    def control_loop(self):
        """主控制循环"""
        # 检查 LiDAR 数据是否超时
        now = self.get_clock().now()
        elapsed = (now - self.last_scan_time).nanoseconds / 1e9
        if elapsed > self.scan_timeout:
            self.get_logger().warn('LiDAR 数据超时，停止小车')
            self.publish_velocity(0.0, 0.0)
            return

        if not self.has_scan:
            return

        sectors = self.analyze_scan()
        if sectors is None:
            return

        # 决策逻辑
        twist = Twist()
        front = sectors['front']
        front_left = sectors['front_left']
        front_right = sectors['front_right']
        left = sectors['left']
        right = sectors['right']

        # 情况 1：正前方有障碍
        if front < self.stop_distance:
            self.get_logger().info(
                f'正前方障碍 {front:.2f}m < {self.stop_distance}m，停止并转向',
                throttle_duration_sec=1,
            )
            # 选择空旷侧转向
            if front_left > front_right:
                twist.angular.z = self.turn_speed  # 左转
            else:
                twist.angular.z = -self.turn_speed  # 右转
            twist.linear.x = 0.0

        # 情况 2：前方较近，减速
        elif front < self.slow_distance:
            speed = self.max_speed * (front - self.stop_distance) / (self.slow_distance - self.stop_distance)
            speed = max(0.1, min(speed, self.max_speed))
            twist.linear.x = speed
            self.get_logger().debug(f'前方 {front:.2f}m，减速到 {speed:.2f}m/s')

            # 如果侧方更近，微调方向
            if front_left < front_right and front_left < self.slow_distance:
                twist.angular.z = -self.turn_speed * 0.3
            elif front_right < front_left and front_right < self.slow_distance:
                twist.angular.z = self.turn_speed * 0.3

        # 情况 3：侧方有障碍，微调方向
        elif left < self.stop_distance or right < self.stop_distance:
            twist.linear.x = self.max_speed * 0.7
            if left < right:
                twist.angular.z = -self.turn_speed * 0.5  # 远离左侧
            else:
                twist.angular.z = self.turn_speed * 0.5   # 远离右侧
            self.get_logger().debug(f'侧方障碍，微调方向')

        # 情况 4：无 obstacle，直线前进
        else:
            twist.linear.x = self.max_speed
            twist.angular.z = 0.0

        self.publish_velocity(twist.linear.x, twist.angular.z)

    def publish_velocity(self, linear_x, angular_z):
        """发布速度指令"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止小车
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
