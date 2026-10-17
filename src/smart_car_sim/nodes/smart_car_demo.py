#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车演示脚本 —— 驱动小车按预设路线行驶，展示传感器数据与故障模拟。

用法：
    ros2 run smart_car_sim smart_car_demo.py
    ros2 run smart_car_sim smart_car_demo.py --ros-args -p car_name:=smart_car_1
    ros2 run smart_car_sim smart_car_demo.py --ros-args -p pattern:=square
    ros2 run smart_car_sim smart_car_demo.py --ros-args -p enable_fault:=true

运动模式：
  - square:   正方形路线
  - circle:   圆形路线
  - figure8:  8字形路线
  - forward:  直线前进后停止
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_srvs.srv import SetBool


class SmartCarDemo(Node):
    def __init__(self):
        super().__init__('smart_car_demo')
        self.declare_parameter('car_name', 'smart_car_1')
        self.declare_parameter('pattern', 'square')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('enable_fault', False)
        self.declare_parameter('demo_duration', 30.0)

        self.car_name = self.get_parameter('car_name').value
        self.pattern = self.get_parameter('pattern').value
        self.speed = self.get_parameter('speed').value
        self.demo_duration = self.get_parameter('demo_duration').value

        self.cmd_pub = self.create_publisher(Twist, f'/{self.car_name}/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{self.car_name}/scan', self.on_scan, 10)
        self.imu_sub = self.create_subscription(
            Imu, f'/{self.car_name}/imu/data', self.on_imu, 10)

        self.fault_cli = self.create_client(SetBool, f'/{self.car_name}/enable_fault')

        self.t0 = self.get_clock().now()
        self.phase = 0
        self.scan_count = 0
        self.imu_count = 0
        self.min_range = float('inf')

        self.timer = self.create_timer(0.05, self.on_tick)

        if self.get_parameter('enable_fault').value:
            self.activate_fault()

        self.get_logger().info(
            f'演示启动: {self.car_name} 模式={self.pattern} '
            f'速度={self.speed}m/s 时长={self.demo_duration}s')

    def on_scan(self, msg):
        self.scan_count += 1
        if msg.ranges:
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid:
                self.min_range = min(self.min_range, min(valid))

    def on_imu(self, msg):
        self.imu_count += 1

    def activate_fault(self):
        if not self.fault_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('故障模拟服务不可用')
            return
        req = SetBool.Request()
        req.data = True
        self.fault_cli.call_async(req)
        self.get_logger().info('故障模拟已激活')

    def on_tick(self):
        elapsed = (self.get_clock().now() - self.t0).nanoseconds / 1e9

        if elapsed > self.demo_duration:
            self.stop()
            self.print_summary()
            raise SystemExit(0)

        twist = Twist()
        if self.pattern == 'square':
            twist = self.pattern_square(elapsed)
        elif self.pattern == 'circle':
            twist = self.pattern_circle(elapsed)
        elif self.pattern == 'figure8':
            twist = self.pattern_figure8(elapsed)
        elif self.pattern == 'forward':
            twist = self.pattern_forward(elapsed)

        self.cmd_pub.publish(twist)

        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            self.get_logger().info(
                f't={elapsed:.1f}s | scan={self.scan_count} imu={self.imu_count} '
                f'最近障碍={self.min_range:.2f}m')

    def pattern_square(self, t):
        side_time = 4.0
        turn_time = 1.5
        cycle = side_time + turn_time
        phase_t = t % cycle
        twist = Twist()
        if phase_t < side_time:
            twist.linear.x = self.speed
        else:
            twist.angular.z = math.pi / 2 / turn_time
        return twist

    def pattern_circle(self, t):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.speed / 1.5
        return twist

    def pattern_figure8(self, t):
        twist = Twist()
        twist.linear.x = self.speed * 0.7
        twist.angular.z = 0.5 * math.sin(t * 0.5)
        return twist

    def pattern_forward(self, t):
        twist = Twist()
        if t < 5.0:
            twist.linear.x = self.speed
        return twist

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def print_summary(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'演示完成: {self.car_name}')
        self.get_logger().info(f'  模式: {self.pattern}')
        self.get_logger().info(f'  LiDAR 消息: {self.scan_count}')
        self.get_logger().info(f'  IMU 消息: {self.imu_count}')
        self.get_logger().info(f'  最近障碍物距离: {self.min_range:.2f}m')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = SmartCarDemo()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
