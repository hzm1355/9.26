#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交通信号灯配时控制器（最小骨架）—— 对应协议 §三.(二)。

功能：
  - 按 绿→黄→红 时序循环切换灯态（时长由 traffic_light_params.yaml 配置）；
  - 通过标准 ROS 2 话题 `traffic_light/state` 发布当前灯态（std_msgs/String），
    供车辆决策节点、路口通行效率统计节点订阅；
  - 使用仿真时间(use_sim_time)，随 Gazebo 暂停/加速同步。

扩展点（下一步开发）：
  1) 视觉驱动：结合 Gazebo LedPlugin 或自定义 visual 插件，让 stop_light 的红/黄/绿
     球体 emissive 随灯态实际点亮（当前骨架仅发布话题，不改渲染）；
  2) 自适应配时：订阅车流量话题，动态调整 green/red 时长；
  3) 多路口联动：为每个路口实例化一个控制器并用 cycle_offset 设相位差；
  4) 事件触发：提供 service 接口实现急救车优先绿通。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TrafficLightController(Node):
    # 灯态循环顺序
    PHASES = ('GREEN', 'YELLOW', 'RED')

    def __init__(self):
        super().__init__('traffic_light_controller')

        # 参数
        self.declare_parameter('green_duration', 10.0)
        self.declare_parameter('yellow_duration', 3.0)
        self.declare_parameter('red_duration', 10.0)
        self.declare_parameter('state_topic', 'traffic_light/state')
        # 注意：use_sim_time 由 launch 的 parameters 自动声明，此处不可再 declare，
        # 否则抛 ParameterAlreadyDeclaredException 导致节点退出。

        self.durations = {
            'GREEN': float(self.get_parameter('green_duration').value),
            'YELLOW': float(self.get_parameter('yellow_duration').value),
            'RED': float(self.get_parameter('red_duration').value),
        }

        topic = self.get_parameter('state_topic').value
        self.pub = self.create_publisher(String, topic, 10)

        # 初始相位与计时
        self.phase_index = 0
        self.elapsed = 0.0
        # 固定 0.1s 定时器；使用仿真时间时按 sim time 推进
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.on_timer)

        self.get_logger().info(
            'TrafficLightController started: '
            f"G={self.durations['GREEN']}s Y={self.durations['YELLOW']}s "
            f"R={self.durations['RED']}s -> topic '{topic}'")
        self.publish_state()

    @property
    def current_phase(self):
        return self.PHASES[self.phase_index]

    def publish_state(self):
        msg = String()
        msg.data = self.current_phase
        self.pub.publish(msg)

    def on_timer(self):
        self.elapsed += self.timer_period
        if self.elapsed >= self.durations[self.current_phase]:
            self.elapsed = 0.0
            self.phase_index = (self.phase_index + 1) % len(self.PHASES)
            self.get_logger().info(f'light -> {self.current_phase}')
            self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
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
