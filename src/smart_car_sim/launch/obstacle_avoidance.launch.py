#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
避障节点启动脚本。

用法：
    ros2 launch smart_car_sim obstacle_avoidance.launch.py
    ros2 launch smart_car_sim obstacle_avoidance.launch.py car_name:=smart_car_1
    ros2 launch smart_car_sim obstacle_avoidance.launch.py max_speed:=0.5 stop_distance:=0.8
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    car_name = LaunchConfiguration('car_name')
    max_speed = LaunchConfiguration('max_speed')
    turn_speed = LaunchConfiguration('turn_speed')
    stop_distance = LaunchConfiguration('stop_distance')
    slow_distance = LaunchConfiguration('slow_distance')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_car_name = DeclareLaunchArgument(
        'car_name', default_value='smart_car_1',
        description='小车名称（话题前缀）')
    declare_max_speed = DeclareLaunchArgument(
        'max_speed', default_value='0.5',
        description='最大前进速度 (m/s)')
    declare_turn_speed = DeclareLaunchArgument(
        'turn_speed', default_value='0.8',
        description='转向速度 (rad/s)')
    declare_stop_distance = DeclareLaunchArgument(
        'stop_distance', default_value='0.8',
        description='停止距离阈值 (m)')
    declare_slow_distance = DeclareLaunchArgument(
        'slow_distance', default_value='1.5',
        description='减速距离阈值 (m)')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时间')

    obstacle_avoidance_node = Node(
        package='smart_car_sim',
        executable='obstacle_avoidance.py',
        name='obstacle_avoidance',
        parameters=[{
            'car_name': car_name,
            'max_speed': max_speed,
            'turn_speed': turn_speed,
            'stop_distance': stop_distance,
            'slow_distance': slow_distance,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_car_name,
        declare_max_speed,
        declare_turn_speed,
        declare_stop_distance,
        declare_slow_distance,
        declare_sim_time,
        obstacle_avoidance_node,
    ])
