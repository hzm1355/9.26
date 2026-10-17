#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车完整演示 —— 在仿真基础上叠加多车驾驶演示与故障模拟。

用法：
    ros2 launch smart_car_sim full_demo.launch.py
    ros2 launch smart_car_sim full_demo.launch.py pattern:=circle
    ros2 launch smart_car_sim full_demo.launch.py demo_cars:="smart_car_1 smart_car_3"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('smart_car_sim')

    sim_launch = os.path.join(pkg_sim, 'launch', 'smart_car_sim.launch.py')

    pattern = LaunchConfiguration('pattern')
    speed = LaunchConfiguration('speed')
    demo_duration = LaunchConfiguration('demo_duration')

    declare_pattern = DeclareLaunchArgument(
        'pattern', default_value='square',
        description='驾驶模式: square / circle / figure8 / forward')
    declare_speed = DeclareLaunchArgument(
        'speed', default_value='0.5',
        description='演示速度 (m/s)')
    declare_duration = DeclareLaunchArgument(
        'demo_duration', default_value='60.0',
        description='演示时长 (s)')

    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'gui': 'true',
            'nav2': 'false',
            'record': 'true',
        }.items(),
    )

    demo_car_1 = Node(
        package='smart_car_sim',
        executable='smart_car_demo.py',
        name='smart_car_demo_1',
        parameters=[{
            'car_name': 'smart_car_1',
            'pattern': pattern,
            'speed': speed,
            'demo_duration': demo_duration,
            'enable_fault': False,
        }],
        output='screen',
    )

    demo_car_2 = Node(
        package='smart_car_sim',
        executable='smart_car_demo.py',
        name='smart_car_demo_2',
        parameters=[{
            'car_name': 'smart_car_2',
            'pattern': 'circle',
            'speed': speed,
            'demo_duration': demo_duration,
            'enable_fault': False,
        }],
        output='screen',
    )

    demo_car_3 = Node(
        package='smart_car_sim',
        executable='smart_car_demo.py',
        name='smart_car_demo_3',
        parameters=[{
            'car_name': 'smart_car_3',
            'pattern': 'figure8',
            'speed': speed,
            'demo_duration': demo_duration,
            'enable_fault': False,
        }],
        output='screen',
    )

    delayed_demo = TimerAction(period=12.0, actions=[
        demo_car_1, demo_car_2, demo_car_3,
    ])

    return LaunchDescription([
        declare_pattern,
        declare_speed,
        declare_duration,
        base_sim,
        delayed_demo,
    ])
