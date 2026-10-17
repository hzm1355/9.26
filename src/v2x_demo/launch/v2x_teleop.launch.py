#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2X demo 遥控：键盘遥控 prius 车辆（发布到全局 /cmd_vel）。

用法（在单独终端运行）：
    ros2 launch v2x_demo v2x_teleop.launch.py

依赖：sudo apt install ros-humble-teleop-twist-keyboard
按键：i/,（前进/后退）、j/l（左转/右转）、k（停止）。
速度/角速度示例：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=5.0 -p turn:=0.8
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                 '--ros-args', '-p', 'speed:=3.0', '-p', 'turn:=0.6'],
            output='screen',
            prefix='xterm -e',
        ),
    ])
