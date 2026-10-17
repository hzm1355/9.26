#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
信号灯配时仿真完整 launch —— 集成所有信号灯相关节点。

包含节点：
  1) traffic_light_manager  —— 多路口信号灯协调管理器
  2) adaptive_timing_node   —— 自适应配时算法节点
  3) traffic_light_recorder —— 信号灯状态记录器
  4) traffic_light_player   —— 信号灯状态回放器
  5) websocket_bridge       —— WebSocket 桥接（Web 可视化）
  6) signal_drive_node      —— 按灯行驶（车辆红灯停绿灯行 + 排队）

用法：
    ros2 launch v2x_demo traffic_signal_full.launch.py
    ros2 launch v2x_demo traffic_signal_full.launch.py enable_adaptive:=true
    ros2 launch v2x_demo traffic_signal_full.launch.py enable_recorder:=true enable_websocket:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_v2x = get_package_share_directory('v2x_demo')
    tl_params = os.path.join(pkg_v2x, 'config', 'traffic_light_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_adaptive = LaunchConfiguration('enable_adaptive')
    enable_recorder = LaunchConfiguration('enable_recorder')
    enable_player = LaunchConfiguration('enable_player')
    enable_websocket = LaunchConfiguration('enable_websocket')
    record_file = LaunchConfiguration('record_file')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='使用仿真时间')
    declare_adaptive = DeclareLaunchArgument(
        'enable_adaptive', default_value='true',
        description='启用自适应配时节点')
    declare_recorder = DeclareLaunchArgument(
        'enable_recorder', default_value='false',
        description='启用信号灯记录器')
    declare_player = DeclareLaunchArgument(
        'enable_player', default_value='false',
        description='启用信号灯回放器')
    declare_websocket = DeclareLaunchArgument(
        'enable_websocket', default_value='false',
        description='启用 WebSocket 桥接')
    declare_record_file = DeclareLaunchArgument(
        'record_file', default_value=os.path.expanduser('~/traffic_light_log.json'),
        description='信号灯记录输出文件')
    declare_signal_drive = DeclareLaunchArgument(
        'enable_signal_drive', default_value='true',
        description='启用按灯行驶节点（车辆红灯停绿灯行）')

    signal_drive = Node(
        package='v2x_demo',
        executable='signal_drive_node.py',
        name='signal_drive_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': os.path.join(pkg_v2x, 'config', 'signal_drive_params.yaml'),
            'path_csv': '/tmp/signal_drive_path.csv',
        }],
        condition=IfCondition(LaunchConfiguration('enable_signal_drive')),
        output='screen',
    )

    traffic_light_manager = Node(
        package='v2x_demo',
        executable='traffic_light_manager.py',
        name='traffic_light_manager',
        parameters=[{
            'config_file': tl_params,
            'use_sim_time': use_sim_time,
            'enable_statistics': True,
            'enable_time_schedule': True,
        }],
        output='screen',
    )

    # 自适应配时：每个信号相位对应一块"排队检测区"，节点按区内静止车辆数
    # 计算 adaptive_factor 并发布到 traffic_light/<group>/adaptive_factor
    direction_zones = """
NS_J1: {group: J1_ns, zones: [z_j1_ns], area: {x_min: -3.0, x_max: 3.0, y_min: -10.5, y_max: -7.0}}
EW_J1: {group: J1_ew, zones: [z_j1_ew], area: {x_min: -16.0, x_max: -11.5, y_min: 2.0, y_max: 7.0}}
NS_J2: {group: J2_ns, zones: [z_j2_ns], area: {x_min: -3.0, x_max: 3.0, y_min: -14.5, y_max: -11.5}}
EW_J2: {group: J2_ew, zones: [z_j2_ew], area: {x_min: 3.0, x_max: 10.0, y_min: -17.0, y_max: -12.0}}
NS_J3: {group: J3_ns, zones: [z_j3_ns], area: {x_min: -1.0, x_max: 4.0, y_min: 0.0, y_max: 3.0}}
EW_J3: {group: J3_ew, zones: [z_j3_ew], area: {x_min: -6.0, x_max: -2.5, y_min: 2.0, y_max: 7.0}}
"""

    adaptive_timing = Node(
        package='v2x_demo',
        executable='adaptive_timing_node.py',
        name='adaptive_timing_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'update_period': 5.0,
            'min_factor': 0.5,
            'max_factor': 2.0,
            'min_vehicles_for_adapt': 2,
            'direction_zones': direction_zones,
        }],
        condition=IfCondition(enable_adaptive),
        output='screen',
    )

    traffic_light_recorder = Node(
        package='v2x_demo',
        executable='traffic_light_recorder.py',
        name='traffic_light_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'output_file': record_file,
            'groups': ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew'],
            'record_detail': True,
        }],
        condition=IfCondition(enable_recorder),
        output='screen',
    )

    traffic_light_player = Node(
        package='v2x_demo',
        executable='traffic_light_recorder.py',
        name='traffic_light_player',
        arguments=['player'],
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_file': record_file,
            'playback_speed': 1.0,
            'loop': False,
        }],
        condition=IfCondition(enable_player),
        output='screen',
    )

    websocket_bridge = Node(
        package='v2x_demo',
        executable='websocket_bridge.py',
        name='websocket_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ws_host': '0.0.0.0',
            'ws_port': 8765,
            'groups': ['J1_ns', 'J1_ew', 'J2_ns', 'J2_ew', 'J3_ns', 'J3_ew'],
        }],
        condition=IfCondition(enable_websocket),
        output='screen',
    )

    return LaunchDescription([
        declare_sim_time,
        declare_adaptive,
        declare_recorder,
        declare_player,
        declare_websocket,
        declare_record_file,
        declare_signal_drive,
        traffic_light_manager,
        adaptive_timing,
        traffic_light_recorder,
        traffic_light_player,
        websocket_bridge,
        signal_drive,
    ])
