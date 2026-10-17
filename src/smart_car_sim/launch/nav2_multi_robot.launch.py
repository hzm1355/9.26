#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多机器人 Nav2 导航启动脚本。

为配置中的每辆智能小车启动独立的 Nav2 导航栈，自动处理命名空间隔离。
复用 TurtleBot3 navigation2 的启动模式，适配多机器人场景。

用法：
    ros2 launch smart_car_sim nav2_multi_robot.launch.py
    ros2 launch smart_car_sim nav2_multi_robot.launch.py map:=/path/to/map.yaml
    ros2 launch smart_car_sim nav2_multi_robot.launch.py robots:="tb3_1 tb3_2"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            PushRosNamespace)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nav2_nodes_for_robot(context: LaunchContext, robot_name, params_file,
                          map_yaml, use_sim_time):
    """为单个机器人创建 Nav2 节点组。"""
    return GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=f'lifecycle_manager_navigation_{robot_name}',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': [
                             'amcl',
                             'controller_server',
                             'planner_server',
                             'behavior_server',
                             'bt_navigator',
                             'recoveries_server',
                             'smoother_server',
                             'waypoint_follower',
                             'velocity_smoother',
                         ]}],
        ),
    ])


def _generate_nodes(context: LaunchContext):
    """OpaqueFunction 回调：解析 robots 参数，为每个机器人生成 Nav2 节点。"""
    params_file = context.launch_configurations.get('params_file', '')
    map_yaml = context.launch_configurations.get('map', '')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'true') == 'true'
    robots_raw = context.launch_configurations.get('robots', 'tb3_1 tb3_2')

    actions = []
    for name in robots_raw.split():
        name = name.strip()
        if name:
            actions.append(
                _nav2_nodes_for_robot(context, name, params_file, map_yaml,
                                      use_sim_time))
    return actions


def generate_launch_description():
    pkg_sim = get_package_share_directory('smart_car_sim')

    default_map = os.path.join(pkg_sim, 'worlds', 'map.yaml')
    default_params = os.path.join(pkg_sim, 'config', 'nav2_tb3_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='地图 YAML 路径'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 参数文件'),
        DeclareLaunchArgument(
            'robots', default_value='tb3_1 tb3_2',
            description='需要启动 Nav2 的机器人名称列表（空格分隔）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用仿真时间'),
        OpaqueFunction(function=_generate_nodes),
    ])
