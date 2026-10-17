#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2X demo + Nav2 自主导航（对应 SETUP.md 检查点 4）。

前置：先在另一终端启动仿真：
    ros2 launch v2x_demo v2x_bringup.launch.py

再启动本文件（SLAM 在线建图 + Nav2 导航栈）：
    ros2 launch v2x_demo v2x_nav2.launch.py

依赖：
    sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
说明：
    - 使用 slam_toolbox 在线异步建图，同时提供定位与地图；
    - nav2 navigation_launch 启动 controller/planner(NavFn=A*/Dijkstra)/bt_navigator；
    - 车辆话题为全局 /scan /odom /cmd_vel，Nav2 可直接使用；
    - 在 RViz 中用 "Nav2 Goal" 下发目标点即可自主导航。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_bringup = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    nav2_bringup = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    # 自定义 Nav2 参数：基于官方默认参数，修补 local_costmap 体素层 z 范围，
    # 使其覆盖激光高度（车顶 z≈1.72m，默认 [0, 0.8] 会导致雷达无法 raytrace）。
    nav2_params_file = os.path.join(
        get_package_share_directory('v2x_demo'), 'config', 'v2x_nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # SLAM 在线建图 + 定位
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_bringup),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Nav2 导航栈（用自定义 params_file）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={'use_sim_time': use_sim_time, 'autostart': 'true',
                              'params_file': nav2_params_file}.items(),
        ),

        # RViz（可选，use_rviz:=false 可关；VcXsrv 不可用时必须关，
        # 否则 rviz 连不上 X 会 SIGABRT，连带 launch 发 SIGINT 关掉整个 Nav2 栈）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'),
                             'launch', 'rviz_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'rviz_config_dir': rviz_config_dir}.items(),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
