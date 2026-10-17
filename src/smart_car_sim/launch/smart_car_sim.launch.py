#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车算法仿真 —— 独立应用启动脚本。

对应需求（一）ROS2智能小车算法仿真模块：
  1. 多车型可配置（尺寸/轴距/最高速度/加速度）
  2. 虚拟传感器仿真（LiDAR/Camera/IMU/Odom），标准 ROS2 话题，兼容 Nav2
  3. Nav2 导航算法验证（A*/Dijkstra/航点巡航/多车协调/避障）
  4. 支持导入自定义 ROS2 包到 Gazebo
  5. 速度/加速度/优先级可配；多车同时运行；轨迹/速度记录；场景回放
  6. 插件化故障模拟（传感器失效/数据延迟/位姿漂移）

用法：
    ros2 launch smart_car_sim smart_car_sim.launch.py
    ros2 launch smart_car_sim smart_car_sim.launch.py gui:=false
    ros2 launch smart_car_sim smart_car_sim.launch.py nav2:=true
    ros2 launch smart_car_sim smart_car_sim.launch.py config_file:=/path/to/cars.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_sim = get_package_share_directory('smart_car_sim')
    pkg_v2x = get_package_share_directory('v2x_demo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_sim, 'worlds', 'algorithm_arena.world')
    car_config = os.path.join(pkg_sim, 'config', 'smart_car_config.yaml')
    regions_config = os.path.join(pkg_sim, 'config', 'arena_regions.yaml')
    template_path = os.path.join(pkg_sim, 'models', 'smart_car', 'model.sdf.template')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    nav2 = LaunchConfiguration('nav2')
    record = LaunchConfiguration('record')

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_sim, 'models') + ':' +
              os.path.join(pkg_v2x, 'models') + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''))

    plugin_paths = [os.path.join(get_package_prefix('smart_car_sim'), 'lib')]
    try:
        plugin_paths.insert(0, '/opt/ros/humble/lib')
    except Exception:
        pass
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=':'.join(plugin_paths) + ':' +
              os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='是否启动 Gazebo GUI')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='使用仿真时间')
    declare_config = DeclareLaunchArgument(
        'config_file', default_value=car_config, description='小车配置 YAML')
    declare_nav2 = DeclareLaunchArgument(
        'nav2', default_value='false', description='是否同时启动 Nav2')
    declare_record = DeclareLaunchArgument(
        'record', default_value='true', description='是否启动轨迹记录')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true', 'pause': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    spawner = Node(
        package='smart_car_sim',
        executable='smart_car_spawner.py',
        name='smart_car_spawner',
        parameters=[{
            'config_file': config_file,
            'template_path': template_path,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    region_trigger = Node(
        package='v2x_demo',
        executable='region_trigger_node.py',
        name='region_trigger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'regions_file': regions_config,
            'vehicle_name_regex': '^(smart_car_|tb3_)',
        }],
        output='screen',
    )

    trajectory_recorder = Node(
        package='smart_car_sim',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_name_regex': '^(smart_car_|tb3_)',
            'output_file': os.path.expanduser('~/smart_car_trajectory.json'),
        }],
        condition=IfCondition(record),
        output='screen',
    )

    trajectory_player = Node(
        package='smart_car_sim',
        executable='trajectory_player.py',
        name='trajectory_player',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_file': os.path.expanduser('~/smart_car_trajectory.json'),
        }],
        output='screen',
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'nav2_multi_robot.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(nav2),
    )

    delayed_nodes = TimerAction(period=8.0, actions=[
        gzclient,
        spawner,
        region_trigger,
        trajectory_recorder,
        trajectory_player,
    ])

    nav2_delayed = TimerAction(period=15.0, actions=[nav2_bringup])

    return LaunchDescription([
        declare_gui,
        declare_sim_time,
        declare_config,
        declare_nav2,
        declare_record,
        set_model_path,
        set_plugin_path,
        gzserver,
        delayed_nodes,
        nav2_delayed,
    ])
