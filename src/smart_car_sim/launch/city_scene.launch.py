#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
城市场景智能小车仿真启动脚本。

基于 v2x_city_scene.world（城市沙盘），启动：
  - gzserver + gzclient
  - smart_car_spawner（生成 car_2 / car_3；car_1 已在 world 中内联）
  - 轨迹记录器
  - 3 辆小车的图案驾驶演示（circle / figure8 / forward）

用法：
    ros2 launch smart_car_sim city_scene.launch.py
    ros2 launch smart_car_sim city_scene.launch.py gui:=false
    ros2 launch smart_car_sim city_scene.launch.py pattern:=circle
    ros2 launch smart_car_sim city_scene.launch.py speed:=0.3
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

    world_path = os.path.join(pkg_sim, 'worlds', 'v2x_city_scene.world')
    car_config = os.path.join(pkg_sim, 'config', 'city_car_config.yaml')
    template_path = os.path.join(pkg_sim, 'models', 'smart_car', 'model.sdf.template')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pattern = LaunchConfiguration('pattern')
    speed = LaunchConfiguration('speed')
    demo_duration = LaunchConfiguration('demo_duration')
    record = LaunchConfiguration('record')

    # ---- 环境变量 ----
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

    # ---- 启动参数 ----
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true', description='是否启动 Gazebo GUI')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='使用仿真时间')
    declare_pattern = DeclareLaunchArgument(
        'pattern', default_value='circle',
        description='car_1 驾驶模式: square / circle / figure8 / forward')
    declare_speed = DeclareLaunchArgument(
        'speed', default_value='0.3',
        description='演示速度 (m/s)，城市场景空间小建议 ≤0.5')
    declare_duration = DeclareLaunchArgument(
        'demo_duration', default_value='60.0',
        description='演示时长 (s)')
    declare_record = DeclareLaunchArgument(
        'record', default_value='true', description='是否启动轨迹记录')

    # ---- Gazebo 服务器 / 客户端 ----
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

    # ---- 小车生成器（car_2 / car_3） ----
    spawner = Node(
        package='smart_car_sim',
        executable='smart_car_spawner.py',
        name='city_car_spawner',
        parameters=[{
            'config_file': car_config,
            'template_path': template_path,
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ---- 轨迹记录 ----
    trajectory_recorder = Node(
        package='smart_car_sim',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_name_regex': '^(car_|smart_car_|tb3_)',
            'output_file': os.path.expanduser('~/city_scene_trajectory.json'),
        }],
        condition=IfCondition(record),
        output='screen',
    )

    # ---- 图案驾驶演示 ----
    # car_1 已在 world 中内联，car_2 / car_3 由 spawner 生成
    demo_car_1 = Node(
        package='smart_car_sim',
        executable='smart_car_demo.py',
        name='city_demo_car_1',
        parameters=[{
            'car_name': 'car_1',
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
        name='city_demo_car_2',
        parameters=[{
            'car_name': 'car_2',
            'pattern': 'figure8',
            'speed': speed,
            'demo_duration': demo_duration,
            'enable_fault': False,
        }],
        output='screen',
    )

    demo_car_3 = Node(
        package='smart_car_sim',
        executable='smart_car_demo.py',
        name='city_demo_car_3',
        parameters=[{
            'car_name': 'car_3',
            'pattern': 'forward',
            'speed': speed,
            'demo_duration': demo_duration,
            'enable_fault': False,
        }],
        output='screen',
    )

    # ---- 分阶段延迟启动 ----
    # 4s: GUI 客户端
    # 6s: 车辆生成器（等待 gzserver 就绪）
    # 8s: 轨迹记录器
    # 12s: 演示节点（等待车辆生成完毕）
    stage1 = TimerAction(period=4.0, actions=[gzclient])
    stage2 = TimerAction(period=6.0, actions=[spawner])
    stage3 = TimerAction(period=8.0, actions=[trajectory_recorder])
    stage4 = TimerAction(period=12.0, actions=[demo_car_1, demo_car_2, demo_car_3])

    return LaunchDescription([
        declare_gui,
        declare_sim_time,
        declare_pattern,
        declare_speed,
        declare_duration,
        declare_record,
        set_model_path,
        set_plugin_path,
        gzserver,
        stage1,
        stage2,
        stage3,
        stage4,
    ])
