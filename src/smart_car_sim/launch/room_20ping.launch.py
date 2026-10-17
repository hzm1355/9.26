#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
room_20ping 户型仿真启动脚本 —— 加载 20平.fbx 转换的室内户型模型 + 智能小车。

用法：
    ros2 launch smart_car_sim room_20ping.launch.py
    ros2 launch smart_car_sim room_20ping.launch.py gui:=false
    ros2 launch smart_car_sim room_20ping.launch.py spawn_car:=false   # 只看户型，不生车
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
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_sim, 'worlds', 'room_20ping.world')
    car_config = os.path.join(pkg_sim, 'config', 'smart_car_config.yaml')
    template_path = os.path.join(pkg_sim, 'models', 'smart_car', 'model.sdf.template')

    gui = LaunchConfiguration('gui')
    spawn_car = LaunchConfiguration('spawn_car')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_sim, 'models') + ':' +
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
    declare_spawn_car = DeclareLaunchArgument(
        'spawn_car', default_value='true', description='是否生成智能小车')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='使用仿真时间')
    declare_config = DeclareLaunchArgument(
        'config_file', default_value=car_config, description='小车配置 YAML')

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
        condition=IfCondition(spawn_car),
    )

    delayed_nodes = TimerAction(period=8.0, actions=[
        gzclient,
        spawner,
    ])

    return LaunchDescription([
        declare_gui,
        declare_spawn_car,
        declare_sim_time,
        declare_config,
        set_model_path,
        set_plugin_path,
        gzserver,
        delayed_nodes,
    ])
