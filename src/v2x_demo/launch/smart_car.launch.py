#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车算法仿真启动脚本。

用法：
    ros2 launch v2x_demo smart_car.launch.py
    ros2 launch v2x_demo smart_car.launch.py config_file:=/path/to/cars.yaml
    ros2 launch v2x_demo smart_car.launch.py gui:=false   # headless
    ros2 launch v2x_demo smart_car.launch.py nav2:=true    # 同时启动 Nav2
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
    pkg_v2x = get_package_share_directory('v2x_demo')
    pkg_sim = get_package_share_directory('smart_car_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_v2x, 'worlds', 'v2x_city.world')
    car_config = os.path.join(pkg_v2x, 'config', 'smart_car_config.yaml')
    template_path = os.path.join(pkg_v2x, 'models', 'smart_car', 'model.sdf.template')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    nav2 = LaunchConfiguration('nav2')

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_v2x, 'models') + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''))

    plugin_paths = [os.path.join(get_package_prefix('smart_car_sim'), 'lib'),
                    os.path.join(get_package_prefix('v2x_gazebo_plugins'), 'lib')]
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
        parameters=[{'use_sim_time': use_sim_time}],
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
    ])

    nav2_delayed = TimerAction(period=15.0, actions=[nav2_bringup])

    return LaunchDescription([
        declare_gui,
        declare_sim_time,
        declare_config,
        declare_nav2,
        set_model_path,
        set_plugin_path,
        gzserver,
        delayed_nodes,
        nav2_delayed,
    ])
