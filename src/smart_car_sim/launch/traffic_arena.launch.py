#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交通路口仿真启动脚本 —— 含信号灯、行人灯、自适应配时、WebSocket 桥接。

用法：
    ros2 launch smart_car_sim traffic_arena.launch.py
    ros2 launch smart_car_sim traffic_arena.launch.py gui:=false
    ros2 launch smart_car_sim traffic_arena.launch.py adaptive:=true
    ros2 launch smart_car_sim traffic_arena.launch.py websocket:=true

启动后，打开 web/index.html 可在浏览器中查看 3D 可视化。
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

    world_path = os.path.join(pkg_sim, 'worlds', 'traffic_arena.world')
    car_config = os.path.join(pkg_sim, 'config', 'smart_car_config.yaml')
    regions_config = os.path.join(pkg_sim, 'config', 'traffic_arena_regions.yaml')
    traffic_params = os.path.join(pkg_v2x, 'config', 'traffic_light_params.yaml')
    template_path = os.path.join(pkg_sim, 'models', 'smart_car', 'model.sdf.template')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    adaptive = LaunchConfiguration('adaptive')
    websocket = LaunchConfiguration('websocket')
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
    declare_adaptive = DeclareLaunchArgument(
        'adaptive', default_value='true', description='是否启动自适应配时')
    declare_websocket = DeclareLaunchArgument(
        'websocket', default_value='true', description='是否启动 WebSocket 桥接')
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

    traffic_light_manager = Node(
        package='v2x_demo',
        executable='traffic_light_manager.py',
        name='traffic_light_manager',
        parameters=[traffic_params, {
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    adaptive_timing = Node(
        package='v2x_demo',
        executable='adaptive_timing_node.py',
        name='adaptive_timing',
        parameters=[{
            'use_sim_time': use_sim_time,
            'groups': ['J1_ns', 'J1_ew'],
        }],
        condition=IfCondition(adaptive),
        output='screen',
    )

    traffic_recorder = Node(
        package='v2x_demo',
        executable='traffic_light_recorder.py',
        name='traffic_light_recorder',
        parameters=[{
            'use_sim_time': use_sim_time,
            'groups': ['J1_ns', 'J1_ew'],
            'output_file': os.path.expanduser('~/traffic_light_recording.json'),
        }],
        condition=IfCondition(record),
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
            'groups': ['J1_ns', 'J1_ew'],
        }],
        condition=IfCondition(websocket),
        output='screen',
    )

    delayed_nodes = TimerAction(period=8.0, actions=[
        gzclient,
        spawner,
        region_trigger,
        traffic_light_manager,
        adaptive_timing,
        traffic_recorder,
        websocket_bridge,
    ])

    return LaunchDescription([
        declare_gui,
        declare_sim_time,
        declare_config,
        declare_adaptive,
        declare_websocket,
        declare_record,
        set_model_path,
        set_plugin_path,
        gzserver,
        delayed_nodes,
    ])
