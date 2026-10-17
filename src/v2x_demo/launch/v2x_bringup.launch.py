#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V2X demo bringup：启动 Gazebo Classic（十字路口 world）+ 生成 prius 车辆 + 信号灯配时控制器。

用法：
    ros2 launch v2x_demo v2x_bringup.launch.py
    ros2 launch v2x_demo v2x_bringup.launch.py gui:=false   # 无 GUI（headless）
"""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_v2x = get_package_share_directory('v2x_demo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_v2x, 'worlds', 'v2x_intersection.world')
    model_path = os.path.join(pkg_v2x, 'models', 'v2x_prius', 'model.sdf')
    tl_params = os.path.join(pkg_v2x, 'config', 'traffic_light_params.yaml')
    flow_params = os.path.join(pkg_v2x, 'config', 'traffic_flow.yaml')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    traffic_flow = LaunchConfiguration('traffic_flow')
    record_traj = LaunchConfiguration('record_trajectory')
    traj_out = LaunchConfiguration('trajectory_out')
    enable_adaptive = LaunchConfiguration('enable_adaptive')

    # 让 Gazebo 找到本包模型（若未通过 package.xml 自动注入）
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_v2x, 'models') + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', ''))

    # 让 Gazebo 找到信号灯视觉点亮插件 libv2x_traffic_light.so
    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(get_package_prefix('v2x_gazebo_plugins'), 'lib') + ':' +
              os.environ.get('GAZEBO_PLUGIN_PATH', ''))

    declare_gui = DeclareLaunchArgument('gui', default_value='true',
                                        description='是否启动 Gazebo GUI 客户端')
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                                             description='使用仿真时间')
    declare_traffic_flow = DeclareLaunchArgument('traffic_flow', default_value='true',
                                                 description='是否启动多车交通流')
    declare_record_traj = DeclareLaunchArgument('record_trajectory', default_value='true',
                                                description='是否启动轨迹记录器')
    declare_traj_out = DeclareLaunchArgument('trajectory_out',
                                             default_value=os.path.expanduser('~/v2x_trajectory.json'),
                                             description='轨迹记录输出文件')
    declare_adaptive = DeclareLaunchArgument('enable_adaptive', default_value='true',
                                             description='启用自适应配时节点')

    # Gazebo server + client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path,
                          'verbose': 'true',
                          'pause': 'false'}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # 生成 prius 车辆到世界原点南侧路口（yaw=π：模型车头为 body -Y，
    # 需 yaw=π 才能让车头朝北进入路口；yaw=π/2 会让车头朝东）
    spawn_prius = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_v2x_prius',
        arguments=['-entity', 'v2x_prius',
                   '-file', model_path,
                   '-x', '0.0', '-y', '-15.0', '-z', '0.4',
                   '-Y', '3.14159'],
        output='screen',
    )

    # 信号灯协调管理器（多路口联动 + 视觉点亮）：读取 config_file 里的 groups/presets，
    # 每组发布 traffic_light/<group>/state，驱动 v2x_traffic_light 插件点亮灯球。
    traffic_light_manager = Node(
        package='v2x_demo',
        executable='traffic_light_manager.py',
        name='traffic_light_manager',
        parameters=[{'config_file': tl_params, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    # chassis->base_link：slam_toolbox / Nav2 默认 robot_base_frame=base_link，
    # 而本车 ackermann 插件发布 odom->chassis，故需补 chassis->base_link。
    # ⚠️ 本模型车头是 body -Y（前轮 y=-1.41、轮转轴沿 X），ROS 约定 base_link+X=车头，
    # 故必须加 yaw -90°，使 base_link+X 指向 chassis -Y（车头方向）；
    # 零旋转会让车“侧向横移”，Nav2 全部几何假设错位（实测：cmd_vel+X 车向东横移）。
    base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_broadcaster',
        arguments=['0', '0', '0', '-1.5708', '0', '0', 'chassis', 'base_link'],
        # 必须用仿真时间戳，否则墙钟戳的静态 TF 与 sim time 的 laser 消息
        # 时间不匹配，message filter 会丢弃 laser 导致 slam 建不出 map。
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # laser 帧静态 TF：lidar 是 sensor 非 link，Gazebo 不为它发 TF，而 /scan 的
    # frame_id=laser（见 model.sdf），slam/costmap 需 base_link->laser 才能用扫描，
    # 故按 lidar 安装偏移 (0,0,1.5) 补 chassis->laser。
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_broadcaster',
        arguments=['0', '0', '1.5', '0', '0', '0', 'chassis', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # base_footprint 帧：slam_toolbox 默认 base_frame=base_footprint（见
    # mapper_params_online_async.yaml），会去查 odom->base_footprint；本车无此帧时
    # 每帧报 "Failed to compute odom pose" 导致 /map 不发布。按 REP-105 补一条
    # base_link->base_footprint 零偏移静态 TF 即可打通该查询链。
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 区域触发公共底座（测速/卡口/停车场/ETC 共用）：订阅 /model_states，
    # 发布 /v2x/region_events 与每区域占用状态；区域定义见 config/regions.yaml。
    region_trigger = Node(
        package='v2x_demo',
        executable='region_trigger_node.py',
        name='region_trigger',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 多车交通流（协议 §三.(一).5 多车）：按 config/traffic_flow.yaml 生成多辆车并沿路点巡航。
    traffic_flow_node = Node(
        package='v2x_demo',
        executable='traffic_flow_node.py',
        name='traffic_flow',
        parameters=[{'config_file': flow_params, 'use_sim_time': use_sim_time}],
        condition=IfCondition(traffic_flow),
        output='screen',
    )

    # 轨迹记录器：订阅 /model_states 记录所有 v2x_* 车轨迹，保存到 trajectory_out。
    trajectory_recorder = Node(
        package='smart_car_sim',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        parameters=[{'output_file': traj_out, 'use_sim_time': use_sim_time}],
        condition=IfCondition(record_traj),
        output='screen',
    )

    # 自适应配时节点：基于车流量动态调整信号灯周期。
    adaptive_timing_node = Node(
        package='v2x_demo',
        executable='adaptive_timing_node.py',
        name='adaptive_timing_node',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_adaptive),
        output='screen',
    )

    # ⭐ 错峰启动：先只起 gzserver，延迟 8s 再起其余节点。
    # 原因：WSL2 + FastDDS 下，若 gzserver 与 spawn_entity / traffic_light /
    # 多个 static_transform_publisher 同时创建 ROS 节点，gazebo_ros_init 在
    # 初始化 DDS 时会与它们并发发现相互竞争，导致 gzserver 段错误(SIGSEGV, exit -11)。
    # 实测：官方 gzserver.launch.py 单独跑不崩，加入并发节点则必崩。
    # 让 gzserver 先独占完成初始化（world 加载约 2~3s），8s 后其余节点再加入即可规避。
    delayed_nodes = TimerAction(period=8.0, actions=[
        gzclient,
        spawn_prius,
        traffic_light_manager,
        base_link_tf,
        laser_tf,
        base_footprint_tf,
        region_trigger,
        traffic_flow_node,
        trajectory_recorder,
        adaptive_timing_node,
    ])

    return LaunchDescription([
        declare_gui,
        declare_sim_time,
        declare_traffic_flow,
        declare_record_traj,
        declare_traj_out,
        declare_adaptive,
        set_model_path,
        set_plugin_path,
        gzserver,
        delayed_nodes,
    ])
