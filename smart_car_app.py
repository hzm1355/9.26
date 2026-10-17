#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 智能小车算法仿真应用 - 统一入口

功能模块：
  1. 多车型可配置（尺寸/轴距/最高速度/加速度）
  2. 虚拟传感器仿真（LiDAR/Camera/IMU/Odom）
  3. Nav2 导航算法验证
  4. 支持导入自定义 ROS2 包
  5. 轨迹记录与回放
  6. 故障模拟

使用方法：
  python3 smart_car_app.py start              # 启动完整仿真
  python3 smart_car_app.py start --no-gui     # 无界面模式
  python3 smart_car_app.py start --nav2       # 启动 Nav2
  python3 smart_car_app.py status             # 查看状态
  python3 smart_car_app.py config             # 查看配置
  python3 smart_car_app.py fault <车辆> <类型> # 注入故障
  python3 smart_car_app.py record start/stop  # 轨迹记录
  python3 smart_car_app.py playback <文件>    # 轨迹回放
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path


class SmartCarApp:
    """智能小车仿真应用主类"""
    
    def __init__(self):
        self.workspace = Path(__file__).parent
        self.config_dir = self.workspace / 'src' / 'smart_car_sim' / 'config'
        self.default_config = self.config_dir / 'smart_car_config.yaml'
        
    def run_command(self, cmd, check=True):
        """运行 shell 命令"""
        print(f"[执行] {' '.join(cmd)}")
        result = subprocess.run(cmd, cwd=self.workspace)
        if check and result.returncode != 0:
            print(f"[错误] 命令失败，返回码: {result.returncode}")
            sys.exit(1)
        return result.returncode
    
    def start(self, args):
        """启动仿真"""
        print("=" * 60)
        print("  ROS2 智能小车算法仿真应用")
        print("=" * 60)
        
        # 构建 launch 命令
        cmd = ['ros2', 'launch', 'smart_car_sim', 'smart_car_sim.launch.py']
        
        if args.no_gui:
            cmd.append('gui:=false')
        
        if args.nav2:
            cmd.append('nav2:=true')
        
        if args.config:
            cmd.append(f'config_file:={args.config}')
        
        if args.no_record:
            cmd.append('record:=false')
        
        print(f"\n[启动] 仿真环境...")
        print(f"  - 配置文件: {args.config or self.default_config}")
        print(f"  - GUI: {'否' if args.no_gui else '是'}")
        print(f"  - Nav2: {'是' if args.nav2 else '否'}")
        print(f"  - 轨迹记录: {'否' if args.no_record else '是'}")
        print()
        
        self.run_command(cmd)
    
    def status(self, args):
        """查看系统状态"""
        print("=" * 60)
        print("  系统状态")
        print("=" * 60)
        
        # 检查 ROS2
        try:
            result = subprocess.run(['ros2', '--version'], 
                                  capture_output=True, text=True, timeout=5)
            print(f"✓ ROS2 版本: {result.stdout.strip()}")
        except:
            print("✗ ROS2 未安装或未配置")
            return
        
        # 检查 Gazebo
        try:
            result = subprocess.run(['gz', '--version'], 
                                  capture_output=True, text=True, timeout=5)
            print(f"✓ Gazebo: {result.stdout.strip().split()[0]}")
        except:
            print("✗ Gazebo 未安装")
        
        # 检查运行中的节点
        print("\n[运行中的节点]")
        self.run_command(['ros2', 'node', 'list'], check=False)
        
        # 检查话题
        print("\n[活跃话题（前 20 个）]")
        self.run_command(['ros2', 'topic', 'list', '--count', '20'], check=False)
        
        # 检查车辆
        print("\n[仿真中的车辆]")
        self.run_command(['ros2', 'topic', 'echo', '/model_states', '--once', 
                         '--field', 'name'], check=False)
    
    def config(self, args):
        """查看或编辑配置"""
        print("=" * 60)
        print("  车辆配置")
        print("=" * 60)
        
        config_file = args.config or self.default_config
        
        if not os.path.exists(config_file):
            print(f"[错误] 配置文件不存在: {config_file}")
            return
        
        with open(config_file, 'r', encoding='utf-8') as f:
            config = json.load(f) if config_file.endswith('.json') else f.read()
        
        if args.edit:
            editor = os.environ.get('EDITOR', 'nano')
            subprocess.run([editor, config_file])
        else:
            print(f"\n配置文件: {config_file}\n")
            if isinstance(config, dict):
                print(json.dumps(config, indent=2, ensure_ascii=False))
            else:
                print(config)
    
    def fault(self, args):
        """注入故障"""
        vehicle = args.vehicle
        fault_type = args.fault_type
        
        valid_types = ['NONE', 'SENSOR_FAIL', 'DATA_LAG', 'POSE_DRIFT', 'ALL']
        if fault_type not in valid_types:
            print(f"[错误] 无效故障类型: {fault_type}")
            print(f"  可选: {', '.join(valid_types)}")
            return
        
        print(f"[故障注入] 车辆: {vehicle}, 类型: {fault_type}")
        
        # 发布故障命令
        cmd = ['ros2', 'topic', 'pub', '--once', 
               f'/{vehicle}/fault_command', 'std_msgs/msg/String',
               f'{{data: "{fault_type}"}}']
        
        self.run_command(cmd)
    
    def record(self, args):
        """轨迹记录控制"""
        action = args.action
        
        if action == 'start':
            print("[轨迹记录] 开始记录")
            self.run_command(['ros2', 'service', 'call', 
                            '/trajectory_recorder/start', 'std_srvs/srv/Trigger'])
        elif action == 'stop':
            print("[轨迹记录] 停止记录")
            self.run_command(['ros2', 'service', 'call',
                            '/trajectory_recorder/stop', 'std_srvs/srv/Trigger'])
        elif action == 'save':
            print("[轨迹记录] 保存轨迹")
            self.run_command(['ros2', 'service', 'call',
                            '/trajectory_recorder/save', 'std_srvs/srv/Trigger'])
        elif action == 'clear':
            print("[轨迹记录] 清空缓冲")
            self.run_command(['ros2', 'service', 'call',
                            '/trajectory_recorder/clear', 'std_srvs/srv/Trigger'])
    
    def playback(self, args):
        """轨迹回放"""
        file_path = args.file
        
        if not os.path.exists(file_path):
            print(f"[错误] 文件不存在: {file_path}")
            return
        
        print(f"[轨迹回放] 文件: {file_path}")
        
        # 调用 trajectory_player 节点
        cmd = ['ros2', 'run', 'smart_car_sim', 'trajectory_player.py',
               '--ros-args', '-p', f'input_file:={file_path}']
        
        self.run_command(cmd)
    
    def obstacle_avoidance(self, args):
        """启动避障节点"""
        car_name = args.car_name
        max_speed = args.max_speed
        stop_distance = args.stop_distance

        print(f"[避障] 启动避障节点：car={car_name} speed={max_speed} stop_dist={stop_distance}")
        print(f"[避障] 话题：/{car_name}/scan -> /{car_name}/cmd_vel")
        print(f"[避障] 按 Ctrl+C 停止\n")

        cmd = ['ros2', 'launch', 'smart_car_sim', 'obstacle_avoidance.launch.py',
               f'car_name:={car_name}',
               f'max_speed:={max_speed}',
               f'stop_distance:={stop_distance}']

        self.run_command(cmd)

    def info(self, args):
        """显示帮助信息"""
        print("""
╔═══════════════════════════════════════════════════════════╗
║         ROS2 智能小车算法仿真应用 - 使用指南             ║
╚═══════════════════════════════════════════════════════════╝

功能模块：
  1. 虚拟环境内置多台 ROS2 智能小车模型
     - 可配置尺寸、轮距、最大速度、加速度等物理参数
     - 预设车型：delivery_bot, medium_car, bus, patrol_car, tb3_burger, tb3_waffle_pi

  2. 虚拟传感器仿真
     - 激光雷达（LiDAR）：360° 扫描，可配置采样点数、范围、噪声
     - 摄像头（Camera）：可配置分辨率、帧率、视场角
     - IMU：200Hz 更新频率
     - 里程计（Odom）：通过 gazebo_ros_p3d 插件提供

  3. Nav2 导航算法验证
     - 兼容 A*、Dijkstra 等路径规划算法
     - 支持定点巡航、多车协同、避障实验
     - 启动命令：python3 smart_car_app.py start --nav2

  4. 避障控制（新增）
     - 基于 LiDAR 的实时避障
     - 自动检测前方/侧方障碍物
     - 智能减速、转向、停止
     - 启动命令：python3 smart_car_app.py avoid <车辆名>

  5. 支持导入自定义 ROS2 功能包
     - 将自定义包放入 src/ 目录
     - 编译：colcon build --packages-select <package_name>
     - 在 launch 文件中 Include 自定义节点

  6. 多车协同与轨迹记录
     - 多车同时运行，模拟车流、拥堵场景
     - 轨迹记录：python3 smart_car_app.py record start/stop/save
     - 轨迹回放：python3 smart_car_app.py playback <file.json>

  7. 故障模拟
     - 传感器失效：SENSOR_FAIL
     - 数据卡顿：DATA_LAG
     - 位姿偏移：POSE_DRIFT
     - 命令：python3 smart_car_app.py fault <车辆名> <故障类型>

常用命令：
  start              启动仿真
  start --nav2       启动仿真 + Nav2
  start --no-gui     无界面模式
  avoid <车>         启动避障（默认 smart_car_1）
  avoid <车> --speed 0.3  自定义速度
  status             查看系统状态
  config             查看车辆配置
  config --edit      编辑配置
  fault <车> <类型>  注入故障
  record start       开始记录轨迹
  record stop        停止记录
  record save        保存轨迹
  playback <文件>    回放轨迹

示例：
  python3 smart_car_app.py start
  python3 smart_car_app.py start --nav2 --no-gui
  python3 smart_car_app.py avoid smart_car_1
  python3 smart_car_app.py avoid smart_car_1 --speed 0.3 --stop 1.0
  python3 smart_car_app.py fault smart_car_1 SENSOR_FAIL
  python3 smart_car_app.py record start
  python3 smart_car_app.py playback ~/smart_car_trajectory.json
""")


def main():
    parser = argparse.ArgumentParser(
        description='ROS2 智能小车算法仿真应用',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        add_help=False)
    
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # start 命令
    start_parser = subparsers.add_parser('start', help='启动仿真')
    start_parser.add_argument('--no-gui', action='store_true', help='无界面模式')
    start_parser.add_argument('--nav2', action='store_true', help='启动 Nav2')
    start_parser.add_argument('--config', type=str, help='自定义配置文件')
    start_parser.add_argument('--no-record', action='store_true', help='禁用轨迹记录')
    
    # status 命令
    subparsers.add_parser('status', help='查看系统状态')
    
    # config 命令
    config_parser = subparsers.add_parser('config', help='查看配置')
    config_parser.add_argument('--config', type=str, help='配置文件路径')
    config_parser.add_argument('--edit', action='store_true', help='编辑配置')
    
    # fault 命令
    fault_parser = subparsers.add_parser('fault', help='注入故障')
    fault_parser.add_argument('vehicle', type=str, help='车辆名称')
    fault_parser.add_argument('fault_type', type=str, 
                             choices=['NONE', 'SENSOR_FAIL', 'DATA_LAG', 'POSE_DRIFT', 'ALL'],
                             help='故障类型')
    
    # record 命令
    record_parser = subparsers.add_parser('record', help='轨迹记录')
    record_parser.add_argument('action', type=str, 
                              choices=['start', 'stop', 'save', 'clear'],
                              help='操作类型')
    
    # playback 命令
    playback_parser = subparsers.add_parser('playback', help='轨迹回放')
    playback_parser.add_argument('file', type=str, help='轨迹文件路径')

    # avoid 命令（避障）
    avoid_parser = subparsers.add_parser('avoid', help='启动避障节点')
    avoid_parser.add_argument('car_name', type=str, nargs='?', default='smart_car_1',
                              help='车辆名称（默认 smart_car_1）')
    avoid_parser.add_argument('--speed', type=float, default=0.5,
                              help='最大速度 m/s（默认 0.5）')
    avoid_parser.add_argument('--stop', type=float, default=0.8,
                              help='停止距离 m（默认 0.8）')

    # info 命令
    subparsers.add_parser('info', help='显示帮助信息')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        print("\n使用 'python3 smart_car_app.py info' 查看详细帮助")
        return
    
    app = SmartCarApp()

    if args.command == 'start':
        app.start(args)
    elif args.command == 'status':
        app.status(args)
    elif args.command == 'config':
        app.config(args)
    elif args.command == 'fault':
        app.fault(args)
    elif args.command == 'record':
        app.record(args)
    elif args.command == 'playback':
        app.playback(args)
    elif args.command == 'avoid':
        # 设置避障参数
        args.max_speed = args.speed
        args.stop_distance = args.stop
        app.obstacle_avoidance(args)
    elif args.command == 'info':
        app.info(args)


if __name__ == '__main__':
    main()
