#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车生成管理器 —— 读取 smart_car_config.yaml，动态生成 SDF 并注入 Gazebo。

职责：
  1) 按配置生成多辆智能小车（LiDAR/Camera/IMU/Odom 全套传感器）
  2) 支持两种模型来源：template（参数化生成）和 prebuilt（复用 TurtleBot3 等现成模型）
  3) 每辆车独立命名空间 -> /<name>/cmd_vel, /<name>/scan, /<name>/imu/data 等
  4) 发布各车所需的静态 TF（chassis -> sensor frames）
  5) 兼容 Nav2 导航栈（标准话题 + TF 树）
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import rclpy
import yaml
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

# 确保能导入同目录的 car_generator 模块
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from car_generator import generate_sdf, get_template_path, load_car_config


def yaw_to_quat(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0))


def _find_model_sdf(model_name):
    """在 GAZEBO_MODEL_PATH 中查找预置模型的 SDF 文件。"""
    search_paths = os.environ.get('GAZEBO_MODEL_PATH', '').split(':')
    try:
        from ament_index_python.packages import get_package_share_directory
        search_paths.insert(0, os.path.join(get_package_share_directory('smart_car_sim'), 'models'))
    except Exception:
        pass
    for base in search_paths:
        if not base:
            continue
        candidate = os.path.join(base, model_name, 'model.sdf')
        if os.path.isfile(candidate):
            return candidate
    return None


def _make_namespaced_sdf(sdf_path, robot_name, namespace):
    """
    读取预置 SDF 并修改帧名以实现多机器人命名空间隔离。
    复用 TurtleBot3 multi_robot.launch.py 的模式，并为缺少显式 frame_name 的
    传感器插件补充命名空间帧名，确保消息 frame_id 与 TF 树一致。
    """
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model_elem = root.find('model')
    if model_elem is not None:
        model_elem.set('name', robot_name)

    for tag in root.iter('odometry_frame'):
        tag.text = f'{namespace}/odom'
    for tag in root.iter('robot_base_frame'):
        tag.text = f'{namespace}/base_footprint'
    for tag in root.iter('frame_name'):
        if tag.text and not tag.text.startswith(f'{namespace}/'):
            tag.text = f'{namespace}/{tag.text}'

    _ROS_PLUGIN_FRAMES = {
        'libgazebo_ros_imu_sensor.so': 'imu_link',
        'libgazebo_ros_camera.so': 'camera_link',
        'libgazebo_ros_ray_sensor.so': 'base_scan',
    }
    for plugin in root.iter('plugin'):
        filename = plugin.get('filename', '')
        if filename in _ROS_PLUGIN_FRAMES:
            fn_elem = plugin.find('frame_name')
            if fn_elem is None:
                default_frame = _ROS_PLUGIN_FRAMES[filename]
                fn_elem = ET.SubElement(plugin, 'frame_name')
                fn_elem.text = f'{namespace}/{default_frame}'

    xml_str = ET.tostring(root, encoding='unicode')
    return '<?xml version="1.0" ?>\n' + xml_str


class SmartCarSpawner(Node):
    def __init__(self):
        super().__init__('smart_car_spawner')
        self.declare_parameter('config_file', '')
        self.declare_parameter('template_path', '')

        use_sim_time = self.get_parameter('use_sim_time').value

        tpl_param = self.get_parameter('template_path').value
        self.template_path = os.path.realpath(
            tpl_param if tpl_param else get_template_path())
        self.get_logger().info(f'模板路径: {self.template_path}')

        cfg_param = self.get_parameter('config_file').value
        if not cfg_param:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('smart_car_sim')
                cfg_param = os.path.join(pkg, 'config', 'smart_car_config.yaml')
            except Exception:
                cfg_param = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'smart_car_config.yaml')

        self.cars = load_car_config(cfg_param)
        self.get_logger().info(f'加载 {len(self.cars)} 辆智能小车 <- {cfg_param}')

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        self.tf_pub = self.create_publisher(
            TransformStamped, '/tf_static',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        self.spawn_all()
        self.create_service(SetBool, '~/enable_fault', self.on_enable_fault)

    def spawn_all(self):
        for car in self.cars:
            name = car['name']
            params = car['params']
            start = params.get('start', [0.0, 0.0, 0.0])
            model_type = params.get('model_type', 'template')

            if model_type == 'prebuilt':
                sdf_xml = self._load_prebuilt_sdf(name, params)
            else:
                try:
                    sdf_xml = generate_sdf(name, params, self.template_path)
                except Exception as e:
                    self.get_logger().error(f'{name}: SDF 生成失败 - {e}')
                    continue

            if sdf_xml is None:
                continue

            req = SpawnEntity.Request()
            req.name = name
            req.xml = sdf_xml
            req.robot_namespace = name
            req.initial_pose = Pose()
            req.initial_pose.position = Point(
                x=float(start[0]), y=float(start[1]), z=0.15)
            yaw = float(start[2]) if len(start) > 2 else 0.0
            req.initial_pose.orientation = yaw_to_quat(yaw)
            req.reference_frame = 'world'

            self._call_spawn(name, req)

            if model_type == 'prebuilt':
                self._publish_prebuilt_tfs(name, params)
            else:
                self._publish_sensor_tfs(name, params)

    def _load_prebuilt_sdf(self, name, params):
        """加载预置模型 SDF 并做命名空间隔离。"""
        model_name = params.get('model_name', '')
        if not model_name:
            self.get_logger().error(f'{name}: 预置模型缺少 model_name 字段')
            return None
        sdf_path = _find_model_sdf(model_name)
        if not sdf_path:
            self.get_logger().error(f'{name}: 找不到模型 {model_name} 的 SDF')
            return None
        self.get_logger().info(f'{name}: 使用预置模型 {model_name} <- {sdf_path}')
        return _make_namespaced_sdf(sdf_path, name, name)

    def _call_spawn(self, name, req):
        if not self.spawn_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(f'{name}: /spawn_entity 服务不可用')
            return
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        try:
            res = fut.result()
            if res and res.success:
                p = req.initial_pose.position
                self.get_logger().info(
                    f'{name}: 生成成功 @ ({p.x:.1f}, {p.y:.1f})')
            else:
                msg = getattr(res, 'status_message', '') if res else 'null'
                self.get_logger().error(f'{name}: 生成失败 {msg}')
        except Exception as e:
            self.get_logger().error(f'{name}: 生成异常 {e}')

    def _publish_prebuilt_tfs(self, name, params):
        """为预置模型（TurtleBot3 等）发布标准 TF 树。"""
        model_name = params.get('model_name', '')
        base_frame = f'{name}/base_link'
        odom_frame = f'{name}/odom'

        self._send_tf(odom_frame, f'{name}/base_footprint', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._send_tf(f'{name}/base_footprint', base_frame, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0)

        if 'waffle_pi' in model_name:
            self._send_tf(base_frame, f'{name}/base_scan',
                          -0.064, 0.0, 0.121, 0.0, 0.0, 0.0)
            self._send_tf(base_frame, f'{name}/imu_link',
                          -0.032, 0.0, 0.068, 0.0, 0.0, 0.0)
            self._send_tf(base_frame, f'{name}/camera_link',
                          0.073, -0.011, 0.084, 0.0, 0.0, 0.0)
            self._send_tf(f'{name}/camera_link', f'{name}/camera_rgb_optical_frame',
                          0.003, 0.011, 0.009, -math.pi / 2, 0.0, -math.pi / 2)
        elif 'burger' in model_name:
            self._send_tf(base_frame, f'{name}/base_scan',
                          -0.032, 0.0, 0.171, 0.0, 0.0, 0.0)
            self._send_tf(base_frame, f'{name}/imu_link',
                          -0.032, 0.0, 0.068, 0.0, 0.0, 0.0)

        self.get_logger().info(f'{name}: 预置模型 TF 发布完成 ({model_name})')

    def _publish_sensor_tfs(self, name, params):
        """为模板生成的车辆发布 sensor frame 的静态 TF。"""
        length = params.get('length', 1.6)
        height = params.get('height', 0.4)
        sensor_cfg = params.get('sensors', {})

        base_frame = f'{name}/base_link'
        odom_frame = f'{name}_odom'

        self._send_tf(odom_frame, base_frame, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        lidar_x = sensor_cfg.get('lidar', {}).get('x', length * 0.1)
        lidar_z = sensor_cfg.get('lidar', {}).get('z', height + 0.15)
        self._send_tf(base_frame, f'{name}/lidar_link',
                      lidar_x, 0.0, lidar_z, 0.0, 0.0, 0.0)

        imu_x = sensor_cfg.get('imu', {}).get('x', 0.0)
        imu_z = sensor_cfg.get('imu', {}).get('z', height * 0.5)
        self._send_tf(base_frame, f'{name}/imu_link',
                      imu_x, 0.0, imu_z, 0.0, 0.0, 0.0)

        cam_x = sensor_cfg.get('camera', {}).get('x', length * 0.4)
        cam_z = sensor_cfg.get('camera', {}).get('z', height + 0.08)
        self._send_tf(base_frame, f'{name}/camera_link',
                      cam_x, 0.0, cam_z, 0.0, 0.3, 0.0)
        self._send_tf(f'{name}/camera_link', f'{name}/camera_optical_frame',
                      0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2)

        self.get_logger().info(f'{name}: TF 帧发布完成')

    def _send_tf(self, parent, child, x, y, z, roll, pitch, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = self._rpy_to_quat(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_pub.publish(t)

    @staticmethod
    def _rpy_to_quat(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def on_enable_fault(self, req, resp):
        resp.success = True
        resp.message = f'故障模拟: {"启用" if req.data else "禁用"}'
        self.get_logger().info(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SmartCarSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
