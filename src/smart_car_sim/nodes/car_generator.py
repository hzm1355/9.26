#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车 SDF 生成器 —— 从 YAML 配置动态生成 Gazebo SDF 模型。

物理参数全部可在 smart_car_config.yaml 中配置，生成器将模板中的占位符替换为
实际数值。支持多种预设车型（小型车/中型车/巴士/配送机器人）。
"""
import math
import os
import yaml


def _default_car_params():
    """默认小车物理参数（中型轿车）。"""
    return {
        'mass': 80.0,
        'length': 1.6,
        'width': 0.8,
        'height': 0.4,
        'wheelbase': 1.0,
        'wheel_radius': 0.1,
        'wheel_width': 0.08,
        'wheel_separation': 0.7,
        'max_speed': 2.0,
        'max_acceleration': 3.0,
        'priority': 0,
        'color': [0.3, 0.5, 0.8],
    }


def _compute_inertia(mass, length, width, height):
    """均匀长方体惯性矩。"""
    ixx = mass * (width**2 + height**2) / 12.0
    iyy = mass * (length**2 + height**2) / 12.0
    izz = mass * (length**2 + width**2) / 12.0
    return ixx, iyy, izz


def _compute_wheel_inertia(mass, radius, width):
    """实心圆柱轮子惯性矩。"""
    ixx = mass * (3 * radius**2 + width**2) / 12.0
    iyy = mass * radius**2 / 2.0
    return ixx, iyy


def generate_sdf(car_name: str, params: dict, template_path: str) -> str:
    """
    根据参数生成完整 SDF 字符串。

    Args:
        car_name: 模型唯一名称（也是 ROS2 命名空间）
        params: 物理参数字典（可包含 sensor/actuator 子配置）
        template_path: SDF 模板文件路径
    """
    p = _default_car_params()
    p.update(params)

    mass = p['mass']
    length = p['length']
    width = p['width']
    height = p['height']
    wheel_radius = p['wheel_radius']
    wheel_width = p['wheel_width']
    wheel_sep = p['wheel_separation']
    wheelbase = p['wheelbase']
    max_speed = p['max_speed']
    max_accel = p['max_acceleration']
    color = p['color']
    color_str = f'{color[0]:.3f} {color[1]:.3f} {color[2]:.3f}'

    ixx, iyy, izz = _compute_inertia(mass, length, width, height)
    wheel_mass = 2.0
    w_ixx, w_iyy = _compute_wheel_inertia(wheel_mass, wheel_radius, wheel_width)

    max_torque = max_accel * wheel_mass * wheel_radius
    max_wheel_accel = max_accel / wheel_radius

    sensor_cfg = p.get('sensors', {})
    lidar_cfg = sensor_cfg.get('lidar', {})
    imu_cfg = sensor_cfg.get('imu', {})
    cam_cfg = sensor_cfg.get('camera', {})

    chassis_z = height / 2.0 + 0.05
    cabin_l = length * 0.45
    cabin_w = width * 0.85
    cabin_h = height * 0.6
    cabin_offset_y = -length * 0.05
    cabin_z = height + cabin_h / 2.0 + 0.05

    wheel_y_front = wheelbase * 0.4
    caster_y = -wheelbase * 0.4
    caster_radius = wheel_radius * 0.6

    vals = {
        'CAR_NAME': car_name,
        'MASS': f'{mass:.2f}',
        'LENGTH': f'{length:.3f}',
        'WIDTH': f'{width:.3f}',
        'HEIGHT': f'{height:.3f}',
        'CHASSIS_Z': f'{chassis_z:.4f}',
        'COLOR': color_str,
        'CABIN_L': f'{cabin_l:.3f}',
        'CABIN_W': f'{cabin_w:.3f}',
        'CABIN_H': f'{cabin_h:.3f}',
        'CABIN_OFFSET_Y': f'{cabin_offset_y:.4f}',
        'CABIN_Z': f'{cabin_z:.4f}',
        'IXX': f'{ixx:.4f}',
        'IYY': f'{iyy:.4f}',
        'IZZ': f'{izz:.4f}',
        'WHEEL_RADIUS': f'{wheel_radius:.4f}',
        'WHEEL_WIDTH': f'{wheel_width:.4f}',
        'WHEEL_SEPARATION': f'{wheel_sep:.4f}',
        'WHEEL_DIAMETER': f'{wheel_radius * 2:.4f}',
        'WHEEL_MASS': f'{wheel_mass:.2f}',
        'WHEEL_IXX': f'{w_ixx:.6f}',
        'WHEEL_IYY': f'{w_iyy:.6f}',
        'WHEEL_LATERAL': f'{wheel_sep / 2:.4f}',
        'WHEEL_Y_FRONT': f'{wheel_y_front:.4f}',
        'CASTER_Y': f'{caster_y:.4f}',
        'CASTER_RADIUS': f'{caster_radius:.4f}',
        'MAX_TORQUE': f'{max_torque:.2f}',
        'MAX_WHEEL_ACCEL': f'{max_wheel_accel:.2f}',
        'DIFF_RATE': '100',
        'LIDAR_X': f'{length * 0.1:.4f}',
        'LIDAR_Y': '0',
        'LIDAR_Z': f'{height + 0.15:.4f}',
        'LIDAR_RATE': str(lidar_cfg.get('update_rate', 10)),
        'LIDAR_SAMPLES': str(lidar_cfg.get('samples', 360)),
        'LIDAR_MIN_RANGE': str(lidar_cfg.get('min_range', 0.12)),
        'LIDAR_MAX_RANGE': str(lidar_cfg.get('max_range', 30.0)),
        'LIDAR_NOISE': str(lidar_cfg.get('noise_stddev', 0.01)),
        'IMU_X': '0',
        'IMU_Y': '0',
        'IMU_Z': f'{height * 0.5:.4f}',
        'IMU_RATE': str(imu_cfg.get('update_rate', 200)),
        'CAM_X': f'{length * 0.4:.4f}',
        'CAM_Y': '0',
        'CAM_Z': f'{height + 0.08:.4f}',
        'CAM_RATE': str(cam_cfg.get('update_rate', 30)),
        'CAM_FOV': str(cam_cfg.get('hfov', 1.2)),
        'CAM_WIDTH': str(cam_cfg.get('width', 640)),
        'CAM_HEIGHT': str(cam_cfg.get('height', 480)),
        'CAM_FAR': str(cam_cfg.get('far_clip', 100)),
    }

    with open(template_path, encoding='utf-8') as f:
        template = f.read()

    return template.format(**vals)


def load_car_config(config_path: str) -> list:
    """
    加载小车配置文件，返回车辆列表。

    每辆车包含 name, type (引用预设), 以及可覆盖的 params。
    """
    with open(config_path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    presets = cfg.get('presets', {})
    cars = []
    for car_entry in cfg.get('cars', []):
        name = car_entry['name']
        preset_name = car_entry.get('type', 'medium_car')
        preset = presets.get(preset_name, _default_car_params())

        merged = dict(preset)
        for k, v in car_entry.items():
            if k not in ('name', 'type'):
                merged[k] = v

        cars.append({'name': name, 'params': merged})

    return cars


def get_template_path():
    """获取模板文件路径（兼容安装后和源码目录）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory('smart_car_sim')
        tpl = os.path.join(pkg, 'models', 'smart_car', 'model.sdf.template')
        if os.path.isfile(tpl):
            return tpl
    except Exception:
        pass
    return os.path.join(os.path.dirname(__file__), '..', 'models', 'smart_car', 'model.sdf.template')
