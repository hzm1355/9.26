#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能小车模块自测 —— 验证 SDF 生成、参数配置、模板加载。

用法：
    python3 car_selftest.py
    ros2 run smart_car_sim car_selftest.py
"""
import os
import sys
import math

# 确保能导入同目录的 car_generator 模块
_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from car_generator import (
    generate_sdf, get_template_path, load_car_config,
    _compute_inertia, _default_car_params,
)


def test_default_params():
    p = _default_car_params()
    assert p['mass'] > 0, '质量必须为正'
    assert p['length'] > 0, '长度必须为正'
    assert p['wheelbase'] > 0, '轴距必须为正'
    assert p['max_speed'] > 0, '最大速度必须为正'
    print(f'  默认参数: mass={p["mass"]}kg, L={p["length"]}m, '
          f'vmax={p["max_speed"]}m/s, amax={p["max_acceleration"]}m/s²')
    print('  [PASS] 默认参数校验')


def test_inertia():
    ixx, iyy, izz = _compute_inertia(100.0, 2.0, 1.0, 0.5)
    assert ixx > 0 and iyy > 0 and izz > 0, '惯性矩必须为正'
    assert iyy > ixx, '长度>宽度时 Iyy>Ixx'
    print(f'  惯性矩(100kg, 2x1x0.5): Ixx={ixx:.2f} Iyy={iyy:.2f} Izz={izz:.2f}')
    print('  [PASS] 惯性矩计算')


def test_template_load():
    tpl = get_template_path()
    assert os.path.isfile(tpl), f'模板文件不存在: {tpl}'
    with open(tpl, encoding='utf-8') as f:
        content = f.read()
    assert '{CAR_NAME}' in content, '模板缺少 {CAR_NAME} 占位符'
    assert '{MASS}' in content, '模板缺少 {MASS} 占位符'
    assert 'diff_drive' in content, '模板缺少差速驱动插件'
    assert 'gazebo_ros_ray_sensor' in content, '模板缺少激光雷达插件'
    assert 'gazebo_ros_imu_sensor' in content, '模板缺少 IMU 插件'
    assert 'gazebo_ros_camera' in content, '模板缺少相机插件'
    print(f'  模板路径: {tpl}')
    print(f'  模板大小: {len(content)} 字节')
    print('  [PASS] 模板加载与完整性')


def test_sdf_generation():
    tpl = get_template_path()
    params = _default_car_params()
    sdf = generate_sdf('test_car', params, tpl)

    assert '<?xml' in sdf, '缺少 XML 头'
    assert '<model name="test_car">' in sdf, '模型名未替换'
    assert '80.00' in sdf, '质量未替换'
    assert '/test_car' in sdf, '命名空间未替换'
    assert '{CAR_NAME}' not in sdf, '残留未替换的占位符'
    assert '{MASS}' not in sdf, '残留未替换的占位符'

    assert '<sensor name="lidar"' in sdf, '缺少 LiDAR 传感器'
    assert '<sensor name="imu_sensor"' in sdf, '缺少 IMU 传感器'
    assert '<sensor name="camera"' in sdf, '缺少相机传感器'
    assert 'libgazebo_ros_diff_drive.so' in sdf, '缺少差速驱动插件'

    print(f'  SDF 大小: {len(sdf)} 字节')
    print('  [PASS] SDF 生成（默认参数）')


def test_sdf_custom_params():
    tpl = get_template_path()
    params = {
        'mass': 150.0,
        'length': 2.5,
        'width': 1.2,
        'height': 0.7,
        'wheelbase': 1.8,
        'wheel_radius': 0.15,
        'wheel_separation': 1.0,
        'max_speed': 5.0,
        'max_acceleration': 6.0,
        'color': [0.9, 0.1, 0.1],
    }
    sdf = generate_sdf('custom_car', params, tpl)
    assert '<model name="custom_car">' in sdf
    assert '150.00' in sdf
    assert '2.500' in sdf
    assert '0.900 0.100 0.100' in sdf
    print('  [PASS] SDF 生成（自定义参数）')


def test_config_load():
    cfg_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'smart_car_config.yaml')
    if not os.path.isfile(cfg_path):
        print('  [SKIP] 配置文件不存在')
        return

    cars = load_car_config(cfg_path)
    assert len(cars) > 0, '配置中无车辆'

    template_count = 0
    prebuilt_count = 0
    for car in cars:
        assert 'name' in car, '缺少 name 字段'
        assert 'params' in car, '缺少 params 字段'
        p = car['params']
        assert p['mass'] > 0
        assert p['max_speed'] > 0
        model_type = p.get('model_type', 'template')
        if model_type == 'prebuilt':
            prebuilt_count += 1
            assert p.get('model_name'), f'{car["name"]} 预置模型缺少 model_name'
            print(f'  车辆: {car["name"]} | 预置模型: {p["model_name"]}')
        else:
            template_count += 1
            print(f'  车辆: {car["name"]} | mass={p["mass"]}kg '
                  f'vmax={p["max_speed"]}m/s color={p.get("color", "?")}')

    print(f'  [PASS] 配置加载 ({len(cars)} 辆车: {template_count} 模板 + {prebuilt_count} 预置)')


def test_full_pipeline():
    """完整流水线：加载配置 -> 生成每辆车 SDF -> 验证（跳过预置模型）。"""
    cfg_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'smart_car_config.yaml')
    tpl = get_template_path()
    if not os.path.isfile(cfg_path):
        print('  [SKIP] 配置文件不存在')
        return

    cars = load_car_config(cfg_path)
    generated = 0
    for car in cars:
        if car['params'].get('model_type', 'template') == 'prebuilt':
            print(f'  {car["name"]}: 预置模型，跳过模板生成 ✓')
            continue
        sdf = generate_sdf(car['name'], car['params'], tpl)
        assert f'<model name="{car["name"]}">' in sdf
        generated += 1
        print(f'  {car["name"]}: SDF {len(sdf)} 字节 ✓')

    print(f'  [PASS] 完整流水线 ({generated} 辆模板生成, '
          f'{len(cars) - generated} 辆预置模型)')


def main():
    print('=' * 60)
    print('智能小车模块自测')
    print('=' * 60)

    tests = [
        ('默认参数', test_default_params),
        ('惯性矩计算', test_inertia),
        ('模板加载', test_template_load),
        ('SDF 生成', test_sdf_generation),
        ('自定义参数 SDF', test_sdf_custom_params),
        ('配置加载', test_config_load),
        ('完整流水线', test_full_pipeline),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        print(f'\n--- {name} ---')
        try:
            fn()
            passed += 1
        except AssertionError as e:
            print(f'  [FAIL] {e}')
            failed += 1
        except Exception as e:
            print(f'  [ERROR] {e}')
            failed += 1

    print(f'\n{"=" * 60}')
    print(f'结果: {passed} 通过, {failed} 失败, 共 {passed + failed} 项')
    print('=' * 60)
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
