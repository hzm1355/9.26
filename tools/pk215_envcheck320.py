#!/usr/bin/env python3
# 出界新判据的离线自测：不连 ROS，直接把 CarAgent 拿来实例化，验证
#   甲) 每台车的 env_x/env_y 由它自己的 route 算出，且严格大于该路线最远的中心线；
#   乙) 历史上"正常压线"的读数不再被判出界（西环 x=-21.8 这一条是原来 22.0 必炸的那颗）；
#   丙) 历史上"真被弹出台面"的读数仍然判出界（x=27.78 / 32.2 —— 台面只有 ±25，
#       这一条要是放掉了，车掉到 z=0 的地面上还会被当成"在跑"）。
# 用法：python3 tools/pk215_envcheck320.py [生效 yaml]
import os
import sys

import yaml

# 按**文件路径**加载，不猜包名：install 侧到底是 site-packages 还是 lib/v2x_demo 的
# 符号链接，各代 colcon 形态不一样，sys.path 猜错就变成"模块不存在"的假失败。
CAND = [
    os.path.expanduser('~/v2x_ws/src/v2x_demo/v2x_demo/signal_drive_node.py'),
    os.path.expanduser('~/v2x_ws/install/v2x_demo/lib/v2x_demo/signal_drive_node.py'),
    '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo/signal_drive_node.py',
]
SRC = next((p for p in CAND if os.path.exists(p)), None)
if SRC is None:
    print('IMPORT FAIL：三个候选路径都不存在：%s' % CAND)
    sys.exit(2)
try:
    import importlib.util
    spec = importlib.util.spec_from_file_location('sd_probe', SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    CarAgent = mod.CarAgent
except Exception as exc:                                # 导入即失败也要点名，不静默
    print('IMPORT FAIL（%s）：%s' % (SRC, exc))
    print('  这一步要在 source 过 /opt/ros/humble 与 ~/v2x_ws/install 的壳里跑，'
          '模块级 import rclpy 才有')
    sys.exit(2)
print('载入：%s' % SRC)


class _Now:
    nanoseconds = 0


class _Clock:
    def now(self):
        return _Now()


class _Log:
    def warn(self, *a, **k):
        pass


class _Pub:
    def publish(self, *a, **k):
        pass


class _Node:
    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Log()

    # CarAgent.__init__ 会自建 cmd_vel 发布者（离线自测里没有 DDS，给个哑对象）
    def create_publisher(self, *a, **k):
        return _Pub()

    def create_subscription(self, *a, **k):
        return None


Y = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    '~/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml')
cfg = yaml.safe_load(open(Y, encoding='utf-8')) or {}
root = cfg.get('signal_drive', cfg)
cars = root.get('cars') or {}

# 历史读数（都是这轮之前从 gz model -p / pk208 里抄下来的真实点）
CALM = [(-21.80, -8.0), (-21.80, 1.4), (20.80, -18.95), (-0.80, -18.86),
        (6.41, -18.95), (-18.84, -16.80)]
FLUNG = [(27.78, -19.49), (32.20, 1.5), (24.60, -23.0), (-0.40, 24.9)]

bad = []
print('%-7s %-42s %8s %8s' % ('车', 'route 端点', 'env_x', 'env_y'))
for name, ccfg in cars.items():
    route = ccfg.get('route') or []
    if len(route) < 2:
        print('%-7s 无 route，跳过' % name)
        continue
    ag = CarAgent(name, ccfg, _Node())
    mx = max(abs(p[0]) for p in route)
    my = max(abs(p[1]) for p in route)
    print('%-7s %-42s %8.2f %8.2f' % (
        name, str([(round(p[0], 1), round(p[1], 1)) for p in route])[:42],
        ag.env_x, ag.env_y))
    if not (ag.env_x > mx and ag.env_y > my):
        bad.append('%s：包络没有覆盖自己的中心线端点 %s' % (name, (ag.env_x, mx)))
    if ag.env_x > 25.0 or ag.env_y > 25.0:
        bad.append('%s：包络越出台面 25，掉出台面那一档会失效' % name)

print('')
print('=== 乙) 正常压线读数必须一律"不出界"（用跑得最宽的路线包络判）===')
WIDE = max(cars.values(), key=lambda c: max(abs(p[0]) for p in (c.get('route') or [[0, 0]])))
ax = CarAgent('probe', WIDE, _Node())
for x, y in CALM:
    trip = abs(x) > max(22.0, ax.env_x) or abs(y) > max(19.0, ax.env_y)
    print('  (%7.2f,%7.2f) 判出界=%s' % (x, y, trip))
    if trip:
        bad.append('正常压线 (%.2f,%.2f) 仍被判出界' % (x, y))

print('')
print('=== 丙) 真被弹出台面的读数必须仍"出界" ===')
for x, y in FLUNG:
    trip = abs(x) > max(22.0, ax.env_x) or abs(y) > max(19.0, ax.env_y)
    print('  (%7.2f,%7.2f) 判出界=%s' % (x, y, trip))
    if not trip:
        bad.append('弹出台面 (%.2f,%.2f) 没被判出界 —— 护栏被放得太宽' % (x, y))

print('')
if bad:
    for b in bad:
        print('FAIL %s' % b)
    print('ENVCHECK320 FAIL（%d 条）' % len(bad))
    sys.exit(1)
print('ENVCHECK320 PASS')
