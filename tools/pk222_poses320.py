#!/usr/bin/env python3
# 用 ROS 侧 /model_states 读六台位姿，替代 `gz model -m X -p` 那种带硬 timeout 的 gz CLI 轮询。
# 为什么换：pk220 跑到 F8 时 gzserver 被 SIGABRT 打死，末行是
#   boost::shared_ptr<gazebo::transport::Connection>::operator-> Assertion `px != 0' failed
# 而 F8 正是连着六发 `gz model` CLI 全部空回的地方 —— 每次 CLI 都要新建/撕掉一条
# gz-transport 连接，`timeout` 又会在握手中途 SIGKILL 它。这条嫌疑不能留着继续喂，
# 所以读数改走 ROS 桥（/model_states 是服务端已有的发布路径，不再新开 transport 连接）。
# 用法：python3 tools/pk222_poses320.py [秒数=3] [car_1 car_2 ...]
import math
import sys
import time

import rclpy
from gazebo_msgs.msg import ModelStates

SECS = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('car') else 3.0
NAMES = [a for a in sys.argv[1:] if a.startswith('car_')] or [
    'car_1', 'car_2', 'car_3', 'car_4', 'car_5', 'car_6']

best = {}


def cb(ms):
    for i, n in enumerate(ms.name):
        if n in NAMES:
            p = ms.pose[i].position
            q = ms.pose[i].orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            v = ms.twist[i].linear
            w = ms.twist[i].angular
            best[n] = (p.x, p.y, p.z, yaw, math.hypot(v.x, v.y, v.z),
                       abs(w.z), ms.twist[i].linear.x)


rclpy.init()
n = rclpy.create_node('pk222_poses320')
n.create_subscription(ModelStates, '/model_states', cb, 10)
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(n, timeout_sec=0.1)
n.destroy_node()
rclpy.shutdown()

print('  车       (x, y)            z      yaw      实测|v|   实测|ωz|')
miss = []
for k in NAMES:
    if k not in best:
        miss.append(k)
        continue
    x, y, z, yaw, vm, wm, _ = best[k]
    print('  %-7s (%7.2f,%7.2f)  %6.3f  %+6.3f   %5.2f    %5.2f'
          % (k, x, y, z, yaw, vm, wm))
if miss:
    print('  !! 读不到：%s（%s）' % (','.join(miss),
          'gzserver 不在或 /model_states 零发布者' if len(miss) == len(NAMES) else '这台不在世界里'))
    sys.exit(4)
print('POSES320 OK')
