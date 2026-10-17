#!/bin/bash
# car_6 的矩形占地 vs world 里所有碰撞件：先校准，再量现场两个位姿。
#
# 为什么必须带校准：这条判据的结论是"有没有东西贴着车"，一旦筛盒的逻辑错了，
# 它会给出"最小间隙 0.83 m，没东西贴着"这种看起来完全自洽的假清白 —— 152 轮就是
# 拿"中心距"当判据，把车头棱边离亭子角 2 cm 读成了"一米五空地"。校准用"车心正好
# 压在亭子中心"这个必然重叠的位置，它必须报重叠，否则后面的读数一句都不信。
set +e
W=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world
tr -d '\r' < /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/car_vs_colliders.py > /tmp/cvc.py

echo "=== 校准：车心压在亭子中心 (9.40,-11.30) yaw 0，必须报重叠 ==="
python3 /tmp/cvc.py "$W" 9.40 -11.30 0.0 2>&1 | tr -d '\r'

echo "=== 现场 A：实验起始位姿 (10.207,-12.588) yaw 2.191 ==="
python3 /tmp/cvc.py "$W" 10.207 -12.588 2.191 2>&1 | tr -d '\r'

echo "=== 现场 B：实验收尾位姿 (10.173,-12.576) yaw 2.142 ==="
python3 /tmp/cvc.py "$W" 10.173 -12.576 2.142 2>&1 | tr -d '\r'

echo "=== 现场 C：152 轮最初卡住的那位 (10.228,-12.566) yaw 2.260 ==="
python3 /tmp/cvc.py "$W" 10.228 -12.566 2.260 2>&1 | tr -d '\r'
echo DONE
