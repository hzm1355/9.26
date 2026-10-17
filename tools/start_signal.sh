#!/bin/bash
# 拉起信号灯配时仿真子系统（协议 §三.(二)）：管理器 + 自适应 + 记录器
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/stop_signal.sh >/dev/null 2>&1 || true

# 先把五车摆到各自路线起点，再拉起节点。
# 顺序反过来的话，驱动节点会先按 SDF 出生点把车开走几十秒，等 place_cars 再把它们
# 瞬移回起点时 idx 已经指向别的航段，车会斜穿街区、冲过角点出界。
timeout -k 2 60 python3 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/place_cars.py || true

setsid nohup ros2 launch v2x_demo traffic_signal_full.launch.py \
  enable_adaptive:=true enable_recorder:=true \
  record_file:=/tmp/tl_log.json > /tmp/signal.log 2>&1 &
sleep 10
# 记录器默认待机，拉起后即开始记录（协议 §三.(二).5 全流程录制）
ros2 service call /traffic_light_recorder/start std_srvs/srv/Trigger > /dev/null 2>&1 || true
echo "=== procs ==="
pgrep -af 'traffic_light_manager|adaptive_timing_node|traffic_light_recorder' | sed 's/ .*ros/ ros/' | head -5
echo "=== signal.log ==="
grep -E 'TrafficLightManager|adaptive|recorder|ERROR|error' /tmp/signal.log | head -12
echo "=== tl topics ==="
ros2 topic list | grep traffic_light
