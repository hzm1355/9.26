#!/usr/bin/env bash
# ============================================================================
# 可靠收发的驱动冒烟：驱动 (v,ω) 持续 T 秒 -> 采位姿对比位移/姿态。
# 关键修复：零速收尾用 -r 20 持续发 2s，不用 --once。
#   原因：ros2 topic pub --once 有 QoS 握手竞态（消息可能在订阅方建立前
#   发出即丢），ackermann 插件的 last_cmd 保持旧值 -> 车不停 ->
#   低摩擦下打滑失控乱走（实测：车被持续驱动 15m 后物理崩坏）。
# 用法: bash drive_test.sh [v] [w] [t]   默认 0.2 0.0 8
# ============================================================================
V=${1:-0.2}; W=${2:-0.0}; T=${3:-8}
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

pose() {  # 采一次 position+orientation（8 字段一行）
  timeout 3 ros2 topic echo /odom --once --field pose.pose 2>/dev/null \
    | grep -E 'x:|y:|z:|w:' | tr -d ' ' | tr '\n' ' '
}

echo "[drive] 起点位姿: $(pose)"
echo "[drive] 驱动 v=$V w=$W t=${T}s ..."
timeout "$T" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $V}, angular: {z: $W}}" > /dev/null 2>&1

# 可靠停车：持续发零速 2s（保证 QoS 握手后消息送达，覆盖 last_cmd）
timeout 2 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" > /dev/null 2>&1

sleep 2
P2=$(pose)
echo "[drive] 终点位姿: $P2"

D=$(timeout 3 ros2 topic echo /distance --once --field data 2>/dev/null | tr -d ' ')
echo "[drive] 插件里程 distance: $D"
echo "[drive] done"
