#!/bin/bash
# 同步 room_layout.world 到 WSL 工作区，并尝试对运行中的 gzserver 实时摆正三个房间
SRC=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/room_layout.world
DST=/home/hzm/v2x_ws/src/smart_car_sim/worlds/room_layout.world

echo "=== [1/2] 同步 world ==="
cp "$SRC" "$DST" && echo "OK: $DST 已更新"

echo "=== [2/2] 实时设置位姿（无需重启） ==="
gz model -m room_33 -p "0 0 0 1.5708 0 0"   && echo "room_33 已摆正"  || echo "room_33 设置失败"
gz model -m room_20 -p "15 0 0 1.5708 0 0" && echo "room_20 已摆正"  || echo "room_20 设置失败"
gz model -m room_12 -p "30 0 0 1.5708 0 0" && echo "room_12 已摆正"  || echo "room_12 设置失败"

echo "=== 当前位姿确认 ==="
gz model -i -m room_20 | head -12
echo "=== 完成 ==="
