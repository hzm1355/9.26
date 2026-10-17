#!/usr/bin/env bash
# 部署生成的场景：D盘src → ~/v2x_ws/src → install 三处同步，并切换 start_all.sh 用新 world
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
SRC=$SIM/src/v2x_demo
WS_SRC=~/v2x_ws/src/v2x_demo
SHARE=~/v2x_ws/install/v2x_demo/share/v2x_demo

for f in worlds/v2x_city.world config/scene_devices.yaml config/regions.yaml; do
  cp -f "$SRC/$f" "$WS_SRC/$f"
  cp -f "$SRC/$f" "$SHARE/$f"
done
echo "[deploy] v2x_city.world + scene_devices.yaml 已同步到 ws/src 与 install"

sed -i 's|WORLD=$SHARE/worlds/v2x_intersection.world|WORLD=$SHARE/worlds/v2x_city.world|' "$SIM/start_all.sh"
echo "[deploy] start_all.sh 当前 WORLD 配置："
grep -n 'WORLD=' "$SIM/start_all.sh" | head -2
