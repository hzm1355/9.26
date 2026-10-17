#!/usr/bin/env bash
# 重新生成 + 部署 + 全停（一次性脚本）
set -e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim

python3 scene_builder.py v2x_scene.yaml src/v2x_demo/worlds/v2x_city.world src/v2x_demo/config/scene_devices.yaml | tail -4

echo '--- junction 交叉块验证 ---'
grep -c 'junction_J' src/v2x_demo/worlds/v2x_city.world
grep -B1 -A6 'name="junction_J1"' src/v2x_demo/worlds/v2x_city.world | grep -E 'name=|size' | head -2

echo '--- road_NS-main 尺寸（应 17.2 长） ---'
grep -A8 'name="road_NS-main"' src/v2x_demo/worlds/v2x_city.world | grep -m1 'size'

bash deploy_scene.sh | head -2
bash stop_all.sh | tail -1
pkill -9 -f '[g]zserver' 2>/dev/null || true
sleep 1
echo '[rebuild+deploy+stop] 完成'
