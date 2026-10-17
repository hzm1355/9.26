#!/usr/bin/env bash
# 抽查生成的 v2x_city.world 与 scene_devices.yaml 关键部件
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo || exit 1
echo '== road_NS-main 路面 =='
grep -A1 '<model name="road_NS-main"' worlds/v2x_city.world | head -2
echo '== 斑马线第一条 =='
grep -B1 -A6 'zebra_J1_NS-main_0"' worlds/v2x_city.world | head -8
echo '== ETC-01 东柱 =='
grep -B1 -A7 'ETC-01_post_e"' worlds/v2x_city.world | head -9
echo '== 停车场道闸杆 =='
grep -B1 -A4 'PKG-NW_barrier"' worlds/v2x_city.world | head -6
echo '== 学校 include =='
grep -B2 -A2 'SCHOOL-01' worlds/v2x_city.world | head -6
echo '== 高架路面 =='
grep -B1 -A7 'VIADUCT-01_deck"' worlds/v2x_city.world | head -9
echo '====DEVICES===='
head -70 config/scene_devices.yaml
