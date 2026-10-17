#!/bin/bash
# 把卡口测试环境恢复成默认取证状态：牌面自发光(lighting off) + 光照回到 day。
# lit_control.sh 跑完受光对照后必须走这一步，否则后续所有识别率数字都带着
# "牌面被场景压暗"这个额外变量，和报告里的主表不是同一套条件。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

# 换回自发光并重新下发五副牌。走 restart_plate_gen.sh 而不是自己 kill+start：
# 它会把**所有**残留实例杀干净（只杀 head -1 的那版实现攒出过 6 个 on/off 混跑的
# plate_gen，材质归谁最后是随机的），并在下发后校验最新 5 张材质的 lighting 标志。
bash $SIM/tools/restart_plate_gen.sh off

# 光照回到白天，并等回读确认（改环境光到相机生效之间还有渲染队列延迟）
bash $SIM/tools/set_lighting.sh day
