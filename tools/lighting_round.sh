#!/bin/bash
# 光照识别率的两轮取证，一次跑完：
#   第 1 轮 牌面自发光（线上默认，lighting off）
#   第 2 轮 牌面受光（lit:=on，牌面跟着场景一起暗）
# 两轮都用 gate_lighting_test.py --both：每档强制南北两环各一张。
#
# 为什么必须 --both：第一轮只等"该档第一张"时，样本随机落在南环或北环，
# 而两环的取景角与补光余量本来就不同——受光对照首轮就是这么把"北环弱光丢牌"
# 和"南环弱光正常"混进同一档的，档与档之间不可比。
#
# 为什么第 2 轮要跑五档：夜间档拆成"有补光/无补光"两条，才能把"这档黑"和
# "补光灯没起作用"分开（custom:0.03,0.02,0.05,0,0 / ...,1）。
#
# 用法：bash tools/lighting_round.sh [第几轮]   不带参数跑两轮
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
EV=$SIM/evidence
mkdir -p "$EV"
GO=${1:-both}
unset ROS_DOMAIN_ID
# gate_lighting_test.py 是个 rclpy 节点，不 source 就 import 不到 rclpy
# （lit_control.sh / restore_default.sh 各自内部 source 过，只有第 1 轮这条是裸调）
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

if [ "$GO" = "both" ] || [ "$GO" = "1" ]; then
  echo '########## 第 1 轮：牌面自发光（lighting off，线上默认）##########'
  bash $SIM/tools/ensure_recognizer.sh
  bash $SIM/tools/reset_fleet.sh 2>&1 | tail -2
  python3 -u $SIM/tools/gate_lighting_test.py --both day backlit lowlight night 150 \
    2>&1 | tr -d '\r' | tee "$EV/lighting_lit_off.txt"
  bash $SIM/tools/restore_default.sh 2>&1 | tr -d '\r' | tail -3
fi

if [ "$GO" = "both" ] || [ "$GO" = "2" ]; then
  echo '########## 第 2 轮：牌面受光（lit:=on）+ 夜间补光开关对照 ##########'
  # 每轮开头都过一次闸：识别器会被单张抓拍带崩，崩了之后这一轮只会静默收 0 行
  bash $SIM/tools/ensure_recognizer.sh
  bash $SIM/tools/reset_fleet.sh 2>&1 | tail -2
  bash $SIM/tools/lit_control.sh on 2>&1 | tr -d '\r' | tee "$EV/lighting_lit_on.txt"
  echo '########## 收尾：换回自发光、光照回 day ##########'
  bash $SIM/tools/restore_default.sh 2>&1 | tr -d '\r' | tail -3
fi
echo '[lighting_round] DONE'
