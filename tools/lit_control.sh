#!/usr/bin/env bash
# 条目 3 的对照实验：把牌面从"自发光(lighting off)"换成"受光(lighting on)"，
# 再看四档光照 + 夜间有/无补光下的识别率。
#
# 为什么要这个对照：默认牌面材质是 lighting off（当初为了 HSV 能找到牌，
# 见 plate_gen_node.py 里的注释），牌面亮度与场景光照解耦，所以逆光/弱光/夜间
# 三档量出来的识别率全都一样——这不叫"光照不影响识别"，这叫"对照组没做好"。
# lighting on 之后牌面跟着场景一起变暗，才轮到光照预设真正说话。
#
# 用法：bash tools/lit_control.sh [off|on]   默认 on；off 表示换回自发光
set -e -o pipefail
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
MODE=${1:-on}
unset ROS_DOMAIN_ID

echo '### 1/4 同步 v2x_demo 并重建（只这一包，秒级）'
rm -rf ~/v2x_ws/src/v2x_demo
cp -r $SIM/src/v2x_demo ~/v2x_ws/src/
find ~/v2x_ws/src/v2x_demo -name '*.py' -exec sed -i 's/\r//g' {} +
cd ~/v2x_ws
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
# 编译失败必须停在这里：`colcon | tail` 会把非零退出吞成 0，
# 于是 plate_gen 用旧代码重启，量出来的"对照组"其实是自发光那一轮，结论整个反过来。
if colcon build --symlink-install --packages-select v2x_demo > /tmp/v2x_build_demo.log 2>&1; then
  echo 'BUILD_OK'
else
  echo 'BUILD_FAILED'; tail -20 /tmp/v2x_build_demo.log; exit 1
fi

echo '### 2/4 唯一地重启 plate_gen（lit:='$MODE'）+ 下发五副牌并校验材质'
# 必须走 restart_plate_gen.sh：这里以前是 `pgrep | head -1` 只杀一个实例，
# 来回切几次就攒出 6 个 plate_gen（on/off 混着），它们都应答 /plate/request，
# 车上到底是哪张材质变成随机的——两轮对照因此整个作废。
bash $SIM/tools/restart_plate_gen.sh $MODE
# 刚重建过 v2x_demo，识别器活着也不代表跑的是新代码
FORCE=1 bash $SIM/tools/ensure_recognizer.sh

echo '### 3/3 跑五档：白天/逆光/弱光/夜间无补光/夜间有补光（--both：每档两个卡口各一张）'
echo '    （gate_lighting_test 开头会自己下发五个真值车牌，新材质就在这里生成）'
# --both 是这轮的要点：受光对照第一轮只等"该档第一张"，样本全落在南环就是"正常"、
# 全落在北环就是"丢牌"，档与档之间根本不可比。每档强制南北各一张才消掉这个偏差。
python3 $SIM/tools/gate_lighting_test.py --both day backlit lowlight \
  custom:0.03,0.02,0.05,0,0 custom:0.03,0.02,0.05,0,1 150 2>&1 | tr -d '\r'
echo '[lit_control] DONE'
