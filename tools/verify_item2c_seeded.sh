#!/bin/bash
# 任务二 条目2c（多路口联动）—— **播种态**量化取证。
#
# 与 verify_item2c.sh 的区别：那份在运行中用 /traffic_light/config 改 offset，
# 而管理器的 offset 只在状态机播种时读取（traffic_light_manager.py::phase_for 里
# `st is None or now < st['start']` 才会 _seed_state），运行中改 offset 对已在跑的组
# 不重播种，量出来会混入"播种时刻的配时残留"（实测 J2_ew 偏 5 s、J3_ew 偏 8 s）。
# 这一份把 6 组统一 8/2/8 与三对联动关系一起写进配置文件再冷启动管理器，
# 得到的就是设计语义本身：offset=3.0 -> 起绿相位差精确 3.00 s；
# complementary / anti_phase_of -> 两两"同时为绿"0.00 s。
#
# 期望值（周期 = 8+2+8 = 18 s，相对基准组 J1_ns）：
#   J1_ns 0 / J1_ew -8（complementary，offset 自动取红时长 8）/ J2_ns 0 /
#   J2_ew +8（anti_phase_of：0+绿8+黄2=10，起绿时刻 -10 ≡ +8）/
#   J3_ns -3（offset 3.0，正 offset 使相位提前）/ J3_ew +5（3+8+2=13，-13 ≡ +5）
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

echo "########## 1) 生成播种态配置 ##########"
python3 -u $TOOLS/mk_link_cfg.py /tmp/tl_link.yaml

echo
echo "########## 2) 停实时栈，用该配置冷启动一个管理器（不带自适应/录制/驱动） ##########"
bash $TOOLS/stop_signal.sh > /dev/null 2>&1
setsid nohup ros2 run v2x_demo traffic_light_manager.py \
  --ros-args -p config_file:=/tmp/tl_link.yaml -p enable_time_schedule:=false \
  > /tmp/tl_link.log 2>&1 < /dev/null &
sleep 12
echo "  管理器实例数: $(pgrep -cf 'traffic_light[_]manager.py')"
grep -a 'TrafficLightManager:' /tmp/tl_link.log | sed 's/.*\]: //; s/^/  /'

echo
echo "########## 3) 起绿相位差 + 两两同时为绿重叠（采样 90 s 仿真时钟） ##########"
timeout -k 3 130 python3 -u $TOOLS/tl_offset_audit.py 90 J1_ns | sed 's/^/  /'

echo
echo "########## 4) 恢复标准信号栈（带自适应 + 录制 + 5 车按灯行驶） ##########"
pkill -f 'traffic_light[_]manager.py' 2>/dev/null || true
sleep 2
bash $TOOLS/start_signal.sh > /dev/null 2>&1
sleep 6
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
