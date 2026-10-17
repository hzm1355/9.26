#!/bin/bash
# 任务二 条目2c（多路口联动配时）量化取证。
# 全部走 enable_adaptive:=false 的隔离栈：offset 是"起绿时刻差"，
# 若各组自适应系数不同，周期长度就不等，3 s 固定相位差会随时间漂移，量出来没意义。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

pub() { timeout -k 3 20 python3 -u $TOOLS/tl_pub.py /traffic_light/config "$1" 3; }

echo "########## 隔离栈（关闭自适应） ##########"
bash $TOOLS/stop_signal.sh > /dev/null 2>&1
setsid nohup ros2 launch v2x_demo traffic_signal_full.launch.py \
  enable_adaptive:=false enable_recorder:=false > /tmp/signal_np.log 2>&1 &
sleep 14
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"

echo
echo "########## 下发联动配置（三对均 绿8/黄2/红8，只差联动关系） ##########"
# J1_ns / J3_ns 配时相同，J3_ns 晚 3 s 起绿（绿波偏移）
pub '{"group": "J1_ns", "offset": 0.0, "green": 8.0, "yellow": 2.0, "red": 8.0}'
pub '{"group": "J3_ns", "offset": 3.0, "green": 8.0, "yellow": 2.0, "red": 8.0}'
# 同路口东西向 complementary：偏移自动取本组红时长，与南北向严格错开
pub '{"group": "J3_ew", "complementary": true, "green": 8.0, "yellow": 2.0, "red": 8.0}'
pub '{"group": "J1_ew", "complementary": true, "green": 8.0, "yellow": 2.0, "red": 8.0}'
# 跨路口严格反相：J2_ew 相对 J2_ns 晚 (绿+黄) 起绿
pub '{"group": "J2_ns", "offset": 0.0, "green": 8.0, "yellow": 2.0, "red": 8.0}'
pub '{"group": "J2_ew", "anti_phase_of": "J2_ns", "green": 8.0, "yellow": 2.0, "red": 8.0}'
sleep 1
timeout -k 3 20 python3 -u $TOOLS/tl_detail.py 4 detail | sed -n '1,7p' | sed 's/^/  /'

echo
echo "########## 起绿时刻 / 相位差 / 反相重叠量化（采样 90 s 仿真时钟） ##########"
timeout -k 3 120 python3 -u $TOOLS/tl_offset_audit.py 90 | sed 's/^/  /'

echo
echo "########## 恢复带自适应的标准信号栈 ##########"
bash $TOOLS/stop_signal.sh > /dev/null 2>&1
bash $TOOLS/start_signal.sh > /dev/null 2>&1
sleep 6
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
