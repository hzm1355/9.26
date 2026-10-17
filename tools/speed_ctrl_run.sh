#!/bin/bash
# 任务四"受控单跑"取证：杀掉车队驱动 → 一辆车定速过两个点位 → 导出核对 → 恢复车队。
#
# 为什么要受控单跑：现场窗口里车队驱动偶发"复位风暴"（车被甩到 y=-35/x=36 这类界外、
# 互相嵌入弹开），600 s 能打出零事件，那种窗口连"真值速度"都不存在，读数对不对无从核对。
# 这里真值由脚本自己的高频 /model_states 插值算出，和检测节点的 L/t 同口径，可比。
#
# 用法：bash speed_ctrl_run.sh [JSON 输出目录=/tmp/v2x_records]
# 日志：/tmp/speed_ctrl_evidence.log
#
# CSV 回读放在 python 侧（speed_ctrl_evidence.py 的 show_csv），这里不再嵌 heredoc：
# 链式 python - <<PY 在 Git Bash 下会挂死，见 memory「脚本写成文件勿内联heredoc」。
set -e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
OUT=${1:-/tmp/v2x_records}
LOG=/tmp/speed_ctrl_evidence.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo "[ctrl] 1) 停掉车队驱动（它会抢 /car_2/cmd_vel 并把车复位走）"
for PID in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x_demo signal_drive_node' || true); do
  kill "$PID" 2>/dev/null || true
done
sleep 2
for PID in $(pgrep -f 'lib/v2x_demo/signal_drive_node' || true) \
           $(pgrep -f 'run v2x_demo signal_drive_node' || true); do
  kill -9 "$PID" 2>/dev/null || true   # rclpy 把 TERM 只当 shutdown 标志
done
sleep 1
N_DRV=$(pgrep -cf 'lib/v2x[_]demo/signal[_]drive[_]node' || true)
echo "  signal_drive 残留实例：${N_DRV:-0}"
if [ "${N_DRV:-0}" -ne 0 ]; then
  echo "!! 驱动没杀干净，它会和脚本抢 car_2 并把车复位走 —— 停跑"
  exit 1
fi

echo "[ctrl] 1.5) 先把全场恢复成静止且已知的现场（recover_fleet.py 自带净空递归与停稳校验）"
# 为什么必须有这一步：上一轮受控跑 car_4/car_2 的 settle 三次都是"落点偏差 18 m、
# 3 s 位移 17 m、沿车头 +x 方向"——那是摆位弹射的签名。原因是脚本接手时现场还在
# 复位风暴里（驱动刚被杀，车还带着 5~8 m/s 在乱飞），落点上正有一辆车掠过，
# 车体一重叠就被接触求解器沿车头轴弹出去。先 recover 到"全部静止 + 各在车道上"，
# 后面的每一次摆位才落在真空位上。
cd $D/tools
timeout -k 10 900 python3 -u recover_fleet.py || echo "  !! recover 有异常车，继续（下面还有停稳闸）"

echo "[ctrl] 2) 预检测速两节点（读位姿的数据源 + 落盘的记录器都得活着）"
# **两个节点都强制换新实例**：
#  * 记录器：导出的"内存累积"必须只含本次受控跑，不和历史窗口混在一张表里；
#  * 检测节点：跳变护栏的 warn 只打印前 5 次和第 100 次（jump_drop<=5 || %100==0），
#    而日志里的历史计数已经远超 5。换新实例 + 重定向日志后，护栏行才是可数的，
#    第 3 趟"注入瞬移 -> 护栏拦掉"才有可核对的证据。
bash $D/tools/start_speed_nodes.sh speed_detect_node.py speed_recorder_node.py 2>&1 | tail -4
echo "  测速话题：$(ros2 topic list 2>/dev/null | grep -cE '^/speed/' || true) 个"

# 护栏计数用日志行数差，不看进程内存：跳变护栏只在 warn 里体现
L0=$(grep -c '超物理上限' /tmp/speed_detect_node.log 2>/dev/null || true)
echo "[ctrl] 3) 开跑（当前护栏告警累计 ${L0:-0} 行）"

mkdir -p $OUT
RC=0
# -u：重定向到文件时 python 默认满块缓冲，长跑中途看不到任何进度（上一轮就是这么瞎的）
# 1500 s：三趟 ×最多 3 次重试，每趟含摆位落稳 + 24 m 定速通过（约 50 s）；
#         420 s / 900 s 都在重试中途被 timeout 掐过，前面几轮就是这么"跑到一半没下文"的。
timeout -k 5 1500 python3 -u $D/tools/speed_ctrl_evidence.py $OUT > $LOG 2>&1 || RC=$?
cat $LOG

L1=$(grep -c '超物理上限' /tmp/speed_detect_node.log 2>/dev/null || true)
echo "[ctrl] 4) 本趟跳变护栏新增 $(( ${L1:-0} - ${L0:-0} )) 行（受控单跑里应该接近 0）"
grep '超物理上限' /tmp/speed_detect_node.log 2>/dev/null | tail -3 | cut -c1-150 || true

echo "[ctrl] 5) 恢复车队（recover_fleet.py 会先递归挪开堵落点的车，再逐个验停稳）"
# 不用 reset_fleet.sh / place_cars.py：它们算出目标点就直接摆，不看那个点上现在有没有车，
# 车体重叠会被接触求解器沿车头轴弹射（最高 ~8 m/s），这就是复位风暴的触发方式。
bash $D/tools/recover_fleet.sh 2>&1 | tail -12

echo "[ctrl] 完成 rc=$RC，日志 $LOG，JSON $OUT/speed_ctrl_evidence.json"
