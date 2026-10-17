#!/bin/bash
# 归位之后的驱动侧复检：只读，不动任何进程。
# 为什么还要再跑一趟：pk201 的 S9 量的是"累计路程"，而驱动节点的**复位本身就是瞬移**，
# 会被累加成路程 —— 换句话说 S9 的"在走"证明不了"顺走"。真正的判据是驱动日志里
# "复位/出界"这一类行**还在不在续发**：续发说明摆位与 diff_drive 插件的 odom 目标没对齐
# （memory「摆位弹射与复位风暴」的第三类能量源），车会一直被打断重定位。
# 顺带查两件事：有没有 gz `player` 在回放（它会直接改写实体位姿，摆位永远钉不住），
# 以及每辆车的 odom 位姿与 model_states 位姿差多少（差得远就是同一套病的根因读数）。
set +e
LOG=/tmp/signal_drive_home320.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1

echo '=== 1) 回放进程（player 一旦在跑，任何摆位都会被它的帧覆盖）'
pgrep -af 'gz sim|gzserver|player' 2>/dev/null | cut -c1-100 | sed 's/^/  /'

echo '=== 2) 驱动日志：复位/出界/离道 这类行的总数，与最近 30 s 内是否还在续发'
echo "  日志总行数：$(wc -l < "$LOG" 2>/dev/null; true)"
for K in 复位 出界 离道 重取航段 巡航; do
  echo "  含「$K」的行数：$(grep -ac "$K" "$LOG" 2>/dev/null; true)"
done
echo '  ## 尾 14 行'
tail -14 "$LOG" 2>/dev/null | cut -c1-170 | sed 's/^/    /'

echo '=== 3) 24 s 逐台累计路程（这一版只看不判定：期望是"顺走"，即路程≈巡航×时间）'
timeout -k 5 60 python3 tools/pk203_track320.py 24 2>&1 | sed 's/^/  /'

echo '=== 4) 每辆车 odom 位姿 vs 模型位姿（差得远=插件 odom 与实际不同步）'
timeout -k 5 45 python3 tools/pk206_odomgap320.py 2>&1 | sed 's/^/  /'
echo 'PK205DONE'
