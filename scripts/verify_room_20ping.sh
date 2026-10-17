#!/bin/bash
# 最终验证：gzserver@11346 加载 room_20ping，正向确认模型存在且仿真步进。
# 隔离于用户 11345 的 room_layout 会话。
MODELS="/home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/models"
SYSMODELS="/usr/share/gazebo-11/models"
export GAZEBO_MASTER_URI="http://localhost:11346"
export GAZEBO_MODEL_PATH="$MODELS:$SYSMODELS:${GAZEBO_MODEL_PATH:-}"
unset DISPLAY
unset LIBGL_ALWAYS_INDIRECT

WORLD="/mnt/d/desktop1/9.26.1/v2x_traffic_sim/scripts/verify_20ping.world"
LOG=/tmp/verify_20ping.log

echo "启动 gzserver @11346 ..."
gzserver "$WORLD" --verbose > "$LOG" 2>&1 &
GZPID=$!
echo "gzserver pid=$GZPID，等待 45s 加载 45MB 网格..."
sleep 45

echo ""
echo "=== 进程存活? ==="
if kill -0 "$GZPID" 2>/dev/null; then echo "OK: gzserver 仍在运行(未崩溃)"; else echo "FAIL: gzserver 已退出"; fi

echo ""
echo "=== gz model -i -m room_20ping (模型状态=正向确认) ==="
timeout 12 gz model -i -m room_20ping 2>&1 | head -40

echo ""
echo "=== gz stats -d 2 (仿真步进) ==="
timeout 12 gz stats -d 2 2>&1 | head -6

echo ""
echo "=== 日志中真实错误(排除headless渲染警告) ==="
grep -iE "error|fail|exception|cannot load|parse|\.obj|mesh" "$LOG" | grep -viE "RenderEngine|render path|X window|open display" | head -20
echo "(以上为空=无网格/解析错误)"

echo ""
echo "=== 清理 ==="
kill "$GZPID" 2>/dev/null
pkill -f verify_20ping 2>/dev/null
sleep 2
echo "剩余 verify_20ping 进程: $(pgrep -f verify_20ping | wc -l) (应为0)"
echo "用户 room_layout gzserver 仍在: $(pgrep -f room_layout.world | wc -l) (应为1)"
echo "=== 完成 ==="
