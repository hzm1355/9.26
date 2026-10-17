#!/bin/bash
# 拉起 gzclient 连到已在跑的 gzserver，并把"活没活"落成可复查的证据。
# 关键：客户端必须和服务端共用 GAZEBO_MODEL_PATH，否则解析不到
# model://room_33/meshes/room_33.obj，会在 "Preparing your world..." 处静默退出。
# 用法：wsl -e bash /tmp/open_gui77.sh [显示地址]
set -e

DISPLAY_IN="${1:-172.26.32.1:1}"
LOG="/tmp/gzgui_$(date +%m%d_%H%M%S).log"

export DISPLAY="$DISPLAY_IN"
export GAZEBO_MODEL_PATH="/home/hzm/v2x_models:/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="/home/hzm/v2x_ws/install/v2x_gazebo_plugins/lib:/home/hzm/v2x_ws/install/smart_car_sim/lib:${GAZEBO_PLUGIN_PATH}"

cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim || exit 1

echo "显示=$DISPLAY  日志=$LOG"

if pgrep -f 'gzcli[e]nt --verbose' > /dev/null; then
  echo "gzclient 已在运行：$(pgrep -f 'gzcli[e]nt --verbose' | tr '\n' ' ')"
else
  setsid nohup gzclient --verbose > "$LOG" 2>&1 &
  echo "已发起，10 s 后首检"
fi

last=0
for w in 10 40 100; do
  sleep $((w - last))
  last=$w
  pids=$(pgrep -f 'gzcli[e]nt --verbose' | tr '\n' ' ' || true)
  if [ -z "$pids" ]; then
    echo "[${w}s] 已退出"
    break
  fi
  echo "[${w}s] 存活 PID=$pids"
done

echo "=== 日志行数：$(wc -l < "$LOG" 2>/dev/null || echo '文件不存在')"
echo "=== 非刷屏行（原始，不判空）："
grep -vE 'Unable to get Material|No link visual with id' "$LOG" 2>/dev/null | tail -n 40 || true
