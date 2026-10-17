#!/usr/bin/env bash
# 循环采样左右前轮转向节角度（gz model 真值），用于诊断转向锁死问题
export GAZEBO_MASTER_URI=http://localhost:11345
ITER=${1:-12}
for i in $(seq 1 "$ITER"); do
  A=$(gz model -m v2x_prius -i 2>/dev/null | grep -E 'name: "v2x_prius::front_(left|right)_steer_joint"' -A2 | grep 'angle:' | tr -d ' ' | tr '\n' ' ')
  echo "[steer $i] $A"
  sleep 1
done
echo "[steer] 采样结束"
