#!/usr/bin/env bash
# ============================================================================
# 区域触发底座端到端验证（WSL 内运行；前置：start_all.sh 已在另一终端常驻）
# 流程：等 /model_states 与区域话题就绪 -> 后台录 /v2x/region_events ->
#       驱车沿 NS-main 北向穿过 SPEED-NS-B 与 ETC-01_zone -> 打印捕获事件。
# 用法: sed 's/\r$//' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/e2e_regions.sh | bash
# ============================================================================
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

echo '[e2e] 1) 等待 /model_states 与 /v2x/region_events 就绪（最多 90s）...'
T=""
for i in $(seq 1 45); do
  T=$(ros2 topic list 2>/dev/null)
  if echo "$T" | grep -q model_states && echo "$T" | grep -q region_events; then
    echo "[e2e]    就绪（第 $i 次轮询）"
    break
  fi
  sleep 2
done
echo "$T" | grep -E 'model_states|region' || { echo '[e2e] FAIL: 话题缺失'; exit 1; }

echo '[e2e] 2) 后台录制 /v2x/region_events（70s）...'
timeout 70 ros2 topic echo /v2x/region_events > /tmp/region_events.log 2>&1 &
REC=$!
sleep 2

echo '[e2e] 3) 驱车北向 v=1.0 持续 18s（应穿过 SPEED-NS-B 与 ETC-01_zone）...'
sed 's/\r$//' "$SIM/drive_arc.sh" > /tmp/e2e_drive.sh
bash /tmp/e2e_drive.sh 1.0 0.0 18

sleep 3
kill $REC 2>/dev/null
echo '[e2e] 4) 捕获到的区域事件：'
grep -E '^region_id|^event_type|^vehicle_id|^speed' /tmp/region_events.log | paste - - - - 2>/dev/null || true
N=$(grep -c '^region_id' /tmp/region_events.log)
echo "[e2e] 事件总数: $N"
[ "$N" -ge 3 ] && echo '[e2e] PASS: 真实仿真中区域事件流正常' || echo '[e2e] FAIL: 事件过少，检查 /tmp/region_trigger.log'
