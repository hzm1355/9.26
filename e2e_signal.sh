#!/usr/bin/env bash
# 信号灯配时仿真 端到端验证（headless）
# 覆盖：灯态循环 / 手动配时 / 自适应调整 / 事件触发 / 录制回放
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH
SHARE=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
TL_LOG=/tmp/e2e_traffic_light_log.json
rm -f "$TL_LOG"

echo "[e2e-signal] gzserver headless..."
gzserver "$SHARE/worlds/v2x_city.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so > /tmp/e2e_signal_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break; kill -0 $GZ 2>/dev/null || { echo "gz died"; tail /tmp/e2e_signal_gz.log; exit 1; }; sleep 1; done

echo "[e2e-signal] traffic_light_manager + adaptive_timing + recorder..."
ros2 run v2x_demo traffic_light_manager.py --ros-args \
  -p config_file:="$SHARE/config/traffic_light_params.yaml" \
  -p use_sim_time:=true > /tmp/e2e_signal_mgr.log 2>&1 &
MGR=$!

ros2 run v2x_demo adaptive_timing_node.py --ros-args \
  -p use_sim_time:=true > /tmp/e2e_signal_adapt.log 2>&1 &
ADAPT=$!

ros2 run v2x_demo traffic_light_recorder.py --ros-args \
  -p output_file:="$TL_LOG" \
  -p use_sim_time:=true > /tmp/e2e_signal_rec.log 2>&1 &
REC=$!
sleep 8

echo "=== (A) 灯态循环验证 ==="
echo "--- 采样 traffic_light/J1_ns/state (应随时间变化) ---"
for i in 1 2 3; do
  ros2 topic echo --once /traffic_light/J1_ns/state 2>/dev/null | grep "data:" || echo "  (no msg)"
  sleep 5
done

echo "--- 采样行人灯 traffic_light/J1_ns/pedestrian ---"
ros2 topic echo --once /traffic_light/J1_ns/pedestrian 2>/dev/null | grep "data:" || echo "  (no msg)"

echo "=== (B) 手动配时验证：切换到 peak 预设 ==="
ros2 topic pub --once /traffic_light/preset std_msgs/msg/String "{data: 'peak'}" 2>/dev/null
sleep 3
grep -i "preset\|peak" /tmp/e2e_signal_mgr.log | tail -3

echo "=== (C) 自适应配时验证 ==="
sleep 6
echo "--- adaptive_factor 输出 ---"
grep -i "adaptive\|factor" /tmp/e2e_signal_adapt.log | tail -3
echo "--- adaptive_stats ---"
ros2 topic echo --once /traffic_light/adaptive_stats 2>/dev/null | head -5

echo "=== (D) 事件触发验证 ==="
echo "--- emergency 全绿 ---"
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: true}" 2>/dev/null | grep -i "message\|success"
sleep 2
echo "--- 紧急模式下灯态应全 GREEN ---"
ros2 topic echo --once /traffic_light/J1_ns/state 2>/dev/null | grep "data:"
ros2 topic echo --once /traffic_light/J1_ew/state 2>/dev/null | grep "data:"

echo "--- 关闭紧急模式 ---"
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: false}" 2>/dev/null | grep -i "message\|success"
sleep 2

echo "--- forced_green ---"
ros2 service call /traffic_light_manager/forced_green std_srvs/srv/SetBool "{data: true}" 2>/dev/null | grep -i "message\|success"
sleep 2
ros2 service call /traffic_light_manager/forced_green std_srvs/srv/SetBool "{data: false}" 2>/dev/null | grep -i "message\|success"

echo "=== (E) 录制回放验证 ==="
echo "--- 开始录制 ---"
ros2 service call /traffic_light_recorder/start std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i "message\|success"
sleep 8
echo "--- 停止录制 ---"
ros2 service call /traffic_light_recorder/stop std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i "message\|success"
echo "--- 保存 ---"
ros2 service call /traffic_light_recorder/save std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i "message\|success"

echo "--- 检查 JSON 文件 ---"
python3 - "$TL_LOG" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    meta = d.get('meta', {})
    events = d.get('events', [])
    print(f"  meta: groups={meta.get('groups')}, duration={meta.get('record_duration')}s, events={meta.get('event_count')}")
    if events:
        groups_seen = set(e['group'] for e in events)
        phases_seen = set(e.get('vehicle_phase', '') for e in events if e.get('vehicle_phase'))
        ped_seen = set(e.get('pedestrian_phase', '') for e in events if e.get('pedestrian_phase'))
        print(f"  groups: {groups_seen}")
        print(f"  vehicle phases: {phases_seen}")
        print(f"  pedestrian phases: {ped_seen}")
        has_detail = any(e.get('control_mode') for e in events)
        print(f"  has control_mode detail: {has_detail}")
        print(f"  PASS: {len(events)} events recorded")
    else:
        print("  WARN: no events recorded")
except Exception as e:
    print(f"  FAIL: {e}")
PY

echo "=== (F) 统计输出验证 ==="
echo "--- stats topic ---"
ros2 topic echo --once /traffic_light/J1_ns/stats 2>/dev/null | head -15 || echo "  (no stats msg - v2x_msgs may not be built)"

echo "=== (G) 错误检查 ==="
grep -i "error\|exception\|traceback\|segfault" /tmp/e2e_signal_gz.log /tmp/e2e_signal_mgr.log /tmp/e2e_signal_adapt.log /tmp/e2e_signal_rec.log 2>/dev/null | grep -vi "no error" | head

kill -9 $MGR $ADAPT $REC $GZ 2>/dev/null
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f traffic_light_manager 2>/dev/null
pkill -9 -f adaptive_timing 2>/dev/null; pkill -9 -f traffic_light_recorder 2>/dev/null
echo "E2E_SIGNAL_DONE"
