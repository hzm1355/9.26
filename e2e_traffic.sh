#!/usr/bin/env bash
# 多车 + 轨迹记录回放 端到端验证（headless）
set +e
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
export GAZEBO_MODEL_PATH=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/models:/mnt/d/desktop1/9.26.1/gazebo_models-master:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/v2x_ws/install/v2x_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH
SHARE=$HOME/v2x_ws/install/v2x_demo/share/v2x_demo
D=$HOME/v2x_ws/src/v2x_demo/v2x_demo
TRAJ=/tmp/e2e_traj.json
rm -f "$TRAJ"

echo "[e2e] gzserver headless..."
gzserver "$SHARE/worlds/v2x_city.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so > /tmp/e2e_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break; kill -0 $GZ 2>/dev/null || { echo "gz died"; tail /tmp/e2e_gz.log; exit 1; }; sleep 1; done

echo "[e2e] traffic_flow + recorder..."
ros2 run v2x_demo traffic_flow_node.py --ros-args -p config_file:="$SHARE/config/traffic_flow.yaml" -p use_sim_time:=true > /tmp/e2e_flow.log 2>&1 &
FLOW=$!
ros2 run v2x_demo trajectory_recorder.py --ros-args -p output_file:="$TRAJ" -p use_sim_time:=true > /tmp/e2e_rec.log 2>&1 &
REC=$!
sleep 8

echo "=== (A) 生成日志 ==="
grep -i "生成\|spawn\|成功\|失败\|error" /tmp/e2e_flow.log | head

echo "=== (B) 车辆快照 t1 ==="
python3 "$D/check_positions.py" v2x_car
sleep 6
echo "=== 车辆快照 t2（应与 t1 不同 = 在行驶）==="
python3 "$D/check_positions.py" v2x_car

echo "=== (C) 保存轨迹并检查 JSON ==="
ros2 service call /trajectory_recorder/save std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i message
python3 - "$TRAJ" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
vs=d['vehicles']
print("vehicles:", list(vs))
for k,v in vs.items():
    print(f"  {k}: {len(v['samples'])} samples, x范围 {min(s[1] for s in v['samples']):.1f}~{max(s[1] for s in v['samples']):.1f}, y范围 {min(s[2] for s in v['samples']):.1f}~{max(s[2] for s in v['samples']):.1f}")
PY

echo "=== (D) 回放：生成幽灵并驱动 ==="
ros2 run v2x_demo trajectory_player.py --ros-args -p input_file:="$TRAJ" -p use_sim_time:=true > /tmp/e2e_play.log 2>&1 &
PLAY=$!
sleep 4
ros2 service call /trajectory_player/load std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i message
ros2 service call /trajectory_player/play std_srvs/srv/Trigger "{}" 2>/dev/null | grep -i message
sleep 3
echo "--- 幽灵快照 p1 ---"
python3 "$D/check_positions.py" playback_
sleep 4
echo "--- 幽灵快照 p2（应移动）---"
python3 "$D/check_positions.py" playback_

echo "=== (E) 错误检查 ==="
grep -i "error\|exception\|traceback\|segfault" /tmp/e2e_gz.log /tmp/e2e_flow.log /tmp/e2e_play.log 2>/dev/null | grep -vi "no error" | head

kill -9 $FLOW $REC $PLAY $GZ 2>/dev/null
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f traffic_flow 2>/dev/null; pkill -9 -f trajectory_ 2>/dev/null
echo "E2E_TRAFFIC_DONE"
