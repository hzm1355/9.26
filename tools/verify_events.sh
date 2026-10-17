#!/bin/bash
# 需求4/5 回归验证：事件触发（急救优先）+ 全流程录制（含 redesigned 灯模）
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
D=/mnt/d/desktop1/9.26.1

echo '=== 1) 录制文件保存 ==='
ros2 service call /traffic_light_recorder/save std_srvs/srv/Trigger 2>/dev/null \
  | grep -E 'message:' | head -2
cp /tmp/tl_log.json $D/tl_log2.json 2>/dev/null || true

echo '=== 2) 事件构成 ==='
python3 - <<'PY'
import json
d = json.load(open('/tmp/tl_log.json'))
ev = d.get('events', d) if isinstance(d, dict) else d
meta = {}
for e in ev:
    k = ('车辆' if e.get('type') == 'vehicle' else '灯态')
    meta[k] = meta.get(k, 0) + 1
print('总事件', len(ev), meta)
print('参与车辆', d.get('vehicles') if isinstance(d, dict) else '?')
gs = {}
for e in ev:
    if e.get('type') != 'vehicle':
        gs[e.get('group', '?')] = gs.get(e.get('group', '?'), 0) + 1
print('灯态事件按组:', gs)
PY

echo '=== 3) 急救优先放行（EMERGENCY）==='
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool '{data: true}' \
  | grep -E 'success|message' | head -2
sleep 4
for g in J1_ns J1_ew J2_ns; do
  printf '  %s 机动=%s 行人=%s 模式=%s\n' "$g" \
    "$(timeout 2 ros2 topic echo --once traffic_light/$g/state std_msgs/msg/String | grep '^data:' | cut -c7-)" \
    "$(timeout 2 ros2 topic echo --once traffic_light/$g/pedestrian std_msgs/msg/String | grep '^data:' | cut -c7-)" \
    "$(timeout 3 ros2 topic echo --once traffic_light/$g/state_detail 2>/dev/null | grep 'control_mode' | awk '{print $2}')"
done
curl -s -o $D/emergency_now.png "http://localhost:8080/cam/inter"
echo '=== 4) 解除优先 ==='
ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool '{data: false}' \
  | grep -E 'success' | head -1
sleep 3
timeout 3 ros2 topic echo --once traffic_light/J1_ns/state_detail 2>/dev/null | grep control_mode
