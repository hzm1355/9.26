#!/bin/bash
# 任务二 条目2（手动配时：单路口独立 + 多路口联动）与 条目3a（多套预设方案）取证。
# 以 enable_adaptive:=false 重启信号栈：读到的是"人为设定的时长本身"，不与自适应系数混叠。
# 灯态读取统一走 /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/tl_detail.py（订阅 6 组 state_detail 取最新一帧），避免 echo --once 拿到半帧/旧帧。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

GRPS="J1_ns J1_ew J2_ns J2_ew J3_ns J3_ew"

detail() { timeout -k 3 25 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/tl_detail.py 5 detail; }

# 按 2s 间隔采样 6 组当前灯色，输出便于肉眼核对相位关系的表
snapshot() {
  printf '  %-6s ' "t+$1s"
  for g in $GRPS; do
    printf '%s ' "$(timeout -k 3 6 ros2 topic echo --once /traffic_light/$g/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print substr($2,1,1)}')"
  done
  echo
}

echo "########## 以「关闭自适应」方式重启信号栈（隔离变量） ##########"
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/stop_signal.sh > /dev/null 2>&1
setsid nohup ros2 launch v2x_demo traffic_signal_full.launch.py \
  enable_adaptive:=false enable_recorder:=false > /tmp/signal_np.log 2>&1 &
sleep 14
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
grep -E 'preset|groups=' /tmp/signal_np.log | head -3 | sed 's/^/  /'
echo "  预设方案定义（share/v2x_demo/config/traffic_light_params.yaml）:"
sed -n '/^presets:/,/^groups:/p' ~/v2x_ws/install/v2x_demo/share/v2x_demo/config/traffic_light_params.yaml \
  | grep -E 'presets|peak|offpeak|night|default' | sed 's/^/    /'

echo
echo "########## 条目3a 四套配时方案切换：G/Y/R 随方案改变（下一周期生效） ##########"
for p in default peak offpeak night; do
  # 必须走 tl_pub.py：`ros2 topic pub --once` 在 DDS 发现完成前就发完退出，这条预设会被静默吃掉。
  timeout -k 3 20 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/tl_pub.py \
    /traffic_light/preset "$p" 3 >/dev/null 2>&1
  sleep 24
  echo "  --- preset=$p ---"
  timeout -k 3 25 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/tl_detail.py 5 detail | sed -n '1,7p' | sed 's/^  /    /'
done

echo
echo "########## 条目2a 全局手动配时：绿8 / 黄2 / 红8（一条指令下发所有路口） ##########"
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"all\", \"green\": 8.0, \"yellow\": 2.0, \"red\": 8.0}"}' >/dev/null 2>&1
sleep 2
timeout -k 3 8 ros2 topic echo --once /traffic_light/config_ack std_msgs/msg/String 2>/dev/null | grep '^data:' | sed 's/^/  ack: /'
sleep 24
detail

echo
echo "########## 条目2b 单路口独立配置：只把 J2_ns 改成 绿4 / 黄2 / 红12 ##########"
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J2_ns\", \"green\": 4.0, \"yellow\": 2.0, \"red\": 12.0}"}' >/dev/null 2>&1
sleep 2
timeout -k 3 8 ros2 topic echo --once /traffic_light/config_ack std_msgs/msg/String 2>/dev/null | grep '^data:' | sed 's/^/  ack: /'
sleep 24
echo "  期望：仅 J2_ns 为 4/2/12，其余 5 组仍为 8/2/8（互不污染）"
detail
echo "  J2_ns 绿灯 4s / J2_ew 反相互补实测（G=绿 R=红 Y=黄）:"
for k in $(seq 1 9); do snapshot $((k * 2)); done

echo
echo "########## 条目2c 多路口联动：J3_ns 相对 J1_ns 相位差 3s（绿波偏移） ##########"
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J1_ns\", \"offset\": 0.0, \"green\": 8.0, \"yellow\": 2.0, \"red\": 8.0}"}' >/dev/null 2>&1
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J3_ns\", \"offset\": 3.0, \"green\": 8.0, \"yellow\": 2.0, \"red\": 8.0}"}' >/dev/null 2>&1
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J3_ew\", \"complementary\": true}"}' >/dev/null 2>&1
sleep 24
echo "  期望：J3_ns 的 GREEN 段整体比 J1_ns 晚 3s 开始（J1/J3 配时相同）"
printf '  %-6s %-8s %-8s %-8s %-8s\n' '时刻' 'J1_ns' 'J1_ew' 'J3_ns' 'J3_ew'
for k in $(seq 1 16); do
  a=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/J1_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  b=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/J1_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  c=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/J3_ns/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  d=$(timeout -k 3 6 ros2 topic echo --once /traffic_light/J3_ew/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')
  printf '  t+%-4s %-8s %-8s %-8s %-8s\n' "$((k*2))" "${a:-?}" "${b:-?}" "${c:-?}" "${d:-?}"
done

echo
echo "########## 恢复标准配置并带自适应重启（进入 5 车长跑 + 录制） ##########"
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/stop_signal.sh > /dev/null 2>&1
bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/start_signal.sh > /dev/null 2>&1
sleep 8
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')"
detail
