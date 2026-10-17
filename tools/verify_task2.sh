#!/bin/bash
# 任务二（信号灯配时仿真模块）条目 1~4 取证脚本
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

GRPS="J1_ns J1_ew J2_ns J2_ew J3_ns J3_ew"

detail() {
  timeout -k 3 8 ros2 topic echo --once /traffic_light/$1/state_detail 2>/dev/null |
    awk -v gid="$1" '
      /^vehicle_phase:/{vp=$2} /^phase_remaining:/{pr=$2}
      /^pedestrian_phase:/{pp=$2} /^pedestrian_remaining:/{pdr=$2}
      /^green_duration:/{g=$2} /^yellow_duration:/{y=$2} /^red_duration:/{r=$2}
      /^control_mode:/{cm=$2}
      END{printf "  %-7s 车=%-6s 剩=%6.2fs 人=%-9s 剩=%6.2fs G=%5.1f Y=%4.1f R=%5.1f 模式=%s\n",
                 gid, vp, pr, pp, pdr, g, y, r, cm}'
}

stats() {
  # stats 每个**周期**才发一条（周期最长 14 s），--once 的超时给到 30 s 才不会稳定读空。
  # 类型必须写全：这条话题历史上同时挂过 std_msgs/String 的旧发布者，届时 echo 不指定
  # 类型会直接报 "contains more than one type"。
  timeout -k 3 30 ros2 topic echo --once /traffic_light/$1/stats v2x_msgs/msg/TrafficLightStats 2>/dev/null |
    awk -v gid="$1" '
      /^vehicle_count:/{vc=$2} /^avg_wait_time:/{aw=$2} /^max_wait_time:/{mw=$2}
      /^throughput:/{tp=$2} /^total_vehicles:/{tv=$2} /^current_green_duration:/{cg=$2}
      /^adaptation_factor:/{af=$2} /^cycle_count:/{cc=$2}
      END{printf "  %-7s 本周期通过=%s 平均等待=%.2fs 最大等待=%.2fs 效率=%.2f辆/分 累计=%s 当前绿=%.2fs 系数=%.2f 周期数=%s\n",
                 gid, vc, aw, mw, tp, tv, cg, af, cc}'
}

echo "########## 条目1 灯色真实时序（机动车灯 + 行人灯，6 个信号组） ##########"
echo "--- 1.1 按仿真时钟记录灯态跳变时间轴（采样 55 s，含 J1_ns 完整周期与行人相位）---"
timeout -k 3 75 python3 -u /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/tl_timeline.py 30 | grep -E 'J1_ns|J2_ns|共 '
echo "--- 1.2 各信号组当前生效配时快照 ---"
for g in $GRPS; do detail $g; done

echo
echo "########## 条目2a 手动配时：全局一条指令改 6 组 绿8 黄2 红8 ##########"
# 走 /traffic_light/config 而不是 ros2 param set：前者是对外接口（带 config_ack 回执），
# param 只改管理器默认值、且没有回执可取证。
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"all\", \"green\": 8.0, \"yellow\": 2.0, \"red\": 8.0}"}' >/dev/null 2>&1
sleep 2
timeout -k 3 10 ros2 topic echo --once /traffic_light/config_ack std_msgs/msg/String 2>/dev/null | grep '^data:'
for g in $GRPS; do detail $g; done

echo
echo "########## 条目2b 单路口独立配置：J2_ns 绿4 黄2 红12（其余组不变） ##########"
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J2_ns\", \"green\": 4.0, \"yellow\": 2.0, \"red\": 12.0}"}' >/dev/null 2>&1
sleep 2
timeout -k 3 8 ros2 topic echo --once /traffic_light/config_ack std_msgs/msg/String 2>/dev/null | grep '^data:'
for g in J2_ns J2_ew J3_ns; do detail $g; done

echo
echo "########## 条目2c 多路口联动：J2_ew 反相跟随 J2_ns，J3_ns 相对 J1_ns 相位差 3s ##########"
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J2_ew\", \"anti_phase_of\": \"J2_ns\"}"}' >/dev/null 2>&1
ros2 topic pub --once /traffic_light/config std_msgs/msg/String \
  '{data: "{\"group\": \"J3_ns\", \"offset\": 3.0}"}' >/dev/null 2>&1
sleep 1
for k in 1 2 3 4 5 6; do
  printf '  t+%02ds ' $((k*2))
  for g in J1_ns J2_ns J2_ew J3_ns; do
    printf '%s=%s ' "$g" "$(timeout -k 3 6 ros2 topic echo --once /traffic_light/$g/state std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')"
  done
  echo
  sleep 2
done

echo
echo "########## 条目3a 三套预设方案切换 ##########"
for p in peak offpeak night default; do
  ros2 topic pub --once /traffic_light/preset std_msgs/msg/String "{data: $p}" >/dev/null 2>&1
  sleep 2
  printf '  preset=%-8s ' $p; detail J1_ns | sed 's/^  [A-Za-z0-9_]* //'
done

echo
echo "########## 条目3b 时段自动切换（早高峰/平峰/晚高峰/夜间时间表） ##########"
timeout -k 3 8 ros2 param get /traffic_light_manager enable_time_schedule 2>/dev/null | tail -1
grep -n -A6 'time_schedule:' ~/v2x_ws/install/v2x_demo/share/v2x_demo/config/traffic_light_params.yaml | head -12
echo "  当前实际时刻: $(date +%H:%M)（节点按此时刻自动选预设，手动切换预设会暂停自动调度）"

echo
echo "########## 条目3c 自适应配时（外部算法节点按车流量调整周期） ##########"
for g in $GRPS; do
  printf '  %-7s factor=%s ' $g \
    "$(timeout -k 3 6 ros2 topic echo --once traffic_light/$g/adaptive_factor std_msgs/msg/String 2>/dev/null | grep '^data:' | awk '{print $2}')"
  detail $g | sed 's/^  [A-Za-z0-9_]* //'
done
echo "  adaptive_stats: $(timeout -k 3 8 ros2 topic echo --once /traffic_light/adaptive_stats std_msgs/msg/String 2>/dev/null | grep '^data:' | cut -c7- | head -c 400)"

echo
echo "########## 条目4a 事件触发：强制绿通（指定方向） ##########"
ros2 topic pub --once /traffic_light/forced_green_group std_msgs/msg/String '{data: J3_ns}' >/dev/null 2>&1
sleep 1.5
printf '  强制 J3_ns 绿通: '; detail J3_ns
printf '  相邻 J3_ew 仍按周期: '; detail J3_ew
ros2 topic pub --once /traffic_light/forced_green_group std_msgs/msg/String '{data: none}' >/dev/null 2>&1
sleep 1.5
printf '  取消强制后 J3_ns 恢复周期: '; detail J3_ns

echo
echo "########## 条目4b 事件触发：急救车优先放行走廊（走廊绿、横向禁行） ##########"
ros2 topic pub --once /traffic_light/emergency_groups std_msgs/msg/String \
  '{data: "[\"J1_ns\", \"J2_ns\", \"J3_ns\"]"}' >/dev/null 2>&1
sleep 1.5
for g in $GRPS; do detail $g; done
ros2 topic pub --once /traffic_light/emergency_groups std_msgs/msg/String '{data: none}' >/dev/null 2>&1
sleep 1.5
printf '  走廊关闭后: '; detail J1_ns

echo
echo "########## 条目4c 事件触发：emergency 全场全绿（服务） ##########"
timeout -k 3 8 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: true}" 2>&1 | tail -1
sleep 1.5
for g in J1_ns J2_ns J3_ew; do detail $g; done
timeout -k 3 8 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool "{data: false}" 2>&1 | tail -1

echo
echo "########## 条目4d 路口通行效率 / 等待车辆数量统计输出 ##########"
for g in $GRPS; do stats $g; done
