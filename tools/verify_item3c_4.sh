#!/bin/bash
# 任务二 条目3c（自适应算法接入）+ 条目4（事件触发 + 统计输出）复验。
# 灯态读数一律用 tools/tl_detail.py（订阅 6 组 state_detail/stats 取最新一帧），
# 指令下发一律用 tools/tl_pub.py（先等订阅者再连发三次）——
# 上一轮这两处用 `ros2 topic echo --once` / `ros2 topic pub --once`，
# 分别吃过"粘滞半帧"和"DDS 首条丢失"的亏。
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
TOOLS=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools

pub() { timeout -k 3 20 python3 -u $TOOLS/tl_pub.py "$1" "$2" 3 >/dev/null 2>&1; }
det() { timeout -k 3 25 python3 -u $TOOLS/tl_detail.py 5 detail | sed -n '1,7p' | sed 's/^/  /'; }

echo "########## 实时栈存活确认 ##########"
echo "  实时节点数: $(pgrep -cf 'install/v2x[_]demo/lib')  自适应节点: $(pgrep -cf 'adaptive[_]timing_node')"

echo
echo "########## 条目3c 自适应算法：各组系数 + 管理器生效配时 ##########"
timeout -k 3 40 python3 -u $TOOLS/tl_detail.py 15 both | sed 's/^/  /'
echo
echo "########## 条目3c 算法侧原始输入与输出（/traffic_light/adaptive_stats + /model_states） ##########"
timeout -k 3 50 python3 -u $TOOLS/tl_adaptive_probe.py 25 | sed 's/^/  /'

echo
echo "########## 条目4a 事件触发：强制绿通（指定方向 J3_ns） ##########"
pub /traffic_light/forced_green_group J3_ns
sleep 1.5
echo "  --- J3_ns 被强制绿通，J3_ew 应仍走自身周期 ---"
det
pub /traffic_light/forced_green_group none
sleep 1.5
echo "  --- 取消强制后恢复周期（模式应回到 ADAPTIVE/FIXED，不再是 MANUAL） ---"
det

echo
echo "########## 条目4b 事件触发：急救车优先放行走廊（纵向绿、横向红、行人禁行） ##########"
pub /traffic_light/emergency_groups '["J1_ns", "J2_ns", "J3_ns"]'
sleep 1.5
det
echo "  管理器日志（走廊/禁行清单）:"
grep -a '急救优先放行' /tmp/signal.log | tail -2 | sed 's/.*\]: /    /'
timeout -k 3 25 python3 -u $TOOLS/tl_timeline.py 6 | grep -E '行人' | head -8 | sed 's/^/    /'
pub /traffic_light/emergency_groups none
sleep 1.5
echo "  --- 走廊关闭后恢复周期 ---"
det

echo
echo "########## 条目4c 事件触发：emergency 服务全场强制放行 ##########"
timeout -k 3 15 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool \
  '{data: true}' 2>&1 | tail -1 | sed 's/^/  /'
sleep 1.5
det
timeout -k 3 15 ros2 service call /traffic_light_manager/emergency std_srvs/srv/SetBool \
  '{data: false}' 2>&1 | tail -1 | sed 's/^/  /'
sleep 1.5
echo "  --- 解除后 ---"
det

echo
echo "########## 条目4d 通行效率 / 等待车辆数量统计（stats 每周期一条，读 20 s） ##########"
timeout -k 3 45 python3 -u $TOOLS/tl_detail.py 20 stats | sed 's/^/  /'
echo "  口径说明：vehicle_count 与 total_vehicles 在管理器里同增且全程不清零，"
echo "            所以两列恒等，属**累计**通过量；每周期通过量请看 stats 里 queue_length 与 cycle_count 增量。"
