#!/bin/bash
# 拉起停车场模块节点（管控 manager + 记录 recorder + 摆渡车 shuttle ×N），
# 并做"实例数对得上、话题有发布者、上场车真在 world 里"的三重确认。
#
# 2026-10-14：摆渡车从一辆（car_6）改成两辆分队（car_3 走白名单月租、car_2 走临时车
# 登记），航线由 parking_zones.yaml 的 shuttle.fleets.<模型名> 给。
#
# 默认只起 **car_3 一辆**，不是漏了 car_2：停车场是**单车道**的（入口/出口车道、通道带
# 复位段全都是同一条），两个 shuttle 实例各自按 loop_period_s 自主循环，一旦同时在线
# 就必然在通道带对顶 —— 那不是"调度"，是撞车。所以分队是**按时间串行**用的：
#   SHUTTLE_MODELS=car_3  …跑完进场、停在 P3   → 杀掉 shuttle（车留在车位里，不挡道）
#   SHUTTLE_MODELS=car_2  …跑进场→出场→复位    → 再起 car_3 演它的出场
# 每段都单独重跑本脚本传模型名即可；car_6 退役但**模型仍留在 world 里**（不删），
# 出生位在 parking_zones.yaml 的 shuttle.park_pose 里挪到了沙盘东南角 (23.5,-23.5)，
# 不再给它起节点。
# 为什么这三重确认都不能省（都是任务三/四踩过的坑，见 memory）：
#  * 同名实例 >1：manager 会双份转发 BarrierState、双份下发抬杆指令，recorder 更狠 ——
#    同一条 ParkingPass 落两行 JSONL，条目 5 的"命中 N 条"直接翻倍；
#  * 只数进程不看话题：manager 起得来但插件的 /parking/gate/*/state 没接上时，它会
#    永远停在"等核验"，看起来一切正常、零通行记录；
#  * shuttle 靠 gzserver 的模型位姿驱动，world 里没这辆模型时它不报错、只是不动，
#    于是录出来的视频是"面板数字在走、画面里什么都没有"。上场车是 world 里的常驻
#    模型，运行期那个 gzserver 要是比 world 文件旧，就必须整栈重启。
#
# 用 set +e：本脚本的收尾是"报告 + 退出码"，检查项里的 grep/timeout/head 天然会非零
# （memory「set -e 里 grep 无匹配会掐断脚本」），一旦中途退出，后面的实例数与话题检查
# 全不执行，看起来像"节点起来了"其实什么都没验。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
CFG=$SIM/src/v2x_demo/config/parking_zones.yaml
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

# 默认重启三个节点；只想让白名单/几何改动热生效时传 `parking_manager_node.py` 单独重启。
# recorder 一重启，内存里的环形记录就清零（/parking/query 查的是内存，JSONL 只是追加日志），
# 所以录条目 5 之前别随便重跑这个脚本。
NODES="${*:-parking_manager_node.py parking_recorder_node.py parking_shuttle_node.py}"
SHUTTLE_MODELS="${SHUTTLE_MODELS:-car_3}"
N_SHUTTLE=$(echo $SHUTTLE_MODELS | wc -w)

mkdir -p /tmp/v2x_parking /tmp/v2x_parking_in /tmp/v2x_parking_out

for NODE in $NODES; do
  # 先杀干净：rclpy 接管 SIGTERM，主线程不在 spin 里的常驻节点 TERM 杀不掉，要补 KILL。
  # 带 lib/ 前缀的匹配式不会命中本脚本自己（写在文件里的模式也不会自匹配）。
  # 这里按**节点脚本名**整片杀，两辆摆渡车是同一份脚本起的两个实例，都要带走
  # （memory「重启常驻节点要杀光同名所有实例」）。
  for PID in $(pgrep -f "lib/v2x_demo/$NODE"); do
    echo "杀掉旧实例 $NODE (pid $PID)"
    kill "$PID" 2>/dev/null
    sleep 1
    kill -9 "$PID" 2>/dev/null
  done
done

# 摆渡车是 cmd_vel 发布者：重启它之前先把车刹住，否则旧实例最后一拍的巡航速度
# 会一直作用到差速轮的默认 1 s 超时（memory「杀掉 cmd_vel 发布者后必须补发零速」）。
if echo "$NODES" | grep -q parking_shuttle_node; then
  for M in $SHUTTLE_MODELS; do
    timeout -k 3 15 ros2 topic pub --once /$M/cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1
  done
fi

# 顺序：manager 先起（BarrierState 转发与判定是权威），recorder 晚几秒只影响它
# 收不收得到开机那几条，shuttle 最后（它要订 manager 转发的 gate_state）。
for NODE in $NODES; do
  if [ "$NODE" = "parking_shuttle_node.py" ]; then
    for M in $SHUTTLE_MODELS; do
      # __node 重映射：两个实例的 ROS 节点名必须分开，否则 rclpy 里同名节点在
      # `ros2 node list` 只显一份，排查时分不清哪个 pid 在开哪辆车。
      setsid nohup ros2 run v2x_demo parking_shuttle_node.py --ros-args \
        --remap __node:=shuttle_$M -p use_sim_time:=true \
        -p "config_file:=$CFG" -p "model:=$M" \
        > "/tmp/shuttle_$M.log" 2>&1 < /dev/null &
      sleep 3
    done
    continue
  fi
  # config_file 显式指 D 盘源码树：改 yaml 点位后只要重跑本脚本就生效，不必 colcon 重装
  # （与 vid_common.sh 里 mgr_restart 传 traffic_light_params.yaml 同一做法）。
  setsid nohup ros2 run v2x_demo "$NODE" --ros-args -p use_sim_time:=true \
    -p "config_file:=$CFG" > "/tmp/${NODE%.py}.log" 2>&1 < /dev/null &
  sleep 3
done
sleep 5

FAIL=0
echo "--- 实例数（manager/recorder 各 1，shuttle 共 $N_SHUTTLE：$SHUTTLE_MODELS）---"
for NODE in $NODES; do
  N=$(pgrep -cf "lib/v2x[_]demo/${NODE%.py}")
  WANT=1
  [ "$NODE" = "parking_shuttle_node.py" ] && WANT=$N_SHUTTLE
  echo "  ${NODE%.py} python 实例 ${N:-0}（应为 $WANT）"
  [ "$N" = "$WANT" ] || FAIL=1
done
for M in $SHUTTLE_MODELS; do
  echo "  ## shuttle_$M 日志尾"
  tail -3 "/tmp/shuttle_$M.log" 2>/dev/null | cut -c1-150 | sed 's/^/     /'
done

echo "--- 各节点日志尾 ---"
for NODE in $NODES; do
  [ "$NODE" = "parking_shuttle_node.py" ] && continue
  echo "  ## ${NODE%.py}"
  tail -4 "/tmp/${NODE%.py}.log" 2>/dev/null | cut -c1-150 | sed 's/^/     /'
done

echo "--- 话题发布者计数 ---"
for T in /parking/gate_state /parking/pass /parking/actions /parking/shuttle_status; do
  printf '  %-26s ' "$T"
  P=$(timeout -k 2 12 ros2 topic info "$T" 2>&1 | tr -d '\r' | grep -a 'Publisher count')
  echo "${P:-读不到（话题不存在）}"
  echo "$P" | grep -qE 'Publisher count: [1-9]' || { echo "     !! 没有发布者"; FAIL=1; }
done

echo "--- 闸机动力学是否接上了（manager 转发的 BarrierState 里要有实测角）---"
# 要重试：刚 launch 的节点 DDS 发现没完成，`--once` 会瞬时读空（整栈重启后头几分钟
# 实测误报过），空回包不等于"没接上"。
GS=
for i in 1 2 3 4; do
  # 类型名要么不写（让 ros2 自己按话题元数据取），要么写全名 v2x_msgs/msg/BarrierState：
  # 裸写 BarrierState 解析不到类型，四次重试全成空回包，看着像"插件没加载"。
  GS=$(timeout -k 3 15 ros2 topic echo /parking/gate_state --no-daemon --once 2>&1 \
        | tr -d '\r' | grep -aE 'gate_id|state:|target_rad|angle_rad|loop_occupied')
  [ -n "$GS" ] && break
  echo "  第 $i 次采样空回包（发现竞态），重试"
  sleep 10
done
if [ -n "$GS" ]; then
  echo "$GS" | head -8 | sed 's/^/  /'
else
  echo "  !! 四次都读不到 BarrierState：libv2x_barrier 插件没加载，或 world 里没有闸机"
  FAIL=1
fi

echo "--- 上场摆渡车是否真在运行期 world 里 ---"
# `gz model -m X -p` 存在时只回一行六个位姿数，不存在时回 "Unable to get info on
# model[X] in the world[default]" —— 那句里也带数字（car_3 的 3），所以判据只能认
# "Unable"，不能认"有没有数字"。
for M in $SHUTTLE_MODELS; do
  P=$(gz model -m $M -p 2>&1 | head -3)
  if echo "$P" | grep -aq 'Unable'; then
    echo "  !! world 里没有 $M（读回：$(echo "$P" | tr -d '\n' | cut -c1-70)）"
    echo "     运行期 gzserver 比 world 文件旧，停车场动力学没法演。"
    echo "     需要整栈重启（tools/restart_clean_stack.sh）——这会打断车队与其它取证，请先确认。"
    FAIL=1
  else
    echo "  $M 位姿：$(echo "$P" | tr -d '\n' | cut -c1-60)"
  fi
done

echo "--- 服务 ---"
SV=$(ros2 service list 2>/dev/null | grep -aE '^/parking/(gate|whitelist|query|export)$')
if [ -n "$SV" ]; then echo "$SV" | sed 's/^/  /'; else echo "  (四个服务一个都没注册，manager/recorder 没起来)"; FAIL=1; fi

[ "$FAIL" = 0 ] && echo "OK：停车场节点就绪（摆渡 $SHUTTLE_MODELS）" || echo "!! 上面有异常，先处理再录"
exit $FAIL
