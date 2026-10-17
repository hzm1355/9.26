#!/bin/bash
# 235：把 D 盘源码同步进运行期工作区并 colcon 重装，然后 md5 硬闸复核。
#
# 为什么这一轮必须先做这个而不是直接录：restart234 的自检里那行
#   [shuttle] car_3 就绪：航点 17 个（进场 7 / 出场 7 / 复位 3）
# 看着像分队生效了，其实运行期那份拷贝（~/v2x_ws/src/v2x_demo，10-12 16:05 的**实体
# 拷贝**，install/ 又是它的拷贝）里 `grep -c fleets` = 0 —— 它根本没读过 fleets，
# 那 17 个点来自 yaml 顶层那三段（默认三段现在也是 7+7+3，所以点数一模一样，
# 日志读不出差别）。同时 signal_drive_node 也在跑旧拷贝，不认识 drive:false，
# 于是 place_cars 把 car_2/car_3 从停车场出生位拽回环路，而 shuttle_car_3 又在同
# 一时刻往 /car_3/cmd_vel 发停车场的航线 —— 两路指令抢同一辆车。
# 这正是 memory「先确认运行期加载的是哪份文件副本」点名的那种坑：改了没生效，
# 而日志长得和"改了没用"一模一样。
#
# 只同步**内容**真不同的文件：parking_shuttle_node.py（fleets+状态行带模型名）、
# signal_drive_node.py（drive:false 消费者）、parking_shuttle_selftest.py、
# config/signal_drive_params.yaml（东腿 20.80 + drive:false）。
# 2026-10-14 起把 parking_zones.yaml / speed_zones.yaml 也一并同步：这两个节点的
# config_file 虽然显式指 D 盘（-p config_file:=/mnt/d/...），但 install/share 下那份
# 是 ws/src 的符号链接，留着 fleets=0 的旧内容就是一个坑 —— 下一个调用方只要按
# share 路径取参数，就会静默退回 car_6 时代的三段航线，而日志与"改了没用"一模一样。
# __init__.py / traffic_light_controller.py 在 cmp 里"不同"只是行尾差异
# （diff 呈现 1,93c1,93 全文不同、肉眼看逐字一致 = CRLF/LF），内容一字不差，不动它们。
set +e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo
W=$HOME/v2x_ws/src/v2x_demo
I=$HOME/v2x_ws/install/v2x_demo
RC=0

echo '== 1 语法闸：编译前先 py_compile，语法错绝不进工作区'
for F in v2x_demo/parking_shuttle_node.py v2x_demo/signal_drive_node.py \
         v2x_demo/parking_shuttle_selftest.py; do
  python3 -m py_compile "$D/$F" 2>&1 | tr -d '\r' | sed 's/^/  /'
  if [ $? -ne 0 ]; then
    echo "  FAIL py_compile $F —— 不同步"
    RC=1
  else
    echo "  ok py_compile $F"
  fi
done
[ "$RC" = 0 ] || exit 1

echo '== 2 同步（D 盘为准）'
for F in v2x_demo/parking_shuttle_node.py v2x_demo/signal_drive_node.py \
         v2x_demo/parking_shuttle_selftest.py config/signal_drive_params.yaml \
         config/parking_zones.yaml config/speed_zones.yaml; do
  cp "$D/$F" "$W/$F" || { echo "  FAIL 拷贝 $F"; exit 1; }
  A=$(md5sum < "$D/$F" | cut -d' ' -f1)
  B=$(md5sum < "$W/$F" | cut -d' ' -f1)
  [ "$A" = "$B" ] && echo "  ok src  $A  $F" || { echo "  FAIL src 两份不一致 $F ($A/$B)"; exit 1; }
done

echo '== 3 colcon 重装 v2x_demo（install/ 是 src 的实体拷贝，不重装就还是旧码）'
cd $HOME/v2x_ws || exit 1
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
timeout -k 10 300 colcon build --packages-select v2x_demo 2>&1 | tail -6 | tr -d '\r' | sed 's/^/  /'

echo '== 4 复核：运行期真正加载的那份 install 拷贝里，新代码的特征必须在'
# install 的布局是 lib/v2x_demo/*.py 与 share/v2x_demo/config/*.yaml（节点从 lib 起，
# launch 的 config_file 从 share 取）；写成 install/v2x_demo/v2x_demo/*.py 会一律
# "No such file"，那读起来就像"同步没生效"，其实是判据脚本自己路径猜错。
LIB=$I/lib/v2x_demo
SHARE=$I/share/v2x_demo
for PAIR in "$LIB/parking_shuttle_node.py:fleets" \
            "$LIB/signal_drive_node.py:ccfg.get('drive'" \
            "$SHARE/config/signal_drive_params.yaml:drive: false" \
            "$SHARE/config/parking_zones.yaml:fleets:" \
            "$SHARE/config/speed_zones.yaml:20.8, 0.5"; do
  F=${PAIR%%:*}; PAT=${PAIR#*:}
  N=$(grep -ac "$PAT" "$F" 2>/dev/null)
  echo "  ${F#$HOME/}  命中「$PAT」 $N 次"
  [ "${N:-0}" != "0" ] || RC=1
done
echo '  负样本：运行期配置里不得再出现旧东腿（14.5 / 16.2 都横穿停车场占地或 P3）'
grep -a '\[20.8,' "$SHARE/config/signal_drive_params.yaml" | wc -l | sed 's/^/    route 里 20.8 出现 /'
grep -a '\[16.2,' "$SHARE/config/signal_drive_params.yaml" | wc -l | sed 's/^/    route 里 16.2 出现 /'
grep -a '\[14.5,' "$SHARE/config/signal_drive_params.yaml" | wc -l | sed 's/^/    route 里 14.5 出现 /'

echo '== 5 五份文件 D 盘 vs install 的 md5 必须相等'
for PAIR in "v2x_demo/parking_shuttle_node.py:$LIB/parking_shuttle_node.py" \
            "v2x_demo/signal_drive_node.py:$LIB/signal_drive_node.py" \
            "config/signal_drive_params.yaml:$SHARE/config/signal_drive_params.yaml" \
            "config/parking_zones.yaml:$SHARE/config/parking_zones.yaml" \
            "config/speed_zones.yaml:$SHARE/config/speed_zones.yaml"; do
  A=$(md5sum < "$D/${PAIR%%:*}" | cut -d' ' -f1)
  B=$(md5sum < "${PAIR#*:}" | cut -d' ' -f1)
  [ "$A" = "$B" ] && echo "  ok $A ${PAIR#*:}" || { echo "  FAIL ${PAIR%%:*}  D=$A install=$B"; RC=1; }
done

[ "$RC" = 0 ] && echo SYNC_BUILD_OK || echo SYNC_BUILD_FAILED
exit $RC
