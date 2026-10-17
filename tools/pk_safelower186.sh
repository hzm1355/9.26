#!/bin/bash
# 手动落杆的安全包装：条目 4 的 4b/4c 就是栽在"杆下有人硬落"上——
# 防砸联锁会把目标角抢回 open_rad 1.5708，而杆本来就在 1.5708 时角位移为零，
# 插件的堵转判据 B（1 s 窗口内实测角位移 < 0.05 rad）把它误读成卡死并锁存 FAULT=STUCK，
# 之后整段入口闸都带故障。所以落杆前后都必须体检，且绝不把锁存留在场上。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PKCLI="python3 $SIM/tools/parking_cli.py"
G=${1:-gate_in}
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

echo '  说明：gate list 的表头行 "闸机 2 道：gate_in、gate_out" 里也含 gate_in，'
echo '        所以取行必须锚在条目符号上，否则 grep -m1 命中的是表头，故障位永远读成空。'
L0=$($PKCLI gate list 2>&1 | tr -d '\r' | grep -m1 -E "· *$G ")
echo "  [$G] 落杆前体检：$L0"
$PKCLI gate lower --gate "$G" 2>&1 | tr -d '\r' | head -2
sleep 7
L=$($PKCLI gate list 2>&1 | tr -d '\r' | grep -m1 -E "· *$G ")
echo "  [$G] 落杆后读回：$L"
if printf '%s' "$L" | grep -q 'fault='; then
  if printf '%s' "$L" | grep -qE 'fault=(STUCK|RAISE_FAIL|NO_CLOSE)'; then
    echo '  !! 触发了防砸→重抬→堵转锁存，当场清掉，不留给后面的段落'
    $PKCLI gate clear --gate "$G" 2>&1 | tr -d '\r' | head -1
    sleep 5
    $PKCLI gate lower --gate "$G" 2>&1 | tr -d '\r' | head -1
    sleep 7
    echo "  [$G] 清除后复落：$($PKCLI gate list 2>&1 | tr -d '\r' | grep -m1 -E "· *$G ")"
  fi
fi
$PKCLI gate auto --gate "$G" 2>&1 | tr -d '\r' | head -1
echo "  [$G] 收尾：$($PKCLI gate list 2>&1 | tr -d '\r' | grep -m1 -E "· *$G ")"
echo SAFELOWER_DONE
