#!/bin/bash
# 163：用人工抬杆把 car_3 从入口核验位放过去，并在车离场后落杆、交还自动。
#
# 为什么必须人工这一脚：入口抓拍插件的重新武装判据是"车到 focus 的水平距离 > far 2.5"
# （v2x_gate_plugin.cc: `armed && d > far_` → armed=false）。而入口车道里最远的一格
# 是等待位 (12.00,-12.80)，离 focus (12.00,-11.21) 只有 1.59 m —— 车在这一段怎么退
# 都够不到 2.5 m，于是 armed 永远留在 true，这道闸再也不会发第二张抓拍。
# 本轮的第一张抓拍（gate_in-2）拍在重 spawn 脚本自己踩油门的那一拍上，那时牌还没重贴，
# 画面是一块空灰矩形 → 识别无结果 → manager 判 PENDING 等人工。manager 侧其实已经
# 放开（reject() 里 g.cycle=None），锁死的是插件的武装位，只有"车真的过杆开进停车场"
# 才能把它解掉：进点 (14.915,-9.55) 离 focus 3.35 m > 2.5，第二轮就是全自动链路。
#
# 落杆不能提前：地感框实测 x 10.15..13.85 / y -12.35..-9.15，车停在进点 (12.00,-9.55)
# 时车心还在框内，此时下发 0 会被防砸联锁挡回（那是条目 3 的砸车保护，不是 bug）。
# 所以轮询等 loop_occupied=False（车已拐到东列 x>13.85）再落。
#
# 用法：bash tools/pk163_manualin308.sh
set +e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools || exit 1
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash
M=car_3

peek() {  # 两道闸的快照（srv 回包，含 state/angle/loop）
  timeout 40 python3 parking_cli.py gate list 2>&1 | tr -d '\r' | sed 's/^/      /'
}

echo "--- 变更前：$M 位姿 + 闸机快照"
timeout 8 gz model -m $M -p 2>&1 | tr -d '\r' | head -1 | sed 's/^/      /'
peek

echo "--- 下发人工抬杆（gate raise，angle 走哨兵 -999 = YAML 的 open_angle）"
timeout 60 python3 parking_cli.py gate raise --gate gate_in 2>&1 | tr -d '\r' | sed 's/^/      /'

echo "--- 轮询：等车过杆并离场（loop_occupied 变 False），最多 25 拍 × 6 s"
LOWED=0
for i in $(seq 1 25); do
  sleep 6
  T=$(date +%H:%M:%S)
  P=$(timeout 8 gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
  echo "[$T] $M 位姿=$P"
  tail -1 /tmp/shuttle_$M.log 2>/dev/null | cut -c1-150 | sed 's/^/      shuttle| /'
  OUT=$(timeout 40 python3 parking_cli.py gate list 2>&1 | tr -d '\r')
  echo "$OUT" | sed 's/^/      /'
  if echo "$OUT" | grep -q 'gate_in.*空闲'; then
    echo "--- 入口地感已释放 → 下发人工落杆，再交还自动"
    timeout 60 python3 parking_cli.py gate lower --gate gate_in 2>&1 | tr -d '\r' | sed 's/^/      /'
    timeout 60 python3 parking_cli.py gate auto --gate gate_in 2>&1 | tr -d '\r' | sed 's/^/      /'
    LOWED=1
    break
  fi
done

echo "--- 收尾复核：闸机快照 + 账本 + 日志尾"
peek
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl 2>&1 \
  | tr -d '\r' | sed 's/^/      /'
echo "      mgr  | $(tail -3 /tmp/parking_manager_node.log 2>&1 | tr -d '\r' | cut -c1-150 | tr '\n' '|')"
echo "      shut | $(tail -3 /tmp/shuttle_$M.log 2>&1 | tr -d '\r' | cut -c1-150 | tr '\n' '|')"
if [ $LOWED -eq 0 ]; then
  echo "MANUALIN_NOTLOWED $M —— 车还没走到地感释放那一格，别按落杆，交人工判"
  exit 2
fi
echo MANUALIN_OK $M
exit 0
