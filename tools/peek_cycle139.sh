#!/bin/bash
# 等一轮摆渡车周期，然后把三方日志与抓拍文件对齐打出来（取证用的"一轮体检"）。
# 为什么等：出场抓拍发生在车穿过 1.4 m 触发圈那一拍，从 shuttle 起步到那儿要 ~60 s，
# 中间还夹着 12 s 车位停留；提前读只会读到"什么都没发生"，误判成节点没动。
# set +e：grep 无匹配天然非零，中途退出会让后面的段整片消失（memory 里踩过）。
set +e
WAIT=${1:-130}
sleep "$WAIT"

echo "=== shuttle（本轮作废计数 / 越杆 / 退回）==="
grep -aE 'shuttle\]' /tmp/parking_shuttle_node.log | tail -12 | cut -c1-200

echo "=== manager（到场 / 抬落杆 / 判定）==="
grep -aE 'gate_out|gate_in' /tmp/parking_manager_node.log | tail -14 | cut -c1-200

echo "=== recognizer（最近 8 条识别结论）==="
# 两种落盘名都要认：ensure_recognizer.sh / restart_clean_stack.sh 写 /tmp/plate_rec.log，
# start_parking_nodes.sh 按 "${NODE%.py}.log" 规则写 /tmp/plate_recognizer_node.log。
# 只匹配 *recogn* 会在前者在场时报"找不到"，把识别结论整段读成空。
R=$(ls -1t /tmp/plate_rec.log /tmp/*recogn*.log 2>/dev/null | head -1)
echo "  日志文件：${R:-找不到 /tmp/*recogn*.log}"
[ -n "$R" ] && tail -8 "$R" | cut -c1-200

echo "=== 抓拍文件（按时间，含大小）==="
for D in /tmp/v2x_parking_in /tmp/v2x_parking_out; do
  echo "  ## $D"
  ls -1t --time-style=+%H:%M:%S -l "$D" 2>/dev/null | head -4
done

echo "=== car_6 位姿（z 低于 6.5 = 掉出台面）==="
gz model -m car_6 -p 2>&1 | head -2
exit 0
