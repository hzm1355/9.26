#!/usr/bin/env bash
# 只读勘查：car_7 到底还在不在 Ring A 西腿上自发跑（ETC 取证的前置）
source /opt/ros/humble/setup.bash
source $HOME/v2x_ws/install/setup.bash
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim

echo "=== signal_drive_params.yaml 里 car_7 那一段（原样）==="
grep -n -A 14 'car_7' $SIM/src/v2x_demo/config/signal_drive_params.yaml 2>/dev/null | head -40
echo
echo "=== 运行副本（install 里那份）与 D 盘是否一致 ==="
diff -q $SIM/src/v2x_demo/config/signal_drive_params.yaml \
        $HOME/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml 2>&1 | head -2
echo
echo "=== 驱动日志里 car_7 的最近 12 行 ==="
grep -a 'car_7' /tmp/signal_drive_fix301.log | tail -12 | cut -c1-160
echo
echo "=== 轨迹 CSV 里 car_7 最近 6 行 ==="
grep -a ',car_7,' /tmp/signal_drive_path.csv 2>/dev/null | tail -6 | cut -c1-140
echo
echo "=== car_7 位姿采样 12 帧 x 5 s（判它是否在动、在哪条腿上）==="
for I in $(seq 1 12); do
  printf "  %s  %s\n" "$(date +%H:%M:%S)" \
    "$(timeout -k 3 8 python3 $SIM/tools/pose155.py car_7 2>&1 | grep -a 当前位姿 | head -1)"
  sleep 4
done
echo
echo "=== 六车横向偏移点名：谁在 x=-21.80 那条腿 ±0.70 内 ==="
for C in car_1 car_2 car_3 car_4 car_5 car_6 car_7; do
  timeout -k 3 8 python3 $SIM/tools/pose155.py $C 2>&1 | grep -a 当前位姿 \
   | awk -v c=$C '{for(i=1;i<=NF;i++){if($i~/^x=/){sub(/^x=/,"",$i);x=$i+0}if($i~/^y=/){sub(/^y=/,"",$i);y=$i+0}if($i~/^yaw=/){sub(/^yaw=/,"",$i);yw=$i+0}}
     on=(x>=-22.50&&x<=-21.10)?"在西腿带内":"不在";
     printf "  %-7s x=%7.2f y=%7.2f yaw=%+5.2f  %s  y-闸心=%+6.2f\n", c,x,y,yw,on,(y+12.0)}'
done
echo
echo "=== 道闸与台账现量 ==="
timeout -k 3 10 ros2 topic echo /etc/gate/etc_gate_1/state --no-daemon --once 2>&1 | grep -aE 'angle_rad|target_rad|loop_occupied|fault:' | head -5
for f in etc_identify etc_pass etc_actions; do printf "  %s=%s" $f $(wc -l < /tmp/v2x_etc/$f.jsonl 2>/dev/null||echo 0); done; echo
echo "DONE"
