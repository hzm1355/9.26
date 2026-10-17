#!/bin/bash
# 一次性把"形状门槛上界"这个改动在两类图上的后果打全：
#  A) 闸机抓拍（gate_in / gate_out）—— 放宽能不能让入口也读出车牌；
#  B) 卡口历史抓拍（任务二/三取证用的那批）—— 放宽会不会把原来认对的改成认错的。
# 每张图都跑 5.5（现状）和 9.0（很宽的对照）两遍，只看"合格候选数 / 胜出框 / 拼出 / 均分"。
# set +e：grep 无匹配天然非零，中途退出会让后面的图整批消失。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash

tr -d '\r' < $SIM/tools/diag_capture139.py > /tmp/diag.py

OUT=/tmp/diag_batch139.txt
: > $OUT

for D in /tmp/v2x_parking_in /tmp/v2x_parking_out; do
  for F in $(ls -1t $D/*.png 2>/dev/null | head -3); do
    for R in 5.5 9.0; do
      echo "#### $(basename $F)  上界=$R" >> $OUT
      python3 /tmp/diag.py "$F" /tmp/o$R $R 2>&1 \
        | grep -aE '合格候选数|胜出框|拼出|尺寸=' >> $OUT
    done
  done
done

# 卡口历史抓拍：从每个目录取最近一张，够覆盖任务二/三用过的那几块牌
for D in $(ls -1d /tmp/v2x_gates* 2>/dev/null); do
  for F in $(ls -1t $D/*.png 2>/dev/null | head -1); do
    for R in 5.5 9.0; do
      echo "#### $D/$(basename $F)  上界=$R" >> $OUT
      python3 /tmp/diag.py "$F" /tmp/g$R $R 2>&1 \
        | grep -aE '合格候选数|拼出' >> $OUT
    done
  done
done

cp $OUT /mnt/d/desktop1/9.26.1/_diag139_batch.txt
tail -60 $OUT | cut -c1-190
exit 0
