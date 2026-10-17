#!/bin/bash
# 运行期加载的 world 到底是哪一份（gzserver 是用相对路径起的），以及那份里 car_6 挂了什么。
set +e
PID=$(pgrep -x gzserver | head -1)
echo "gzserver PID=$PID"
echo "== cwd =="
readlink /proc/$PID/cwd
echo "== cmdline =="
tr '\0' ' ' < /proc/$PID/cmdline; echo
CWD=$(readlink /proc/$PID/cwd)
W="$CWD/src/smart_car_sim/worlds/v2x_city_scene.world"
echo "== 这份 world 是否存在：$W =="
ls -l "$W" 2>&1
echo "== 与 D 盘那份的差异 =="
diff -q "$W" /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world && echo SAME
echo "== car_6 的 plugin 块原文 =="
awk '/<model name="car_6">/,/<\/model>/' "$W" 2>/dev/null | grep -n 'plugin\|odometrySource\|legacyMode\|updateRate\|update_rate\|cmdTimeout\|cmd_vel_timeout\|topicName' | head -20
echo "== 全文里所有可能施力/施矩的插件 =="
grep -n 'planar_move\|apply_joint\|ROSPlanarMove\|libgazebo_ros_joint\|force\|torque' "$W" 2>/dev/null | grep -av '<friction>\|max_wheel_torque' | head -20
echo DONE
