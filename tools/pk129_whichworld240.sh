#!/bin/bash
# 确认 gzserver 真正加载的是哪一份 world（运行期副本坑：D 盘改完未必是跑的那份）。
set +e
P=$(pgrep -f gzserver | head -1)
echo "gzserver PID=$P"
echo "-- cmdline"
tr '\0' '\n' < /proc/$P/cmdline | grep -aiE 'world|\.world' | head -5
echo "-- cwd"
ls -l /proc/$P/cwd
echo "-- 候选 world 的实体与 md5"
for F in \
  /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world \
  /home/hzm/v2x_ws/src/smart_car_sim/worlds/v2x_city_scene.world \
  /home/hzm/v2x_ws/install/smart_car_sim/share/smart_car_sim/worlds/v2x_city_scene.world \
  /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/worlds/v2x_city.world ; do
  if [ -f "$F" ]; then
    printf '%-100s %s  %s B  car_4=%s\n' "$F" "$(md5sum $F | cut -c1-8)" \
      "$(stat -c %s $F)" "$(grep -aA3 '<model name="car_4"' $F | grep -a -m1 '<pose>' | tr -d ' \r')"
  else
    echo "$F  不存在"
  fi
done
echo "-- GAZEBO_MODEL_PATH / 启动脚本里的 world 路径"
grep -rn 'v2x_city_scene.world' /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/*.sh 2>/dev/null | cut -c1-140 | head -8
echo DONE
