#!/usr/bin/env bash
# #113 钉死"运行期加载的是哪份 world/车辆定义"：文件在不在、多大、有几辆车、mtime 与 gzserver 启动时刻的先后。
set +e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$D/src/smart_car_sim/worlds/v2x_city_scene.world
echo "=== 1) world 文件本体 ==="
ls -la "$D/src/smart_car_sim/worlds/" | head -30
echo "--- stat ---"
stat "$W" 2>&1 | head -12
echo "--- 结构计数 ---"
echo "行数: $(wc -l < "$W" 2>/dev/null)"
echo "字节: $(wc -c < "$W" 2>/dev/null)"
echo "model name 出现: $(grep -c 'model name' "$W" 2>/dev/null)"
echo "include 出现: $(grep -c 'include' "$W" 2>/dev/null)"
echo "car_ 出现: $(grep -c 'car_' "$W" 2>/dev/null)"
echo "uri 出现: $(grep -c '<uri>' "$W" 2>/dev/null)"
echo "=== 2) 车辆定义的真源 ==="
ls -la "$D/src/smart_car_sim/config/" 2>&1 | head -20
grep -n 'car_6' "$D/src/smart_car_sim/config/vehicle_preset.yaml" 2>/dev/null | head -10
echo "=== 3) gzserver 启动时刻 vs world mtime ==="
for p in $(pgrep -x gzserver); do
  ps -o lstart=,etime=,pid= -p $p
  echo "  cmdline: $(tr '\0' ' ' < /proc/$p/cmdline)"
done
stat -c 'world mtime: %y' "$W" 2>&1
echo "=== 4) 运行态 car_6 的插件来源 ==="
grep -n 'filename=.*diff\|filename=.*drive' "$W" 2>/dev/null | head -10
echo "--- 安装树里存在的 diff 插件 so ---"
find "$D/install" -name '*diff*so' -o -name '*drive*so' 2>/dev/null | head -10
echo "--- gzserver 进程实际加载的插件 so（含 diff/drive 字样） ---"
for p in $(pgrep -x gzserver); do
  grep -ao '/[^ ]*\(diff\|drive\)[^ ]*\.so' /proc/$p/maps 2>/dev/null | sort -u | head -10
done
exit 0
