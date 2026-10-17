#!/usr/bin/env bash
# 只读勘查：任务六 ETC 现场状态（进程/节点实例/台账/最近日志尾）
echo "=== 进程 ==="
echo "gzserver=$(pgrep -x gzserver | head -1) gzclient=$(pgrep -x gzclient | head -1)"
echo "=== 节点真实例 ==="
for N in signal_drive_node traffic_light_manager adaptive_timing_node traffic_light_recorder \
         etc_rsu_node etc_manager_node etc_recorder_node \
         plate_gen_node plate_recognizer_node plate_recorder_node \
         parking_shuttle_node parking_manager_node speed_detect_node; do
  C=$(pgrep -cf "lib/v2x[_]demo/${N}" 2>/dev/null); [ -z "$C" ] && C=0
  printf "  %-26s %s\n" "$N" "$C"
done
echo "=== ETC 台账 /tmp/v2x_etc ==="
ls -l /tmp/v2x_etc/ 2>/dev/null || echo "  目录不存在"
for f in /tmp/v2x_etc/*.jsonl; do
  [ -f "$f" ] && printf "  %-34s %5s 行  mtime=%s\n" "$(basename $f)" "$(wc -l < $f)" "$(stat -c %y $f | cut -c1-19)"
done
echo "=== 最近 3 条 etc_pass ==="
tail -3 /tmp/v2x_etc/etc_pass.jsonl 2>/dev/null | cut -c1-300 || echo "  无"
echo "=== 最近 3 条 etc_identify ==="
tail -3 /tmp/v2x_etc/etc_identify.jsonl 2>/dev/null | cut -c1-300 || echo "  无"
echo "=== 301/302/303 产物 ==="
for d in /tmp/etc_run301 /tmp/etc_revert302 /tmp/fleet_guard303 /tmp/video_stage303; do
  echo "  -- $d"
  ls -1 $d 2>/dev/null | sed 's/^/     /' || echo "     不存在"
done
echo "  /tmp 下 30x 相关目录："
ls -1dt /tmp/*30[0-9]* /tmp/*29[0-9]* 2>/dev/null | head -15 | sed 's/^/     /'
echo "=== 各节点日志尾 ==="
for L in /tmp/etc_rsu*.log /tmp/etc_manager*.log /tmp/etc_recorder*.log /tmp/signal_drive*.log; do
  [ -f "$L" ] && { echo "  --- $L (mtime $(stat -c %y $L | cut -c1-16)) ---"; tail -5 "$L" | cut -c1-200 | sed 's/^/      /'; }
done
echo "=== world 是否已插 ETC 块 ==="
W=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/worlds/v2x_city_scene.world
grep -ac 'GENERATED: etc' $W 2>/dev/null | sed 's/^/  etc 块标记数=/'
md5sum $W 2>/dev/null | sed 's/^/  md5=/'
echo "DONE"
