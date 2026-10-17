#!/usr/bin/env bash
# 按用户 10-16 点名的范围执行："全回退到 10-14 出视频那一版"。
#
# 10-14 是任务五五段取证视频的产出日（10:11/10:19/10:31/13:27/13:40）。10-15 对**共用行为**
# 的改动只有 4 处，本脚本把它们全部退回；其余 10-15 产物（任务六 ETC 的节点源码、
# etc_zones.yaml、诊断脚本）原样留在盘上只是不再上场：
#   ① worlds/v2x_city_scene.world：10-15 12:03 注入的 etc 生成块 + car_7 块 → 用现成备份
#      v2x_city_scene.world.pre_deploy_1015_120330.bak 覆盖。已核对：除这两块外一字节不差，
#      car_6／parking_stripes／灯杆几何都在 3082 行之前，没被动过。
#   ② signal_drive_node.py：朝向门 + 航点超时日志两处 → 已用编辑工具改回原文（这里做编译闸与锚点复核）。
#   ③ signal_drive_params.yaml：car_7 上 Ring A 那条分队 → 已删（复核解析出来的车只有 5 辆在驱）。
#   ④ tools/stop_signal.sh：零速名单里的 car_7 → 已删。
#      **一处偏离要明说**：stop_signal.sh 的"逐具名节点 pkill"结构留着没退回 10-14 的整片包目录式，
#      因为整片式会连带杀掉车牌/测速/停车场节点（README §七.3 记过那次"DONE 但实例 0"的事故），
#      退回它只会让恢复现场更难，不属于"出视频那一版"的必要条件。
#
# ETC 三节点只停不删；台账整目录改名归档，免得新现场读到一个已经不存在的 etc_gate_1。
# 收尾六道就绪闸：运行期实体点名（car_7/etc_* 必须不在）→ 实例数 → 车队在动的辆数 →
# 复位风暴计数 → 停车场/卡口链路在产出 → :8080/:8090 应答 + gzclient 存活。
set +e
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
W=$D/src/smart_car_sim/worlds/v2x_city_scene.world
BK=$W.pre_deploy_1015_120330.bak
SRC=$HOME/v2x_ws/src/v2x_demo
TS=$(date +%m%d_%H%M%S)
OUT=/tmp/restore_video_stage303
LOG=$OUT/run.log
CSV=/tmp/signal_drive_path.csv
mkdir -p "$OUT"; : > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source ~/v2x_ws/install/setup.bash
zero() { for c in car_1 car_2 car_3 car_4 car_5 car_6 car_7; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1; done; }

say "===== 0) 前置闸：基线备份必须存在，且它里面确实没有 etc/car_7 ====="
if [ ! -f "$BK" ]; then say "  !! 基线 $BK 不存在，整脚本停（未改动任何文件）"; exit 1; fi
say "  备份行数 $(wc -l < "$BK")｜当前行数 $(wc -l < "$W")"
say "  备份里 etc_ 块 $(grep -ac 'name="etc_' "$BK")｜car_7 块 $(grep -ac 'name="car_7' "$BK")（都应为 0）"
say "  当前里 etc_ 块 $(grep -ac 'name="etc_' "$W")｜car_7 块 $(grep -ac 'name="car_7' "$W")（应 >0，正是要去掉的）"
cmp -s "$BK" "$W.pre_etc_20261015_120340.bak" && say "  两份 10-15 12:03 备份内容一致" || say "  两份 10-15 12:03 备份内容有差（本脚本用 pre_deploy 那份）"

say "===== 1) 先把现在这一版存档（保证可反悔），再用备份覆盖 world ====="
cp -a "$W" "$W.pre_restore1014_${TS}.bak"
for F in src/v2x_demo/v2x_demo/signal_drive_node.py src/v2x_demo/config/signal_drive_params.yaml tools/stop_signal.sh; do
  cp -a "$D/$F" "$D/$F.reverted1014_${TS}.keep"
done
cp -a "$BK" "$W"
say "  world 现行数 $(wc -l < "$W")｜mtime $(stat -c %y "$W" | cut -c1-19)"
say "  归档：$(basename "$W").pre_restore1014_${TS}.bak 与 *.reverted1014_${TS}.keep"

say "===== 2) 同步 D 盘真值到 ~/v2x_ws/src（运行期读这一份）+ 编译闸与锚点复核 ====="
tr -d '\r' < "$D/src/v2x_demo/v2x_demo/signal_drive_node.py" > "$SRC/v2x_demo/signal_drive_node.py"
tr -d '\r' < "$D/src/v2x_demo/config/signal_drive_params.yaml" > "$SRC/v2x_demo/config/signal_drive_params.yaml"
say "  已同步 signal_drive_node.py 与 signal_drive_params.yaml 两份"
python3 -m py_compile "$SRC/v2x_demo/signal_drive_node.py" \
  && say "  py_compile 通过" || { say "  !! 编译不过，停（world 已覆盖，用 .pre_restore1014_${TS}.bak 可原样退回）"; exit 1; }
say "  朝向门：新写法残留 $(grep -ac '车头歪' "$SRC/v2x_demo/signal_drive_node.py")（应 0）｜退回写法 $(grep -ac '不朝这条线' "$SRC/v2x_demo/signal_drive_node.py")（应 1）"
say "  超时日志：条件式守卫残留 $(grep -ac 'if not ag.stopped:' "$SRC/v2x_demo/signal_drive_node.py")（应 0）"
say "  出界阈值原样：$(grep -a "declare_parameter('oob_x'" "$SRC/v2x_demo/signal_drive_node.py" | tr -d ' ')"
python3 -c 'import yaml,sys
c=yaml.safe_load(open(sys.argv[1],encoding="utf-8"))["signal_drive"]["cars"]
drv=sorted(k for k,v in c.items() if v.get("drive",True))
print("  yaml 解析 OK，全部车型：%s" % " ".join(sorted(c)))
print("  进 signal_drive 实际驱动的：%s（car_7 应不在其中）" % " ".join(drv))' \
  "$SRC/v2x_demo/config/signal_drive_params.yaml" 2>&1 | tee -a "$LOG"
say "  yaml 里 car_7 字样剩 $(grep -ac 'car_7' "$SRC/v2x_demo/config/signal_drive_params.yaml") 行（应为 0）"

say "===== 3) 停掉任务六 ETC 三节点（只停不删），先补零速，台账整目录归档 ====="
zero
for PID in $(pgrep -f 'lib/v2x[_]demo/etc[_]' ); do say "  杀 etc 节点 pid $PID"; kill -9 "$PID" 2>/dev/null; done
for PID in $(pgrep -f 'run v2x[_]demo etc[_]'); do kill -9 "$PID" 2>/dev/null; done
sleep 2
say "  残留 etc 真实例 $(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)（应 0）"
if [ -d /tmp/v2x_etc ]; then mv /tmp/v2x_etc /tmp/v2x_etc.archive_${TS}; say "  台账归档 -> /tmp/v2x_etc.archive_${TS}"; else say "  /tmp/v2x_etc 不存在"; fi

say "===== 4) 整栈重启到一个干净物理态（README §七.3：脏物理态只有整栈重启能断根）====="
T0=$(date +%s)
bash $D/tools/restart_clean_stack.sh > "$OUT/restart.log" 2>&1
RC=$?
say "  restart_clean_stack 退出码 $RC，用时 $(( $(date +%s) - T0 )) s，原样尾部 25 行："
tail -25 "$OUT/restart.log" | cut -c1-170 | sed 's/^/    /' | tee -a "$LOG"

say "===== 5) 就绪闸①：运行期实体点名（回退是否生效要看效果，不看文件）====="
for M in car_1 car_2 car_3 car_4 car_5 car_6 parking_gate_in car_7 etc_gate_1 etc_rsu_1 etc_gantry_g1; do
  case "$M" in
    car_7|etc_*) WANT=absent ;;
    *)           WANT=present ;;
  esac
  R=$(timeout -k 3 15 gz model -m $M -p 2>&1 | tr -d '\r' | head -1)
  if echo "$R" | grep -aq 'Unable'; then
    GOT=absent
  else
    GOT=present
  fi
  if [ "$GOT" = "$WANT" ]; then
    say "  $M：$GOT（符合预期）$([ "$GOT" = present ] && echo "｜$(echo $R | cut -c1-40)")"
  else
    say "  $M：$GOT（!! 预期 $WANT）"
  fi
done

say "===== 6) 就绪闸②：各路实例数（各应为 1，etc_* 应为 0）====="
for N in signal_drive_node traffic_light_manager adaptive_timing_node traffic_light_recorder \
         plate_gen_node plate_recognizer_node plate_recorder_node speed_detect_node \
         speed_recorder_node parking_manager_node parking_recorder_node parking_shuttle_node; do
  printf '  %-24s %s\n' "$N" "$(pgrep -cf "lib/v2x[_]demo/${N}" || true)" | tee -a "$LOG"
done
say "  etc_* 三节点 $(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)（应 0）"
say "  gzserver=$(pgrep -x gzserver | head -1)  gzclient=$(pgrep -x gzclient | head -1)"

say "===== 7) 就绪闸③：车队在动的辆数（口径见 fleet_moving_guard303.py，瞬移不计入）====="
sleep 25
python3 $D/tools/fleet_moving_guard303.py "$CSV" 30 2>&1 | tee -a "$LOG"

say "===== 8) 就绪闸④：复位风暴计数（改前基线：301 那轮 251 行日志里 8 条出界）====="
for L in /tmp/signal.log "$OUT/restart.log"; do
  [ -f "$L" ] || continue
  say "  $(basename "$L")：出界 $(grep -ac '出界' "$L")｜离道 $(grep -ac '离道' "$L")｜卡死 $(grep -ac '顶住障碍卡死' "$L")"
done
say "  /tmp/signal.log 尾部 12 行原样："
tail -12 /tmp/signal.log 2>/dev/null | cut -c1-170 | sed 's/^/    /' | tee -a "$LOG"

say "===== 9) 就绪闸⑤：停车场与卡口链路在产出（任务五条目5 依赖这两条）====="
say "  停车场台账：$(ls -1 /tmp/v2x_parking 2>/dev/null | tr '\n' ' ')"
say "  90 s 内被写过的停车场文件 $(find /tmp/v2x_parking -type f -newermt '-90 seconds' 2>/dev/null | wc -l) 个（>0 才算链路在走）"
say "  卡口实拍帧最新三张：$(ls -1t /tmp/v2x_gates 2>/dev/null | head -3 | tr '\n' ' ')"
say "  180 s 内新增帧 $(find /tmp/v2x_gates -type f -newermt '-180 seconds' 2>/dev/null | wc -l) 张（车未到卡口时可以是 0，不当故障）"
say "  car_6 位姿：$(timeout -k 3 20 python3 $D/tools/pose155.py car_6 2>&1 | grep -a 当前位姿 | head -1)"

say "===== 10) 就绪闸⑥：回显 :8080 与看板 :8090 应答 ====="
for PT in 8080 8090; do
  C=$(timeout -k 2 8 curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:$PT/ 2>/dev/null)
  say "  :$PT -> ${C:-无应答}"
done

say "===== 11) 结论 ====="
BAD=0
[ "$(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)" = "0" ] || { say "  !! ETC 节点还在上场"; BAD=1; }
timeout -k 3 15 gz model -m car_7 -p 2>&1 | grep -aq 'Unable' || { say "  !! 运行期里 car_7 还在"; BAD=1; }
timeout -k 3 15 gz model -m etc_gate_1 -p 2>&1 | grep -aq 'Unable' || { say "  !! 运行期里 etc_gate_1 还在"; BAD=1; }
[ "$(pgrep -cf 'lib/v2x[_]demo/signal_drive_node' || true)" = "1" ] || { say "  !! 驱动节点实例数不是 1"; BAD=1; }
[ -n "$(pgrep -x gzclient || true)" ] || { say "  !! gzclient 不在（GUI 没起来）"; BAD=1; }
N=$(python3 $D/tools/fleet_moving_guard303.py "$CSV" 30 2>/dev/null | grep -ao 'MOVING=[0-9]*' | cut -d= -f2)
[ "${N:-0}" -ge 3 ] || { say "  !! 车队在动的辆数 ${N:-0} < 3，物理态仍不可录"; BAD=1; }
if [ "$BAD" = 0 ]; then
  say "READY_FOR_VIDEO_STAGE：已回到 10-14 出视频那一版的现场（无 car_7/ETC，四路链路齐，车队在动，GUI 在）"
else
  say "!! NOT_READY：上面有未过项，先处理再谈录制"
fi
say "全部证据留在 $OUT（run.log / restart.log）；反悔用 $W.pre_restore1014_${TS}.bak 与 *.reverted1014_${TS}.keep"
say 'RVS303DONE'
