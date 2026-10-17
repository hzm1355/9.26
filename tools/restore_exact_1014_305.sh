#!/bin/bash
# ==================================================================================
# 目的：把"代码"钉到盘上**最早可证的真实副本**上，并把现场恢复到能录任务五视频的状态。
#
# 为什么目标只能是 10-14 收工态、而不是 10-13：
#   全盘 + ~/v2x_ws + /tmp 找过 signal_drive_node* / parking_shuttle_node* /
#   parking_manager_node* / *.bak / *.keep，能拿到的最早 .py 副本是
#   /tmp/v2x_demo.pre_onboard_20261015_131512 —— 它是 10-15 12:03 对运行期源码树做的
#   整包快照，内容等于 10-14 收工态。10-13 的 .py 没有任何副本（工程不是 git 仓库，
#   home 侧的 .bak135/.bak136/.bak139 已被 10-15 21:40 那次整目录同步覆盖掉，现在
#   install 里那条 parking_zones.yaml.bak139 符号链接是断的）。所以"退到 10-13 的代码"
#   只有一种诚实做法：凡 10-13 之后**新增**的文件（ETC 那一批）移出工程树；凡 10-13
#   就存在、后来被改过的文件，只能退到有证据的最近一版 = 这份快照。再往前就是我自己
#   编内容，不做。
#
# 本脚本还会修一个 303 那轮暴露的缺口：signal_drive_params.yaml 当时只改了 D 盘、
# home 侧那份没落地（mtime 还停在 10-15 21:40，里面 car_7 仍在）——运行期读的是
# install share 的符号链接 → home src，所以节点其实一直在按带 car_7 的配置跑。
#
# 全程只停/启信号灯链路与 gzclient，不动 gzserver、不动停车场/车牌/测速三件套。
# 反悔路径：D 盘 signal_drive_node.py.pre_exact1014_<ts>.keep（我手改的那一版）。
# ==================================================================================
set -e
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

OUT=/tmp/restore_exact1014_305
mkdir -p "$OUT"
RUN=$OUT/run.log
rm -f "$RUN"
exec >>"$RUN" 2>&1
TS=$(date +%m%d_%H%M%S)

S=/tmp/v2x_demo.pre_onboard_20261015_131512
D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
N=$D/src/v2x_demo
H=/home/hzm/v2x_ws/src/v2x_demo
I=/home/hzm/v2x_ws/install/v2x_demo

md5_of() { md5sum "$1" 2>/dev/null | cut -c1-32; }

echo "===== 0) 前置闸：快照必须是 10-14 收工态，且 parking 两文件本来就没被我动过 ====="
[ -f "$S/v2x_demo/signal_drive_node.py" ] || { echo "!! 快照里没有 signal_drive_node.py，中止"; exit 1; }
echo "  快照行数 $(wc -l < $S/v2x_demo/signal_drive_node.py)（应为 859）｜字节 $(wc -c < $S/v2x_demo/signal_drive_node.py)"
echo "  yaml：快照 md5 $(md5_of "$S/config/signal_drive_params.yaml")"
echo "        D 盘  $(md5_of "$N/config/signal_drive_params.yaml")  —— 两个必须相同"
[ "$(md5_of "$S/config/signal_drive_params.yaml")" = "$(md5_of "$N/config/signal_drive_params.yaml")" ] \
  || { echo "!! D 盘 yaml 与快照不一致，说明 303 那轮的手改没还原到位，中止"; exit 1; }
echo "  parking_manager_node.py  快照 $(md5_of "$S/v2x_demo/parking_manager_node.py") D 盘 $(md5_of "$N/v2x_demo/parking_manager_node.py")"
echo "  parking_shuttle_node.py  快照 $(md5_of "$S/v2x_demo/parking_shuttle_node.py") D 盘 $(md5_of "$N/v2x_demo/parking_shuttle_node.py")"
echo "  D 盘 yaml 里 car_7 字样 $(grep -c 'car_7' "$N/config/signal_drive_params.yaml" || true) 行（应为 0）"

echo "===== 1) 用快照精确覆盖驱动节点（替掉我 303 那轮手改的重排），旧版先存档 ====="
cp -a "$N/v2x_demo/signal_drive_node.py" "$N/v2x_demo/signal_drive_node.py.pre_exact1014_$TS.keep"
cp -a "$S/v2x_demo/signal_drive_node.py" "$N/v2x_demo/signal_drive_node.py"
echo "  覆盖后 D 盘 md5 $(md5_of "$N/v2x_demo/signal_drive_node.py")（应等于快照）"
echo "  归档：v2x_demo/signal_drive_node.py.pre_exact1014_$TS.keep"

echo "===== 2) 同步 D 盘真值到 home src（运行期实际读这一份）+ 三方 md5 对账 ====="
cp "$N/v2x_demo/signal_drive_node.py" "$H/v2x_demo/signal_drive_node.py"
cp "$N/config/signal_drive_params.yaml" "$H/config/signal_drive_params.yaml"
for F in v2x_demo/signal_drive_node.py config/signal_drive_params.yaml; do
  L=$(readlink -f "$I/$F" 2>/dev/null || echo "无此链接")
  echo "  $F"
  echo "    D 盘      $(md5_of "$N/$F")"
  echo "    home src  $(md5_of "$H/$F")"
  echo "    install 链接指向 $L"
done
echo "  home 侧 yaml 里 car_7 字样 $(grep -c 'car_7' "$H/config/signal_drive_params.yaml" || true) 行（应为 0）"
python3 -m py_compile "$H/v2x_demo/signal_drive_node.py" && echo "  py_compile 通过"
echo "  锚点：10-14 原写法「不是这条线的来车方向」$(grep -c '不是这条线的来车方向' "$H/v2x_demo/signal_drive_node.py" || true) 处（应为 1）"
echo "        我手改写法「车头不朝这条线」残留 $(grep -c '车头不朝这条线' "$H/v2x_demo/signal_drive_node.py" || true) 处（应为 0）"

echo "===== 3) 只重启信号灯链路（start_signal.sh 内部自带 stop_signal + place_cars 摆位）====="
bash "$D/tools/start_signal.sh" > "$OUT/start_signal.log" 2>&1 || true
sleep 20
echo "  start_signal 尾部 8 行原样："
tail -8 "$OUT/start_signal.log" | sed 's|^|    |'

echo "===== 4) 重开 gzclient（303 那轮唯一没过的一项：GUI 不在）====="
bash "$D/tools/open_gui77.sh" > "$OUT/gui.log" 2>&1 || true
sleep 3
GZC=$(pgrep -x gzclient | head -1 || true)
echo "  gzclient pid=${GZC:-无}｜open_gui77 尾部："
tail -6 "$OUT/gui.log" | sed 's|^|    |'

echo "===== 5) 就绪闸①：运行期实体点名（看效果，不看文件）====="
for M in car_1 car_2 car_3 car_4 car_5 car_6 parking_gate_in; do
  P=$(timeout 12 gz model -m "$M" -p 2>&1 | tr '\n' ' ')
  case "$P" in
    *Unable*) echo "  $M：absent（不符合预期！应存在）" ;;
    *) echo "  $M：present（符合预期）｜$(echo "$P" | cut -c1-46)" ;;
  esac
done
for M in car_7 etc_gate_1 etc_rsu_1 etc_gantry_g1 etc_markings_lane_a; do
  P=$(timeout 12 gz model -m "$M" -p 2>&1 | tr '\n' ' ')
  case "$P" in
    *Unable*) echo "  $M：absent（符合预期）" ;;
    *) echo "  $M：present（不符合预期！回退没生效）" ;;
  esac
done

echo "===== 6) 就绪闸②：各路实例数（应各 1，etc_* 应 0）====="
for PAT in 'signal[_]drive_node.py' 'traffic[_]light[_]manager' 'adaptive[_]timing_node' \
           'traffic[_]light[_]recorder' 'plate[_]gen[_]node' 'plate[_]recognizer[_]node' \
           'plate[_]recorder[_]node' 'speed[_]detect[_]node' 'speed[_]recorder[_]node' \
           'parking[_]manager[_]node' 'parking[_]recorder[_]node' 'parking[_]shuttle[_]node' \
           'websocket[_]bridge'; do
  echo "  $(printf '%-26s' "$PAT") $(pgrep -cf "lib/v2x[_]demo/$PAT" || true)"
done
echo "  etc_* 真实例 $(pgrep -cf "lib/v2x[_]demo/etc[_]" || true)（应 0）"
echo "  gzserver=$(pgrep -x gzserver | head -1)  gzclient=${GZC:-无}"

echo "===== 7) 就绪闸③：车队在动的辆数（瞬移不计入）====="
python3 "$D/tools/fleet_moving_guard303.py" /tmp/signal_drive_path.csv 30

echo "===== 8) 就绪闸④：出界/离道/卡死 计数 + 日志尾 ====="
echo "  signal.log：出界 $(grep -c '出界' /tmp/signal.log || true)｜离道 $(grep -c '离道' /tmp/signal.log || true)｜卡死 $(grep -c '卡死' /tmp/signal.log || true)"
tail -8 /tmp/signal.log | sed 's|^|    |'

echo "===== 9) 就绪闸⑤：停车场链路在产出（任务五条目 1~5 都靠它）====="
echo "  90 s 内被写过的停车场文件 $(find /tmp/v2x_parking -maxdepth 1 -type f -mmin -1.5 2>/dev/null | wc -l) 个（>0 才算在走）"
echo "  看板与回显：:8080 -> $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/ || true)｜:8090 -> $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8090/ || true)"

echo "===== 10) 结论 ====="
BAD=0
[ -n "$GZC" ] || { echo "  !! gzclient 不在"; BAD=1; }
[ "$(pgrep -cf "lib/v2x[_]demo/signal[_]drive_node.py" || true)" = "1" ] || { echo "  !! 驱动节点实例数不是 1"; BAD=1; }
[ "$(pgrep -cf "lib/v2x[_]demo/etc[_]" || true)" = "0" ] || { echo "  !! 还有 ETC 节点活着"; BAD=1; }
[ "$(md5_of "$N/v2x_demo/signal_drive_node.py")" = "$(md5_of "$S/v2x_demo/signal_drive_node.py")" ] \
  || { echo "  !! 驱动节点与快照不一致"; BAD=1; }
if [ "$BAD" = "0" ]; then
  echo "  READY_FOR_VIDEO_STAGE（代码=10-14 收工态逐字节一致；10-13 的 .py 无副本，见头部说明）"
else
  echo "  !! NOT_READY：上面有未过项"
fi
echo "证据留在 $OUT（run.log / start_signal.log / gui.log）；反悔用 $N/v2x_demo/signal_drive_node.py.pre_exact1014_$TS.keep"
echo "REX305DONE"
