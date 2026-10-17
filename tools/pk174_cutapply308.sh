#!/usr/bin/env bash
# 一次性：把沙盘网格 room_33.obj 里停车场**正南 S1 + 西侧 W1** 两片纯视觉楼体裁掉，
# 并只重启 gzclient 让画面生效。
#
# 为什么只重启 gzclient 就够：room_33 的 collision 早就换成等效盒（台面盒 + bldg_*），
# OBJ 的 470 万面只负责"画"。闸机相机/车牌识别是 **gzserver 进程内**渲染的，
# 客户端重启不动服务端场景，所以物理、ROS 节点、闸机状态、账本、car_3 这一轮全不受影响。
#
# 每一步都有闸，任何一步不合格就退出；换文件之后失败会自动把网格回滚回去。
# 回滚（随时可做）：mv room_33.obj.pre_cut_20261016 room_33.obj 然后重启 gzclient。
set -e

REPO=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
MESH=$REPO/src/smart_car_sim/models/room_33/meshes/room_33.obj
BAK=$MESH.orig_20261016
PRE=$MESH.pre_cut_20261016
CUT=$MESH.cut.obj
LOG=/tmp/pk174_gzclient.log
export DISPLAY=172.26.32.1:1
export GAZEBO_MODEL_PATH=/home/hzm/v2x_models:$REPO/src/smart_car_sim/models:

echo "--- 前置闸"
pgrep -f "gzserver src/smart_car_sim/worlds" >/dev/null \
  || { echo 'FAIL gzserver 不在，改了画面也没人渲染'; exit 2; }
SZ=$(stat -c %s "$MESH")
[ "$SZ" -gt 400000000 ] || { echo "FAIL 原件只有 $SZ 字节，不像 493 MB 的沙盘网格"; exit 2; }
FREE=$(df -B1 --output=avail /mnt/d | tail -1 | tr -d ' ')
[ "$FREE" -gt 2500000000 ] \
  || { echo "FAIL D 盘余量 $FREE 字节，放不下备份+裁剪版两份"; exit 2; }
echo "gzserver 在；原件 $SZ 字节；D 盘余量 $FREE；gzclient pid: $(pgrep -x gzclient | tr '\n' ' ')"

echo "--- 备份原件 -> $BAK"
if [ -f "$BAK" ]; then
  echo "已存在（$(stat -c %s "$BAK") 字节），不覆盖"
else
  cp "$MESH" "$BAK"
fi
[ "$(stat -c %s "$BAK")" = "$SZ" ] || { echo 'FAIL 备份字节数和原件不等'; exit 2; }

echo "--- 生成裁剪版（只裁 S1 正南 + W1 西侧西南角）"
cd "$REPO"
python3 tools/pk173_objcut308.py --write --only=S1,W1

echo "--- 体检：拿裁剪版再扫一遍，待删必须是 0，且顶点行数不变"
python3 tools/pk173_objcut308.py --obj="$CUT" --only=S1,W1 > /tmp/pk174_verify.txt 2>&1
tail -4 /tmp/pk174_verify.txt
grep -Eq "待删 0[ ]" /tmp/pk174_verify.txt \
  || { echo 'FAIL 裁剪版里还留着该删的面，不换文件'; exit 2; }
VCUT=$(grep -c "^v " "$CUT")
VOBJ=$(grep -c "^v " "$MESH")
[ "$VCUT" = "$VOBJ" ] || { echo "FAIL 顶点行数变了 $VOBJ -> $VCUT（索引会错位）"; exit 2; }
echo "顶点行数一致（$VCUT）；裁剪版 $(stat -c %s "$CUT") 字节"

echo "--- 换文件"
mv "$MESH" "$PRE"
mv "$CUT" "$MESH"
echo "现用 $(stat -c %s "$MESH") 字节；回滚件 $PRE 与 $BAK"

echo "--- 只重启 gzclient（先杀光所有实例，再拉一个新的）"
for p in $(pgrep -x gzclient); do kill -TERM "$p" 2>/dev/null || true; done
sleep 3
for p in $(pgrep -x gzclient); do kill -9 "$p" 2>/dev/null || true; done
nohup gzclient --verbose > "$LOG" 2>&1 < /dev/null &
echo "已拉起，等 50 s 让它把 470 MB 网格读进去"
sleep 50
NEW=$(pgrep -x gzclient | tr '\n' ' ')
if [ -z "$NEW" ]; then
  echo 'FAIL gzclient 没起来，日志尾：'
  tail -25 "$LOG"
  echo '自动回滚网格文件'
  mv "$MESH" "$CUT"
  mv "$PRE" "$MESH"
  exit 3
fi
echo "gzclient 新 pid: $NEW"

echo "--- 效果复核：闸机相机再抓一帧（服务端渲染，车牌应当还在）"
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash
python3 tools/pk171_camgrab308.py /parking/gate_in/image_raw 10 /tmp/pk174_gatein.png \
  | tail -1 | tee /tmp/pk174_cam.txt
cp -f /tmp/pk174_gatein.png /mnt/d/desktop1/9.26.1/pk174_gatein.png 2>/dev/null || true
grep -q "FRAMES=0" /tmp/pk174_cam.txt \
  && { echo 'FAIL 相机断流（服务端出问题了），网格已换但画面未复核'; exit 4; } || true
echo 'DONE 网格已裁、gzclient 已重启、相机仍在出帧'
