#!/bin/bash
# ==================================================================================
# 任务五（停车场闸机）+ 任务六（ETC）代码隔离。用户点名的范围与方式（10-16）：
#   范围：只代码 + 配置，**world 里的停车场几何、道闸、car_6 一概保留**（不动场景）。
#   方式：**移动到隔离归档目录**，不 rm —— 工程不是 git 仓库，物理删除不可逆。
#
# 两档模式：
#   bash 本脚本.sh list    —— 只清点 + 做归档 tar + 校验，不碰工程树任何文件。
#   bash 本脚本.sh apply   —— 先停节点（补零速）、再移动、再改 setup.py/launch 引用、
#                              再 colcon build 硬闸，最后过就绪闸。
#
# 为什么要"改引用"这一步：entry_points / launch 里只要还写着被移走的脚本，
# colcon build 或 ros2 launch 会直接报错，等于把任务一~四的链路一起弄坏。
# 隔离清单由通配符**推导**（不是我凭记忆列的），漏列就是 apply 后 build 报缺件，
# 多列（误伤任务一~四的文件）由"引用反查 + build 闸 + 车队在动闸"三层兜住。
# ==================================================================================
set -e
MODE=${1:-list}
OUT=/tmp/quarantine_parketc_307
mkdir -p "$OUT"
RUN=$OUT/${MODE}_$(date +%m%d_%H%M%S).log
exec >>"$RUN" 2>&1
echo "模式=$MODE  日志=$RUN"

D=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
PKG=$D/src/v2x_demo
TOOLS=$D/tools
ARCH=$D/存档_弃用任务五六_1016
mkdir -p "$ARCH"

# ---- 待隔离文件推导：由三个规则化推导器生成，A/B/C 三层，逐条可复核 ----
#   A 层 文件名直接命中 parking_/etc_/barrier；
#   B 层 内容只服务任务五/六（命中 56 特征、不命中任务一~四特征）；
#   C 层 留守脚本里**真调用**了 A/B 且自己主业也是任务五/六（ratio56>=0.5、不在 LIVE 白名单）。
# 为什么不让 shell 通配符来列：上一版通配符只认 "*parking*/*etc*"，漏掉了
# _diag_barrier_*.sh、test_barrier_*.py 这类名字里没有 parking 的文件，
# 也完全没有覆盖 180 个"名字无关、内容只服务这两个任务"的脚本。
NODE_SRC=$PKG/v2x_demo
CLEAN='pre_|archive|quarantine|\.(keep|bak|orig)$'
python3 "$TOOLS/quarantine_manifest_307.py" > "$OUT/tier_ab.log" 2>&1
# 顺序是硬要求：tier_c 要拿"A+B 已定"的清单去反查谁调用了它们，所以先把 A+B 合成
# MANIFEST.txt 给它，跑完再把 C 并回清单。两步合并成一步会让 tier_c 读到空清单、
# 于是 C 层恒为 0 —— 那就是"看起来跑了，其实什么都没查"。
cat "$OUT/MANIFEST_A.txt" "$OUT/MANIFEST_B.txt" | grep -v -E "$CLEAN" | sort -u > "$OUT/MANIFEST.txt"
python3 "$TOOLS/quarantine_tier_c_307.py"   > "$OUT/tier_c.log"  2>&1
cat "$OUT/MANIFEST_A.txt" "$OUT/MANIFEST_B.txt" "$OUT/MANIFEST_C.txt" 2>/dev/null \
  | grep -v -E "$CLEAN" | sort -u > "$OUT/MANIFEST.txt"
NTOT=$(wc -l < "$OUT/MANIFEST.txt")
NA=$(wc -l < "$OUT/MANIFEST_A.txt"); NB=$(wc -l < "$OUT/MANIFEST_B.txt"); NC=$(wc -l < "$OUT/MANIFEST_C.txt")
sed 's|$|:|' "$OUT/MANIFEST.txt" > "$OUT/_manchr.txt"
echo "  分层：A=$NA B=$NB C=$NC → 去重后 $NTOT 项"
echo "  无内容信号、默认留下的脚本 $(wc -l < "$OUT/MANIFEST_NOSIG.txt" 2>/dev/null || echo 0) 个：$(sed 's|.*/||' "$OUT/MANIFEST_NOSIG.txt" 2>/dev/null | tr '\n' ' ')"

echo "===== 1) 拟隔离清单（$NTOT 项，逐条字节数）====="
while IFS= read -r F; do printf '  %9s  %s\n' "$(wc -c < "$F")" "${F#$D/}"; done < "$OUT/MANIFEST.txt"

echo "===== 2) 边界体检：清单里绝不能有任务一~四的文件 ====="
# 只按**文件名**判，不按全路径 —— 上次用全路径会被目录名 v2x_traffic_sim 里的
# "traffic" 误报成"清单里有任务二的文件"，那条闸等于没在检查任何东西。
echo "  文件名同时含 traffic_light/plate/speed/signal_drive/websocket 的（应为空）："
while IFS= read -r F; do
  case "$(basename "$F")" in
    *traffic_light*|*plate*|*speed*|*signal_drive*|*websocket*) echo "    误伤？ $F" ;;
  esac
done < "$OUT/MANIFEST.txt"
echo "    （以上若为空即通过）"

echo "===== 3) 反向引用：工程树里还有哪些地方引用了这些模块（决定要不要打补丁）====="
PAT='parking_manager_node|parking_recorder_node|parking_shuttle_node|etc_manager_node|etc_rsu_node|etc_recorder_node|parking_zones\.yaml|etc_zones\.yaml'
grep -rn -E "$PAT" "$PKG/setup.py" "$PKG/CMakeLists.txt" "$PKG/launch" 2>/dev/null | cut -c1-150 | sed 's|^|    |' || echo "    打包/launch：无引用"
echo "  其它源码里的 import（parking_/etc_ 模块名）："
grep -rn -E "^(import|from) v2x_demo\.(parking_|etc_)|import (parking|etc)_" "$NODE_SRC" 2>/dev/null | grep -v -E "/(parking_|etc_)" | cut -c1-150 | sed 's|^|    |' || echo "    无跨模块 import"
echo "  共享文件里的 parking/etc 引用行数（这两个同时服务任务一~四，本轮**不自动改**，只报数）："
for SH in "$NODE_SRC/v2x_dashboard.py" "$TOOLS/vid_rec.py"; do
  if [ -f "$SH" ]; then
    echo "    ${SH#$D/}  $(grep -c -E 'parking|etc' "$SH" || true) 行"
  else
    echo "    找不到 $SH"
  fi
done

echo "===== 3b) 留下来的脚本里对「将被隔离的脚本」的调用点（不改就会指向空气）====="
# 分级由 tools/quarantine_invoke_check_307.py 做：第 1 类=真调用（必须补丁），
# 第 2 类=只是读 /tmp 日志（节点走了就再也读不到新行，脚本自己多半有 || true 兜住）。
python3 "$TOOLS/quarantine_refer_check_307.py" > "$OUT/refer.log" 2>&1 || true
python3 "$TOOLS/quarantine_invoke_check_307.py" > "$OUT/invoke.log" 2>&1 || true
sed -n '1,3p' "$OUT/invoke.log" | sed 's|^|    |'
echo "  第 1 类逐条（留在树上的文件:行号 → 调用内容）："
sed -n '/第 1 类：/,/第 2 类：/p' "$OUT/invoke.log" | grep -E '^  [^ ]|^[[:space:]]+[^[:space:]]' | sed 's|^|    |'
echo '  C 层就是从这些调用里按"主业是任务五六"(ratio56>=0.5)判出来的，明细见 '"$OUT"'/tier_c.log'

echo "===== 3c) 漏网检查：内容里讲 parking/ETC、但文件名没带的 tools 脚本 ====="
grep -rlE "parking|etc_gate|etc_rsu|BarrierCommand" "$TOOLS"/*.sh "$TOOLS"/*.py 2>/dev/null \
  | grep -v -Ff "$OUT/MANIFEST.txt" | sed "s|$TOOLS/||;s|^|    |" || echo "    无"

echo "===== 3d) 共享文件里要动的两处：vid_rec 的 --mode parking 面板 + 整栈重启脚本的停车场段 ====="
echo "  vid_rec.py 停车场面板摘除预演（--dry，一个字节没写）："
python3 "$TOOLS/quarantine_patch_vidrec307.py" --dry 2>&1 | sed 's|^|    |'
echo "  restart_clean_stack.sh 里要摘掉的整段（9c 停车场三节点）："
grep -n -E "\[restart\] 9c|start_parking_nodes" "$TOOLS/restart_clean_stack.sh" | sed 's|^|    |'
echo "  v2x_dashboard.py：查过它**没有** /parking 或 /etc 话题订阅，停车场几何来自"
echo "    config/v2x_scene.yaml（留守），所以看板不用改；移走节点后那两块几何照样画得出来。"

echo "===== 4) 归档 tar（list/apply 都做，内容以清单为准）====="
TAR=$ARCH/任务五六代码隔离_1016.tgz
MH=$(md5sum "$OUT/MANIFEST.txt" | cut -c1-32)
if [ ! -f "$TAR" ] || [ "$(cat "$OUT/_man.md5" 2>/dev/null)" != "$MH" ]; then
  tar -czf "$TAR" -T "$OUT/MANIFEST.txt" 2>&1 | sed 's|^|    |' || true
  echo "$MH" > "$OUT/_man.md5"
  sync
fi
echo "  tar 条目数 $(tar -tzf "$TAR" 2>/dev/null | wc -l)（应等于清单 $NTOT）"
echo "  tar 字节 $(wc -c < "$TAR")｜sha256 $(sha256sum "$TAR" | cut -c1-32)"
[ "$(tar -tzf "$TAR" | wc -l)" = "$NTOT" ] || echo "  !! 归档条目数与清单不等，apply 之前必须补齐"

if [ "$MODE" = "list" ]; then
  echo "===== list 档结束：工程树未被改动。确认清单无误后再跑 apply ====="
  echo "清单文件：$OUT/MANIFEST.txt   归档：$TAR"
  echo "Q307DONE"
  exit 0
fi

echo "===== 5) apply 前置闸：停车场/ETC 节点必须先停，且停车要先补零速 ====="
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash
for c in car_2 car_3 car_6; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
done
for PAT in 'parking[_]manager[_]node' 'parking[_]recorder[_]node' 'parking[_]shuttle[_]node' \
           'etc[_]manager[_]node' 'etc[_]rsu[_]node' 'etc[_]recorder[_]node'; do
  P=$(pgrep -f "lib/v2x[_]demo/$PAT" || true)
  [ -n "$P" ] && kill -9 $P || true
done
sleep 2
for c in car_2 car_3 car_6; do
  timeout -k 2 8 ros2 topic pub --once /$c/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' >/dev/null 2>&1 || true
done
echo "  停后实例点名：parking $(pgrep -cf 'lib/v2x[_]demo/parking[_]' || true)｜etc $(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)（都应 0）"

echo "===== 6) 移动：工程树 -> 隔离目录（保持相对路径，便于将来原路移回）====="
MOVED=0
while IFS= read -r F; do
  REL="${F#$D/}"
  mkdir -p "$ARCH/$(dirname "$REL")"
  mv "$F" "$ARCH/$REL" && MOVED=$((MOVED+1)) || echo "  !! 移动失败 $REL"
done < "$OUT/MANIFEST.txt"
# 同模块的 __pycache__ 残留（.pyc）一并移走，否则 install 侧符号链接会指向不存在的源
find "$NODE_SRC/__pycache__" -name 'parking_*.pyc' -o -name 'etc_*.pyc' 2>/dev/null | while read -r C; do
  mv "$C" "$ARCH/pyc_$(basename "$C")" || true
done
echo "  已移动 $MOVED 项（清单 $NTOT 项）"
[ "$MOVED" = "$NTOT" ] || echo "  !! 移动数与清单不等，把差异报出来别继续"

echo "===== 7) 同步 home src 与 install 里的同源副本 ====="
H=/home/hzm/v2x_ws/src/v2x_demo
for F in v2x_demo/parking_manager_node.py v2x_demo/parking_recorder_node.py v2x_demo/parking_shuttle_node.py \
         v2x_demo/etc_manager_node.py v2x_demo/etc_rsu_node.py v2x_demo/etc_recorder_node.py \
         config/parking_zones.yaml config/etc_zones.yaml; do
  [ -e "$H/$F" ] && { mkdir -p "$ARCH/wsl_home/$F"; mv "$H/$F" "$ARCH/wsl_home/$F/x"; echo "  已移出 home src：$F"; } || true
done

echo "===== 7b) 运行期配置副本对账（303 那轮把同步路径写错了，yaml 一直没落地）====="
# 现场证据：install/share/.../signal_drive_params.yaml 是符号链接 → home src 那份
# （149 行、含 7 处 car_7/etc_），而 D 盘真值是 125 行、0 处。启动时 launch 生成的
# /tmp/launch_params_* 取的就是这一份，所以现在跑着的驱动节点正在读带 car_7 的配置。
# 停车场/ETC 代码一隔离，这份配置里的 car_7 段就变成"给一辆不存在的车派活"。
HD=$H/config/signal_drive_params.yaml
DD=$PKG/config/signal_drive_params.yaml
cp -a "$HD" "$ARCH/signal_drive_params.yaml.home_pre_quarantine_1016.bak" 2>/dev/null || true
cp -a "$DD" "$HD"
echo "  三方 md5（D 盘真值 / home src / install 符号链接目标）："
md5sum "$DD" "$HD" "$(readlink -f /home/hzm/v2x_ws/install/v2x_demo/share/v2x_demo/config/signal_drive_params.yaml)" 2>&1 | sed 's|^|    |'
echo "  同步后 car_7/etc_ 计数：$(grep -cE 'car_7|etc_' "$HD" || true)（应为 0）"
echo "  顺带报数（本脚本不改）驱动节点 .py 的三份副本是否一致："
md5sum "$NODE_SRC/signal_drive_node.py" "$H/v2x_demo/signal_drive_node.py" 2>/dev/null | sed 's|^|    |' || true
md5sum /tmp/v2x_demo.pre_onboard_20261015_131512/v2x_demo/signal_drive_node.py 2>/dev/null | sed 's|^|    10-14 快照: |' || true

echo "===== 8) 改引用：把 entry_points / launch 里指向被移走脚本的条目摘掉 ====="
for FP in "$PKG/setup.py" "$PKG/CMakeLists.txt"; do
  [ -f "$FP" ] || continue
  cp -a "$FP" "$ARCH/$(basename "$FP").pre_quarantine_1016.bak"
  echo "  $FP 里 parking/etc 相关行（移动前快照）："
  grep -n -E "$PAT" "$FP" | sed 's|^|    |' || echo "    无"
done
for PP in "$PKG/CMakeLists.txt" "$PKG/setup.py"; do
  [ -f "$PP" ] || continue
  if grep -q -E "parking_|etc_" "$PP"; then
    cp -a "$PP" "$ARCH/$(basename "$PP").pre_quarantine_1016.bak"
    echo "  打补丁：$PP（原件已备份进隔离目录）"
    python3 "$TOOLS/quarantine_patch_pack307.py" "$PP" 2>&1 | sed 's|^|    |'
  else
    echo "  $PP 无 parking/etc 引用，不动"
  fi
done
for L in "$PKG"/launch/*.py; do
  [ -f "$L" ] || continue
  if grep -q -E "$PAT" "$L"; then
    cp -a "$L" "$ARCH/launch_$(basename "$L").pre_quarantine_1016.bak"
    echo "  !! launch 里有引用，需人工确认后再改：$L"
  fi
done

echo "===== 8b) 共享文件的两处补丁（都在隔离目录里留了原件备份）====="
cp -a "$TOOLS/restart_clean_stack.sh" "$ARCH/restart_clean_stack.sh.pre_quarantine_1016.bak"
python3 "$TOOLS/quarantine_patch_restart307.py" 2>&1 | sed 's|^|    |'
bash -n "$TOOLS/restart_clean_stack.sh" && echo "    bash -n 通过" || echo "    !! bash -n 失败"
cp -a "$TOOLS/vid_rec.py" "$ARCH/vid_rec.py.pre_quarantine_1016.bak"
python3 "$TOOLS/quarantine_patch_vidrec307.py" 2>&1 | sed 's|^|    |'
python3 -m py_compile "$TOOLS/vid_rec.py" && echo "    py_compile 通过" || echo "    !! py_compile 失败"
echo "    --help 实际回包（choices 里不该再有 parking）："
timeout -k 3 25 python3 "$TOOLS/vid_rec.py" --help 2>&1 | tr -d '\r' | grep -aE "mode|choices" | head -4 | sed 's|^|      |'

echo "===== 9) 编译硬闸：colcon build（只这两个包，失败就停在这里）====="
cd /home/hzm/v2x_ws
if colcon build --packages-select v2x_demo v2x_msgs > "$OUT/build.log" 2>&1; then
  echo "  colcon build 通过；尾部："
else
  echo "  !! colcon build 失败，工程树已移动但引用可能未改净 —— 用 $ARCH 原路移回"
fi
tail -12 "$OUT/build.log" | sed 's|^|    |'

echo "===== 10) 就绪闸：剩下的三条链路必须完好 ====="
bash "$D/tools/start_signal.sh" > "$OUT/start_signal.log" 2>&1 || true
sleep 18
for PAT in 'signal[_]drive_node.py' 'traffic[_]light[_]manager' 'adaptive[_]timing_node' \
           'traffic[_]light[_]recorder' 'plate[_]gen[_]node' 'plate[_]recognizer[_]node' \
           'plate[_]recorder[_]node' 'speed[_]detect[_]node' 'speed[_]recorder[_]node' \
           'websocket[_]bridge'; do
  echo "  $(printf '%-24s' "$PAT") $(pgrep -cf "lib/v2x[_]demo/$PAT" || true)"
done
echo "  已隔离的：parking $(pgrep -cf 'lib/v2x[_]demo/parking[_]' || true)｜etc $(pgrep -cf 'lib/v2x[_]demo/etc[_]' || true)（应 0）"
python3 "$D/tools/fleet_moving_guard303.py" /tmp/signal_drive_path.csv 30 || true
echo "  看板 :8090 -> $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8090/ || true)｜回显 :8080 -> $(curl -s -o /dev/null -w '%{http_code}' http://127.0.0.1:8080/ || true)"
echo "  gzserver=$(pgrep -x gzserver | head -1)｜gzclient=$(pgrep -x gzclient | head -1 || echo 无)"

echo "===== 11) 结论 ====="
echo "  隔离目录：$ARCH（含 tar 归档 + 移动后的原路径副本）"
echo "  注意：car_2/car_3 的 drive:false 与 car_6 摆渡位仍在 yaml/world 里 —— 摆渡代码已隔离，"
echo "        它们就是停着不动的两辆车，要不要归队是下一个单独决定，本脚本没替你改。"
echo "Q307DONE"
