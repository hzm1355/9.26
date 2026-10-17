#!/bin/bash
# 一次性完成：删墙后的整栈重启 + 效果复核（盯住"南口复位那一格能不能真转开"）。
#
# 为什么要串成一个脚本：重启整栈是破坏性动作，而它只有配上"改后要用效果复核"
# 才算交付 —— 分开跑就得两次批准，中间那段没人盯，日志会被后续动作冲掉。
#
# 用法（WSL）：bash /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/pk179_rebootverify308.sh [fleet] [盯几轮]
#   fleet = yes|skip（默认 yes，带五车车队；skip 则驱动节点全程不在）
#   盯几轮 = pk_watch308.sh 的轮数，每轮 10 s（默认 46 轮 ≈ 8 min，够跑完
#            进场→出场→复位那一格的原地转，还留一段第二轮起步）
#
# 全程只用 set +e：收尾是"报告 + 退出码"，检查里的 grep/timeout/wc 天然会非零，
# set -e 会在中途把脚本掐断，剩下的复核一行都不执行，看起来像"跑过了"其实没验。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
FLEET=${1:-yes}
ROUNDS=${2:-46}
RESTART_LOG=/tmp/pk179_restart.log
WATCH_LOG=/tmp/pk179_watch.log
SHLOG=/tmp/shuttle_car_3.log
unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
source /home/hzm/v2x_ws/install/setup.bash

echo '### 1/6 前置闸：world 必须先过体检，否则整栈重启会把 gzserver 起死'
# 手删 collision 之后最容易犯的错是 XML 注释里出现连续两个减号 —— gzserver 直接
# 拒载整个 world，那是"重启完什么都没了"，比不改还糟。所以这一条不通就绝不往下走。
PRECHK=/tmp/pk179_prechk.txt
python3 "$SIM/tools/pk177_worldprechk308.py" > "$PRECHK" 2>&1
RC=$?
sed 's/^/    /' "$PRECHK"
if [ "$RC" != 0 ]; then
  echo "!! 体检没过（rc=$RC），不重启。先修 world 再来。"
  exit 1
fi
if [ "$(grep -ac 'c130 的 collision 行：0 处' "$PRECHK")" != "1" ]; then
  echo '!! 体检没报"c130 已删除"，说明改的不是运行期加载的那份 world，不重启。'
  exit 1
fi
if [ "$(grep -acE '吃进|最近拦路件' "$PRECHK")" = "0" ]; then
  echo '!! 体检没打出净空表（判据脚本自己没跑完），不重启。'
  exit 1
fi

echo '### 2/6 记下改前基线（楔住现场与节点挂起状态），事后要拿它对比'
echo "  car_3 位姿：$(timeout 8 gz model -m car_3 -p 2>&1 | tr -d '\n' | cut -c1-60)"
grep -a '停驶挂起' "$SHLOG" 2>/dev/null | tail -1 | cut -c1-200 | sed 's/^/  挂起行| /'

echo "### 3/6 整栈重启（fleet=$FLEET，约 5 min，日志 $RESTART_LOG）"
bash "$SIM/tools/restart_clean_stack.sh" "$FLEET" 2>&1 | tee "$RESTART_LOG" | tail -25
echo "  gzserver=$(pgrep -x gzserver | head -1) gzclient=$(pgrep -x gzclient | head -1)"
if [ -z "$(pgrep -x gzserver)" ]; then
  echo '!! 重启完 gzserver 不在 —— 别再往下盯，直接查 gzserver 的 stderr'
  exit 1
fi

echo '### 4/6 生效核对：运行期那份 world 必须是改过的、car_3 必须回到出生位'
# 光看文件改了不算生效（memory「先确认运行期加载的是哪份文件副本」）：gzserver 是
# 启动时读 SDF 的，所以这里要拿"新起的 gzserver 的 cwd+cmdline"对上 D 盘那份文件，
# 再看 car_3 的位姿 —— 重启会把它弹回 world 里的出生位，楔住的状态随之消失。
GP=$(pgrep -x gzserver | head -1)
echo "  gzserver pid=$GP cwd=$(readlink -f /proc/$GP/cwd) cmd=$(tr '\0' ' ' < /proc/$GP/cmdline | cut -c1-80)"
for i in 1 2 3; do
  P=$(timeout 8 gz model -m car_3 -p 2>&1 | tr -d '\n' | cut -c1-60)
  echo "$P" | grep -aq 'Unable' || break
  echo "  第 $i 次读不到位姿（世界还在加载），等 20 s"
  sleep 20
done
echo "  car_3 位姿：$P"

echo '### 5/6 效果复核：盯进场→出场→复位那一格的原地转（46 轮 × 10 s）'
bash "$SIM/tools/pk_watch308.sh" "$ROUNDS" car_3 2>&1 | tee "$WATCH_LOG" | tail -30

echo '### 6/6 判读：这一轮要看到的是"转开了"，不是又挂起'
# 只数关键词、不拿"日志有行数"当证据：watcher 每秒都在写状态行，行数多不等于转开了。
echo '  --- 顶住/挂起类关键词 ---'
for K in '没转过' '停驶挂起' '顶住次数到上限' '转不开'; do
  C=$(grep -ac "$K" "$SHLOG" 2>/dev/null || echo 0)
  printf '    %-14s %s 行\n' "$K" "${C:-0}"
done
echo '  --- 复位段是否跨过那一格 ---'
grep -aE 'reset_route|复位|#1[4-6]|原地转|航点' "$SHLOG" 2>/dev/null | tail -12 | cut -c1-170 | sed 's/^/    /'
echo '  --- 通行事件与闸机动作（重启后新账，改前已轮转备份） ---'
wc -l /tmp/v2x_parking/parking_pass.jsonl /tmp/v2x_parking/barrier_actions.jsonl 2>/dev/null | sed 's/^/    /'
tail -2 /tmp/v2x_parking/parking_pass.jsonl 2>/dev/null | cut -c1-190 | sed 's/^/    /'
echo '[pk179] DONE（重启日志 /tmp/pk179_restart.log，盯梢日志 /tmp/pk179_watch.log）'
