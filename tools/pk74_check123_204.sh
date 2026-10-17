#!/bin/bash
# 报告要引用的三件小事，一次跑完（全离线/只读，不改任何在环状态）：
# 1) 条目 1/2/3 成片的体检数字（帧数/时长/分辨率/体积/亮度分布）——
#    这三段录的时候没把 vid_check 的输出留档，报告与 README 的表不能靠猜；
# 2) 三份"现场快照"是不是同一份内容（研究员指出跨 2 小时抓拍图计数不变，先坐实）；
# 3) 记录器自报行数与字幕时间轴末行，用来核对片长与字幕时长是否同尺。
set +e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
TOOLS=$SIM/tools
VID='/mnt/d/desktop1/9.26.1/视频取证/任务五_停车场闸机'
EV=$SIM/evidence/任务五_视频取证字幕
OUT=/mnt/d/desktop1/9.26.1/_diag155/check123_204.log

for N in 1 2 3; do
  case $N in
    1) F="$VID/条目1_场景建模_四要素点位尺寸与可配置.mp4";;
    2) F="$VID/条目2_图像话题与车牌识别算法对接_三方对账.mp4";;
    3) F="$VID/条目3_闸机动力学_升降时序防砸与独立实测.mp4";;
  esac
  echo "########## 条目 $N 体检：$(basename "$F") ##########"
  ls -l --time-style=+'%F_%T' "$F" 2>/dev/null | tr -d '\r'
  timeout -k 20 600 python3 $TOOLS/vid_check.py "$F" 2>&1 | tr -d '\r'
  echo
done

echo '########## 三份现场快照是否同内容（md5 + 关键行）##########'
md5sum "$EV/条目2_现场快照.txt" "$EV/条目3_现场快照.txt" "$EV/条目4_现场快照.txt" 2>/dev/null | tr -d '\r'
echo '--- 每份的快照时刻与账本行数、抓拍图计数 ---'
grep -a '快照\|jsonl\|png' "$EV/条目2_现场快照.txt" "$EV/条目3_现场快照.txt" "$EV/条目4_现场快照.txt" 2>/dev/null | tr -d '\r' | cut -c1-150
echo
echo '########## 录制器自报行（帧数/时长）##########'
for N in 2 3 4; do
  echo "--- 条目 ${N}_录制器日志.txt ---"
  head -3 "$EV/条目${N}_录制器日志.txt" 2>/dev/null | tr -d '\r' | cut -c1-170
done
echo
echo '########## 字幕时间轴末行（脚本内容结束时刻）##########'
for N in 2 3 4; do
  echo -n "条目 $N 字幕最后一行："
  tail -1 "$EV/条目${N}_字幕流.txt" 2>/dev/null | tr -d '\r' | cut -c1-120
done
echo CHECK123_DONE
