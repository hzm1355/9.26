#!/bin/bash
# 任务三 条目4 视频取证：识别结果实时界面展示 + 识别记录经 ROS2 话题存储 + 数据导出。
# 画面右侧就是实时展示界面（抓拍/识别计数 + 最近十条结果），
# 同一批结果由 plate_recorder_node 订阅 /plate/result 落到 JSONL，
# 再用 /plate/export 服务导成 CSV/JSON，check_export_csv.py 校验字段完整性。
#
# 两个坑（上一版就踩在字幕上）：
#   · Trigger 的回包是 `message='...'` 而不是 `message: ...`，用后者做 sed 会把
#     整行过滤没，字幕里等于"调了服务但什么都没发生"；
#   · `ls /tmp/v2x_records/*.csv | tail -3` 按字母序取尾，拿到的是 speed_sweep 系列
#     文件，真正的导出物 plate_records.csv 反而不露脸。
# 导出条数是记录器**本进程启动以来**的内存窗口，JSONL 是跨重启的全量追加流水，
# 两者对不上是正常的，字幕里必须写清楚，否则会被读成"导出漏了 989 条"。
# 第三坑：capq 的参数用双引号时，里面的 $var 会被驱动脚本自己的 shell 先展开成空串
# （上一版字幕就成了 "JSONL  行 / CSV  行"），凡是要在命令里自己赋值取值的都写单引号。
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e   # 驱动脚本宁可少一步操作也不能中途退出：中途退出会把字幕停在半路，整段视频作废
OUT="$VID_ROOT/任务三_卡口车牌识别/条目4_识别结果实时展示_话题存储_数据导出.mp4"
G3=$VIEW_G3
G2=$VIEW_G2
REC=/tmp/v2x_records/plate_records.jsonl

vid_start "$OUT" 175 "$VIEW_INTER,$G3,$G2" plate \
  "任务三 条目4  识别结果实时界面展示 + ROS2 话题存储 + 数据导出"
cap "本视频右栏 = 实时展示界面；下面还会核对 HTTP 看板、话题存活性与导出文件"
capq "bash $SIM_ROOT/tools/ensure_recorder.sh | tail -2"
sleep 8
cap "== 4a 记录链路存活：/plate/result 的发布者与订阅者数（订阅者=记录器）=="
capq "timeout -k 3 20 ros2 topic info /plate/result | sed -n '1,4p'"
capq "pgrep -af 'plate[_]recorder_node' | head -2"
sleep 18
cap "== 4b 识别记录经话题存储：JSONL 落盘行数与 50 s 窗口内的增量（卡口约每 30~70 s 过一次车）=="
capq 'a=$(wc -l < /tmp/v2x_records/plate_records.jsonl); sleep 50; b=$(wc -l < /tmp/v2x_records/plate_records.jsonl); echo "落盘 $a 行 -> 50 s 后 $b 行（增量 $((b-a)) 行）"'
capq "tail -2 /tmp/v2x_records/plate_records.jsonl | cut -c1-150"
sleep 5
cap "== 4c 数据导出：调 /plate/export 服务生成 CSV/JSON（回包自带命中率与光照分组）=="
capq "timeout -k 3 45 ros2 service call /plate/export std_srvs/srv/Trigger 2>&1 | tr -d '\\r' | tail -2"
capq 'ls -l --time-style=+%H:%M:%S /tmp/v2x_records/plate_records.csv /tmp/v2x_records/plate_records.json | tr -s " " | cut -d" " -f5,6,7'
cap "== 4c-2 口径对照：CSV 是记录器本进程启动以来的窗口，JSONL 是跨重启全量流水 =="
capq 'j=$(wc -l < /tmp/v2x_records/plate_records.jsonl); c=$(awk "END{print NR-1}" /tmp/v2x_records/plate_records.csv); echo "JSONL $j 行 / CSV $c 行"'
sleep 8
cap "== 4d 导出文件字段与统计校验（18 字段、空值行、命中率、光照×卡口分组）=="
capq "timeout -k 3 60 python3 -u $TOOLS/check_export_csv.py 2>&1 | tail -13"
cap "== 4e 浏览器看板（:8090）同样在实时展示同一批抓拍 =="
capq "timeout -k 3 12 curl -s -o /dev/null -w '看板 HTTP %{http_code}\\n' http://localhost:8090/"
vid_wait
