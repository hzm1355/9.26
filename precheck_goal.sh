#!/usr/bin/env bash
# 预检：send_goal.sh 展开后的 YAML 与 bash 语法双重校验（不实际发目标）
SG=/mnt/d/desktop1/9.26.1/v2x_traffic_sim/send_goal.sh
bash -n "$SG" && echo "BASH_SYNTAX_OK" || { echo "BASH_SYNTAX_FAIL"; exit 1; }
# 与 send_goal.sh 2.47 -5.39 0.605 0.796 展开后一致的字符串
S='{pose: {header: {frame_id: map}, pose: {position: {x: 2.47, y: -5.39, z: 0.0}, orientation: {z: 0.605, w: 0.796}}}}'
python3 -c 'import yaml,sys; d=yaml.safe_load(sys.argv[1]); print("YAML_OK", d)' "$S"
