#!/bin/bash
# 一次性验证：重启管理器后，六组实测起绿相位差是否等于 YAML 里的 offset 配置。
# 期望（周期 13 s，绿5 黄3 红5）：J1_ns 0 / J1_ew -5 / J2_ns -6 / J2_ew -1 / J3_ns -3 / J3_ew +2
source /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools/vid_common.sh
set +e
for p in $(pgrep -f 'adaptive[_]timing_node' || true); do kill -9 "$p" 2>/dev/null || true; done
sleep 1
for g in J1_ns J1_ew J2_ns J2_ew J3_ns J3_ew; do pub "/traffic_light/$g/adaptive_factor" 1.0; done
mgr_restart
sleep 25
timeout -k 3 95 python3 -u "$TOOLS/tl_offset_audit.py" 45 J1_ns
