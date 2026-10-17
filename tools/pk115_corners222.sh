#!/bin/bash
# 勘查 222：东腿挪到 x=16.20 后，北环与南环要各向东延长一段，核那两段净空。
set -e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim
echo "== A 北环延伸段 y=1.5 走廊 x 14.0..16.7"
python3 tools/lot_clearance.py --rect=14.00,16.70,1.00,2.00 || true
echo
echo "== B 南环延伸段 y=-16.8 走廊 x 14.0..16.7"
python3 tools/lot_clearance.py --rect=14.00,16.70,-17.30,-16.30 || true
echo
echo "== C 东腿 x=16.20 与北环角点邻域"
python3 tools/lot_clearance.py --rect=15.40,17.00,0.40,2.60 || true
echo
echo "== D 东腿 x=16.20 与南环角点邻域"
python3 tools/lot_clearance.py --rect=15.40,17.00,-17.60,-15.40 || true
