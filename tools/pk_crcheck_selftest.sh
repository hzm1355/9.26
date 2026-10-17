#!/bin/bash
# 一次性点验：本轮改过的两个文件有没有被写成 CRLF（D 盘的老坑），再直接跑离线自测。
set -e
SIM=/mnt/d/desktop1/9.26.1/v2x_traffic_sim
sed -i 's/\r$//' "$0"
for f in "$SIM/tools/parking_shuttle_selftest.py" \
         "$SIM/src/v2x_demo/v2x_demo/parking_shuttle_node.py" \
         "$SIM/tools/parking_shuttle_selftest.sh"; do
  n=$(tr -cd '\r' < "$f" | wc -c)
  echo "CR $n  $f"
  [ "$n" -eq 0 ] || sed -i 's/\r$//' "$f"
done
exec bash "$SIM/tools/parking_shuttle_selftest.sh"
