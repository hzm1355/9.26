#!/bin/bash
# 开录前的依赖在场闸：条目 4/5 两段脚本点名的工具必须全在，缺一个就是几十分钟空转。
set +e
cd /mnt/d/desktop1/9.26.1/v2x_traffic_sim/tools || exit 1
for f in vid_common.sh pk_wait_pass169.sh pk_yaml_check165.py pk_action_stat.py \
         pk_safelower186.sh pk_pass_census.py pk_record_audit.py parking_cli.py \
         pk_probe308.py vid_rec.py; do
  if [ -f "$f" ]; then
    printf 'OK       %-24s %s B\n' "$f" "$(wc -c < "$f")"
  else
    printf 'MISSING  %s\n' "$f"
  fi
done
echo "--- bash -n ---"
for s in vid_common.sh vid_parking_4.sh vid_parking_5.sh; do
  if bash -n "$s" 2>/tmp/_synerr; then echo "语法过  $s"; else echo "语法挂  $s"; head -3 /tmp/_synerr; fi
done
echo "--- python 可解析 ---"
for p in pk_pass_census.py pk_record_audit.py pk_yaml_check165.py pk_action_stat.py; do
  if python3 -m py_compile "$p" 2>/tmp/_pyerr; then echo "可编译  $p"; else echo "编译挂  $p"; head -3 /tmp/_pyerr; fi
done
echo "--- 配置真值在场 ---"
wc -c /mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/parking_zones.yaml
