#!/usr/bin/env bash
# 快速查看 gazebo_models-master 各模型的碰撞体尺寸/规模，决定场景摆放
BASE=/mnt/d/desktop1/9.26.1/gazebo_models-master
for m in school office_building gas_station law_office oak_tree pine_tree house_1; do
  echo "== $m =="
  if [ -f "$BASE/$m/model.sdf" ]; then
    grep -m2 '<size>\|<mesh>' "$BASE/$m/model.sdf" | head -4
    echo "  (sdf lines: $(wc -l < "$BASE/$m/model.sdf"))"
  else
    echo "  model.sdf 不存在"
  fi
done
