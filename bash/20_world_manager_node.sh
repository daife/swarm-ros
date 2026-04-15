#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

ARGS=(
  -p "world_mode:=${WORLD_MODE}"                     # 可选: fixed(random对比)/random(泛化训练)
  -p "seed_mode:=${SEED_MODE}"                       # 可选: single/sequence/none
  -p "seed_value:=${SEED_VALUE}"                     # 可选: single 模式固定种子
  -p "seed_sequence:=${SEED_SEQUENCE}"               # 可选: sequence 模式种子列表
  -p "obstacle_density:=${OBSTACLE_DENSITY}"         # 可选: 障碍密度
  -p "world_half_extent:=${WORLD_HALF_EXTENT}"       # 可选: 世界边界半径
)

echo "[20_world_manager_node] start"
exec ros2 run world_manager world_manager_node --ros-args "${ARGS[@]}"
