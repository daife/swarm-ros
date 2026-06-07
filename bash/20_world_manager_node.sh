#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

ARGS=(
  -p "world_mode:=${WORLD_MODE}"                     # 可选: none(无障碍)/fixed/random
  -p "seed_mode:=${SEED_MODE}"                       # 可选: single/sequence/none
  -p "seed_value:=${SEED_VALUE}"                     # 可选: single 模式固定种子
  -p "seed_sequence:=${SEED_SEQUENCE}"               # 可选: sequence 模式种子列表
  -p "obstacle_density:=${OBSTACLE_DENSITY}"         # 可选: 障碍密度；0 表示无障碍
  -p "world_half_extent:=${WORLD_HALF_EXTENT}"       # 可选: 世界边界半径
  -p "world_name:=${WORLD_NAME}"                     # 可选: Gazebo world 名
  -p "obstacle_model_package:=${OBSTACLE_MODEL_PACKAGE}" # 可选: 障碍模型来源包
  -p "obstacle_pool_box:=${OBSTACLE_POOL_BOX}"       # 可选: 方形障碍池数量
  -p "obstacle_pool_rect:=${OBSTACLE_POOL_RECT}"     # 可选: 长方体障碍池数量
  -p "obstacle_pool_cylinder:=${OBSTACLE_POOL_CYLINDER}" # 可选: 圆柱障碍池数量
  -p "obstacle_staging_x:=${OBSTACLE_STAGING_X}"     # 可选: 障碍 staging 区 x
  -p "obstacle_staging_y:=${OBSTACLE_STAGING_Y}"     # 可选: 障碍 staging 区 y
  -p "obstacle_staging_stride:=${OBSTACLE_STAGING_STRIDE}" # 可选: 障碍 staging 间距
  -p "obstacle_spawn_attempts:=${OBSTACLE_SPAWN_ATTEMPTS}" # 可选: 障碍采样尝试次数
  -p "obstacle_min_clearance:=${OBSTACLE_MIN_CLEARANCE}"   # 可选: 障碍间最小间距
)

echo "[20_world_manager_node] start"
exec ros2 run world_manager world_manager_node --ros-args "${ARGS[@]}"
