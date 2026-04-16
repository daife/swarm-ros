#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                 # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                   # 可选: 逃避者数量
  -p "collision_distance:=${COLLISION_DISTANCE}"     # 可选: 统一碰撞距离约束
  -p "capture_radius:=${CAPTURE_RADIUS}"             # 可选: reset 时追逃最小间隔
  -p "world_half_extent:=${WORLD_HALF_EXTENT}"       # 可选: 世界边界半径
  -p "spawn_half_extent:=${SPAWN_HALF_EXTENT}"       # 可选: 出生采样边界半径
  -p "spawn_attempts:=${SPAWN_ATTEMPTS}"             # 可选: 单体采样最大尝试次数
  -p "spawn_z:=${FIXED_Z}"                           # 可选: 出生高度
  -p "agent_model_package:=${AGENT_MODEL_PACKAGE}"   # 可选: 模型来源包
  -p "agent_staging_x:=${AGENT_STAGING_X}"           # 可选: agent staging 区 x
  -p "agent_staging_y:=${AGENT_STAGING_Y}"           # 可选: agent staging 区 y
  -p "agent_staging_stride:=${AGENT_STAGING_STRIDE}" # 可选: agent staging 间距
  -p "world_name:=${WORLD_NAME}"                     # 可选: Gazebo world 名，影响 set_pose 服务路径
  -p "pursuer_name_prefix:=${PURSUER_PREFIX}"        # 可选: 追捕者模型前缀
  -p "evader_name_prefix:=${EVADER_PREFIX}"          # 可选: 逃避者模型前缀
)

echo "[21_agent_manager_node] start"
exec ros2 run agent_manager agent_manager_node --ros-args "${ARGS[@]}"
