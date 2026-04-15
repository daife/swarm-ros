#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                 # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                   # 可选: 逃避者数量
  -p "safe_spawn_distance:=${SAFE_SPAWN_DISTANCE}"   # 可选: 与障碍最小安全距离
  -p "spawn_half_extent:=${SPAWN_HALF_EXTENT}"       # 可选: 出生采样边界半径
  -p "spawn_attempts:=${SPAWN_ATTEMPTS}"             # 可选: 单体采样最大尝试次数
  -p "world_name:=${WORLD_NAME}"                     # 可选: Gazebo world 名，影响 set_pose 服务路径
  -p "pursuer_name_prefix:=${PURSUER_PREFIX}"        # 可选: 追捕者模型前缀
  -p "evader_name_prefix:=${EVADER_PREFIX}"          # 可选: 逃避者模型前缀
)

echo "[21_agent_manager_node] start"
exec ros2 run agent_manager agent_manager_node --ros-args "${ARGS[@]}"
