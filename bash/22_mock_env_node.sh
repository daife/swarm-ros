#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                 # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                   # 可选: 逃避者数量
  -p "dt:=${GAZEBO_CONTROLLER_DT}"                   # 可选: mock 积分周期(s)
  -p "world_half_extent:=${WORLD_HALF_EXTENT}"       # 可选: 世界边界半径
  -p "max_speed:=${GAZEBO_MAX_SPEED}"                # 可选: 速度上限
  -p "linear_drag:=${GAZEBO_LINEAR_DRAG}"            # 可选: 线性阻尼
  -p "spawn_half_extent:=${SPAWN_HALF_EXTENT}"       # 可选: 出生采样半径
)

echo "[22_mock_env_node] start"
exec ros2 run mock_env mock_env_node --ros-args "${ARGS[@]}"
