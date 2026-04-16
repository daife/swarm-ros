#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                         # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                           # 可选: 逃避者数量
  -p "pursuer_name_prefix:=${PURSUER_PREFIX}"                # 可选: 追捕者模型前缀
  -p "evader_name_prefix:=${EVADER_PREFIX}"                  # 可选: 逃避者模型前缀
  -p "world_name:=${WORLD_NAME}"                             # 可选: Gazebo world 名
  -p "controller_enabled:=${GAZEBO_CONTROLLER_ENABLED}"      # 可选: true启用控制回写/false仅桥接
  -p "controller_dt:=${GAZEBO_CONTROLLER_DT}"                # 可选: 控制器积分周期(s)
  -p "max_speed:=${GAZEBO_MAX_SPEED}"                        # 可选: 最大速度
  -p "pursuer_max_speed:=${GAZEBO_PURSUER_MAX_SPEED}"        # 可选: pursuer 速度上限
  -p "evader_max_speed:=${GAZEBO_EVADER_MAX_SPEED}"          # 可选: evader 速度上限
  -p "linear_drag:=${GAZEBO_LINEAR_DRAG}"                    # 可选: 线性阻尼
  -p "control_requires_reset:=${GAZEBO_CONTROL_REQUIRES_RESET}" # 可选: 首次 reset 前禁止控制回写
  -p "world_half_extent:=${WORLD_HALF_EXTENT}"               # 可选: 位置边界裁剪半径
  -p "fixed_z:=${FIXED_Z}"                                   # 可选: 固定飞行高度
)

echo "[22_sim_bridge_node] start"
exec ros2 run sim_bridge sim_bridge_node --ros-args "${ARGS[@]}"
