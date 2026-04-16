#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/env_defaults.sh"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                   # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                     # 可选: 逃避者数量
  -p "max_episodes:=${MAX_EPISODES}"                   # 可选: 最大回合数
  -p "max_steps:=${MAX_STEPS}"                         # 可选: 每回合最大步数
  -p "reset_policy:=${RESET_POLICY}"                   # 可选: 1固定/2安全随机/3先重置世界
  -p "collision_penalty:=${COLLISION_PENALTY}"         # 可选: 碰撞惩罚累计值
  -p "capture_radius:=${CAPTURE_RADIUS}"               # 可选: 捕获判定半径
  -p "capture_steps:=${CAPTURE_STEPS}"                 # 可选: 连续捕获步数阈值
  -p "initial_reset_delay_sec:=${INITIAL_RESET_DELAY_SEC}" # 可选: 启动首次 reset 延迟
  -p "reset_agents_delay_sec:=${RESET_AGENTS_DELAY_SEC}" # 可选: world reset 后 agent reset 延迟
  -p "reset_grace_steps:=${RESET_GRACE_STEPS}"         # 可选: reset 后碰撞忽略步数
  -p "always_reset_world:=${ALWAYS_RESET_WORLD}"       # 可选: 每轮是否都刷新障碍
)

echo "[24_training_coordinator_node] start"
exec ros2 run training_coordinator training_coordinator_node --ros-args "${ARGS[@]}"
