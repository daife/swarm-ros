#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                   # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                     # 可选: 逃避者数量
  -p "max_episodes:=${MAX_EPISODES}"                   # 可选: 最大回合数
  -p "max_steps:=${MAX_STEPS}"                         # 可选: 每回合最大步数
  -p "reset_policy:=${RESET_POLICY}"                   # 可选: 1固定/2安全随机/3先重置世界
  -p "collision_penalty:=${COLLISION_PENALTY}"         # 可选: 碰撞惩罚累计值
  -p "terminate_on_collision:=${TERMINATE_ON_COLLISION}" # 可选: 碰撞是否立即结束 episode；论文默认 false
  -p "capture_radius:=${CAPTURE_RADIUS}"               # 可选: 捕获判定半径
  -p "capture_required_pursuers:=${CAPTURE_REQUIRED_PURSUERS}" # 可选: 同时进入捕获半径的 pursuer 数
  -p "capture_steps:=${CAPTURE_STEPS}"                 # 可选: 连续捕获步数阈值
  -p "initial_reset_delay_sec:=${INITIAL_RESET_DELAY_SEC}" # 可选: 启动首次 reset 延迟
  -p "always_reset_world:=${ALWAYS_RESET_WORLD}"       # 可选: 每轮是否都刷新障碍；论文复现默认 false
  -p "metrics_output_dir:=${METRICS_OUTPUT_DIR}"       # 可选: 论文指标输出目录
  -p "metrics_window_episodes:=${METRICS_WINDOW_EPISODES}" # 可选: 成功率统计窗口
)

echo "[24_training_coordinator_node] start"
exec ros2 run training_coordinator training_coordinator_node --ros-args "${ARGS[@]}"
