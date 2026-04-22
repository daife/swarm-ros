#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_FILE="${1:-}"
source "${SCRIPT_DIR}/_load_config.sh" "${CONFIG_FILE}"

ARGS=(
  -p "num_pursuers:=${NUM_PURSUERS}"                                  # 可选: 追捕者数量
  -p "num_evaders:=${NUM_EVADERS}"                                    # 可选: 逃避者数量
  -p "pursuer_algo:=${PURSUER_ALGO}"                                  # 可选: maddpg/flocking/cbc_tp_net(回退flocking)
  -p "pursuer_train_mode:=${PURSUER_TRAIN_MODE}"                      # 可选: train_scratch/train_load/eval_only
  -p "evader_algo:=${EVADER_ALGO}"                                    # 可选: ddpg/flocking
  -p "evader_train_mode:=${EVADER_TRAIN_MODE}"                        # 可选: train_scratch/train_load/eval_only

  -p "max_accel:=${MAX_ACCEL}"                                        # 可选: 动作加速度幅值上限
  -p "pursuer_max_accel:=${PURSUER_MAX_ACCEL}"                        # 可选: pursuer 最大加速度
  -p "evader_max_accel:=${EVADER_MAX_ACCEL}"                          # 可选: evader 最大加速度
  -p "capture_radius:=${CAPTURE_RADIUS}"                              # 可选: 捕获判定距离
  -p "collision_penalty:=${COLLISION_PENALTY}"                        # 可选: 碰撞惩罚
  -p "discount_gamma:=${DISCOUNT_GAMMA}"                              # 可选: 折扣因子
  -p "target_tau:=${TARGET_TAU}"                                      # 可选: target 网络软更新系数

  -p "pursuer_actor_lr:=${PURSUER_ACTOR_LR}"                          # 可选: pursuer actor 学习率
  -p "pursuer_critic_lr:=${PURSUER_CRITIC_LR}"                        # 可选: pursuer critic 学习率
  -p "evader_actor_lr:=${EVADER_ACTOR_LR}"                            # 可选: evader actor 学习率
  -p "evader_critic_lr:=${EVADER_CRITIC_LR}"                          # 可选: evader critic 学习率

  -p "buffer_capacity:=${BUFFER_CAPACITY}"                            # 可选: replay buffer 容量
  -p "batch_size:=${BATCH_SIZE}"                                      # 可选: 采样 batch 大小
  -p "warmup_steps:=${WARMUP_STEPS}"                                  # 可选: 起训前热身步数
  -p "train_updates_per_step:=${TRAIN_UPDATES_PER_STEP}"              # 可选: 每步参数更新次数
  -p "exploration_noise_std:=${EXPLORATION_NOISE_STD}"                # 可选: 探索噪声标准差
  -p "policy_update_interval:=${POLICY_UPDATE_INTERVAL}"              # 可选: 策略更新间隔
  -p "optimize_horizon_steps:=${OPTIMIZE_HORIZON_STEPS}"              # 可选: 优化时域参数

  -p "checkpoint_dir:=${CHECKPOINT_DIR}"                              # 可选: 权重保存目录
  -p "checkpoint_prefix:=${CHECKPOINT_PREFIX}"                        # 可选: 权重文件名前缀
  -p "use_synthetic_obs_when_missing:=${USE_SYNTHETIC_OBS_WHEN_MISSING}" # 可选: 启动早期缺观测回退
  -p "torch_device:=${TORCH_DEVICE}"                                  # 可选: auto/cpu/cuda/cuda:0
)

# ros2 参数解析不接受空字符串覆写（例如 -p xxx:=）。
# 因此权重参数只在非空时追加。
if [[ -n "${PURSUER_WEIGHTS}" ]]; then
  ARGS+=( -p "pursuer_weights:=${PURSUER_WEIGHTS}" )
fi

if [[ -n "${EVADER_WEIGHTS}" ]]; then
  ARGS+=( -p "evader_weights:=${EVADER_WEIGHTS}" )
fi

echo "[23_rl_policy_node] start"
exec ros2 run rl_policy rl_policy_node --ros-args "${ARGS[@]}"
