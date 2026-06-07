#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# ROS setup 脚本在某些发行版里会直接读取未定义变量，和 `set -u` 冲突。
# 这里临时关闭 nounset，source 完毕后恢复严格模式。
set +u
source "${ROOT_DIR}/env.sh"
source "${ROOT_DIR}/install/setup.bash"
set -u

# ---------- 10_gazebo_gui.sh ----------
export WORLD_NAME="${WORLD_NAME:-urban}"                    # 可选: Gazebo world 名称
export WORLD_SDF="${WORLD_SDF:-${ROOT_DIR}/src/swarm_bringup/worlds/urban.world}"  # 可选: world 文件路径
export GAZEBO_GUI="${GAZEBO_GUI:-true}"                    # 可选: true/false（默认图形化）

# ---------- 11_parameter_bridge.sh ----------
export NUM_PURSUERS="${NUM_PURSUERS:-5}"                   # 可选: 追捕者数量
export NUM_EVADERS="${NUM_EVADERS:-1}"                     # 可选: 逃避者数量
export PURSUER_PREFIX="${PURSUER_PREFIX:-pursuer}"         # 可选: 追捕者模型名前缀
export EVADER_PREFIX="${EVADER_PREFIX:-evader}"            # 可选: 逃避者模型名前缀

# ---------- 20_world_manager_node.sh ----------
export WORLD_MODE="${WORLD_MODE:-none}"                    # 可选: none/fixed/random；默认无障碍物训练
export SEED_MODE="${SEED_MODE:-single}"                    # 可选: single/sequence/none(纯随机种子)
export SEED_VALUE="${SEED_VALUE:-42}"                      # 可选: 固定随机种子
export SEED_SEQUENCE="${SEED_SEQUENCE:-[42, 100, 777, 2026]}"  # 可选: 种子序列(YAML数组)
export OBSTACLE_DENSITY="${OBSTACLE_DENSITY:-0.0}"         # 可选: 障碍密度；0 表示无障碍
export WORLD_HALF_EXTENT="${WORLD_HALF_EXTENT:-7.0}"       # 可选: 世界半边长
export OBSTACLE_MODEL_PACKAGE="${OBSTACLE_MODEL_PACKAGE:-swarm_bringup}"  # 可选: 障碍模型来源包
export OBSTACLE_POOL_BOX="${OBSTACLE_POOL_BOX:-3}"         # 可选: 方形障碍池数量
export OBSTACLE_POOL_RECT="${OBSTACLE_POOL_RECT:-3}"       # 可选: 长方体障碍池数量
export OBSTACLE_POOL_CYLINDER="${OBSTACLE_POOL_CYLINDER:-3}"   # 可选: 圆柱障碍池数量
export OBSTACLE_STAGING_X="${OBSTACLE_STAGING_X:-20.0}"  # 可选: 障碍 staging 区 x
export OBSTACLE_STAGING_Y="${OBSTACLE_STAGING_Y:-20.0}"  # 可选: 障碍 staging 区 y
export OBSTACLE_STAGING_STRIDE="${OBSTACLE_STAGING_STRIDE:-1.5}" # 可选: 障碍 staging 间距
export OBSTACLE_SPAWN_ATTEMPTS="${OBSTACLE_SPAWN_ATTEMPTS:-400}" # 可选: 障碍布局采样尝试次数
export OBSTACLE_MIN_CLEARANCE="${OBSTACLE_MIN_CLEARANCE:-0.2}"   # 可选: 障碍间最小间距

# ---------- 21_agent_manager_node.sh ----------
export COLLISION_DISTANCE="${COLLISION_DISTANCE:-0.8}"     # 可选: 统一碰撞距离约束
export SPAWN_HALF_EXTENT="${SPAWN_HALF_EXTENT:-6.5}"       # 可选: 出生采样边界半径
export SPAWN_ATTEMPTS="${SPAWN_ATTEMPTS:-300}"             # 可选: 单个体采样最大尝试次数
export FIXED_Z="${FIXED_Z:-1.0}"                           # 可选: z 轴固定高度
export AGENT_ALTITUDE_STEP="${AGENT_ALTITUDE_STEP:-0.35}"  # 可选: 不同无人机高度间隔
export PAPER_SPAWN_LAYOUT="${PAPER_SPAWN_LAYOUT:-true}"    # 可选: pursuer 边缘、target 中心初始化
export TARGET_SPAWN_HALF_EXTENT="${TARGET_SPAWN_HALF_EXTENT:-2.0}" # 可选: target 中心采样半径
export PURSUER_EDGE_BAND="${PURSUER_EDGE_BAND:-2.0}"       # 可选: pursuer 边缘采样带宽
export IGNORE_SAME_ROLE_SPAWN_CLEARANCE="${IGNORE_SAME_ROLE_SPAWN_CLEARANCE:-true}" # 可选: 同角色不同高度，忽略平面间距约束
export AGENT_MODEL_PACKAGE="${AGENT_MODEL_PACKAGE:-swarm_bringup}"  # 可选: agent 模型来源包
export AGENT_STAGING_X="${AGENT_STAGING_X:-20.0}"        # 可选: agent staging 区 x
export AGENT_STAGING_Y="${AGENT_STAGING_Y:-25.0}"        # 可选: agent staging 区 y
export AGENT_STAGING_STRIDE="${AGENT_STAGING_STRIDE:-1.5}" # 可选: agent staging 间距

# ---------- 22_sim_bridge_node.sh ----------
export GAZEBO_CONTROLLER_ENABLED="${GAZEBO_CONTROLLER_ENABLED:-true}"  # 可选: 是否启用控制器
export GAZEBO_CONTROLLER_DT="${GAZEBO_CONTROLLER_DT:-0.02}"            # 可选: 控制积分周期(s)
export GAZEBO_MAX_SPEED="${GAZEBO_MAX_SPEED:-3.0}"                      # 可选: 速度上限
export GAZEBO_PURSUER_MAX_SPEED="${GAZEBO_PURSUER_MAX_SPEED:-2.5}"     # 可选: pursuer 速度上限，论文要求小于 target
export GAZEBO_EVADER_MAX_SPEED="${GAZEBO_EVADER_MAX_SPEED:-3.0}"       # 可选: evader/target 速度上限
export GAZEBO_LINEAR_DRAG="${GAZEBO_LINEAR_DRAG:-0.15}"                # 可选: 线性阻尼
export GAZEBO_CONTROL_REQUIRES_RESET="${GAZEBO_CONTROL_REQUIRES_RESET:-true}" # 可选: 首次 reset 前是否禁止控制回写

# ---------- 23_rl_policy_node.sh ----------
export PURSUER_ALGO="${PURSUER_ALGO:-cbc_tp_net}"        # 可选: cbc_tp_net/maddpg/flocking
export PURSUER_TRAIN_MODE="${PURSUER_TRAIN_MODE:-train_scratch}"  # 可选: train_scratch/train_load/eval_only
export PURSUER_WEIGHTS="${PURSUER_WEIGHTS:-}"             # 可选: 追捕者权重路径
export EVADER_ALGO="${EVADER_ALGO:-ddpg}"                 # 可选: ddpg
export EVADER_TRAIN_MODE="${EVADER_TRAIN_MODE:-train_scratch}"  # 可选: train_scratch/train_load/eval_only
export EVADER_WEIGHTS="${EVADER_WEIGHTS:-}"               # 可选: 逃避者权重路径
export MAX_ACCEL="${MAX_ACCEL:-1.0}"                      # 可选: 动作最大加速度
export PURSUER_MAX_ACCEL="${PURSUER_MAX_ACCEL:-1.2}"      # 可选: pursuer 最大加速度，论文要求大于 target
export EVADER_MAX_ACCEL="${EVADER_MAX_ACCEL:-0.8}"        # 可选: evader/target 最大加速度
export CAPTURE_RADIUS="${CAPTURE_RADIUS:-2.0}"            # 可选: 判定捕获半径
export CAPTURE_REQUIRED_PURSUERS="${CAPTURE_REQUIRED_PURSUERS:-3}" # 可选: 同时进入捕获半径的 pursuer 数
export COLLISION_PENALTY="${COLLISION_PENALTY:--10.0}"    # 可选: 碰撞终止惩罚
export DISCOUNT_GAMMA="${DISCOUNT_GAMMA:-0.95}"           # 可选: 折扣因子
export TARGET_TAU="${TARGET_TAU:-0.01}"                   # 可选: 软更新系数
export PURSUER_ACTOR_LR="${PURSUER_ACTOR_LR:-0.001}"      # 可选: 追捕者 Actor 学习率
export PURSUER_CRITIC_LR="${PURSUER_CRITIC_LR:-0.001}"    # 可选: 追捕者 Critic 学习率
export EVADER_ACTOR_LR="${EVADER_ACTOR_LR:-0.001}"        # 可选: 逃避者 Actor 学习率
export EVADER_CRITIC_LR="${EVADER_CRITIC_LR:-0.001}"      # 可选: 逃避者 Critic 学习率
export BUFFER_CAPACITY="${BUFFER_CAPACITY:-1000000}"      # 可选: replay 容量
export BATCH_SIZE="${BATCH_SIZE:-512}"                    # 可选: batch 大小
export WARMUP_STEPS="${WARMUP_STEPS:-500}"                # 可选: 热身步数
export TRAIN_UPDATES_PER_STEP="${TRAIN_UPDATES_PER_STEP:-1}"  # 可选: 每步更新次数
export TRAIN_EVERY_STEPS="${TRAIN_EVERY_STEPS:-50}"       # 可选: replay 新增多少步后更新一次
export EXPLORATION_NOISE_STD="${EXPLORATION_NOISE_STD:-0.2}"  # 可选: 探索噪声
export POLICY_UPDATE_INTERVAL="${POLICY_UPDATE_INTERVAL:-1}"  # 可选: 策略更新间隔
export TP_HISTORY_STEPS="${TP_HISTORY_STEPS:-10}"         # 可选: TP Net 历史步数 n
export TP_PREDICT_STEPS="${TP_PREDICT_STEPS:-3}"          # 可选: TP Net 预测步数 k
export TP_HIDDEN_DIM="${TP_HIDDEN_DIM:-128}"              # 可选: TP Net LSTM 隐藏维度
export TP_LR="${TP_LR:-0.001}"                            # 可选: TP Net 学习率
export CBC_HIDDEN_DIM="${CBC_HIDDEN_DIM:-128}"            # 可选: CBC 双向 LSTM 隐藏维度
export REWARD_SAFE_DISTANCE="${REWARD_SAFE_DISTANCE:-2.0}" # 可选: 障碍安全距离
export REWARD_NORMALIZATION="${REWARD_NORMALIZATION:-0.05}" # 可选: 奖励距离归一化系数
export OPTIMIZE_HORIZON_STEPS="${OPTIMIZE_HORIZON_STEPS:-500}"  # 可选: 时域参数
export CHECKPOINT_DIR="${CHECKPOINT_DIR:-artifacts/checkpoints}"  # 可选: 权重保存目录
export CHECKPOINT_PREFIX="${CHECKPOINT_PREFIX:-pursuer_policy}"     # 可选: 权重文件前缀
export METRICS_OUTPUT_DIR="${METRICS_OUTPUT_DIR:-artifacts/metrics}" # 可选: 论文指标输出目录
export METRICS_WINDOW_EPISODES="${METRICS_WINDOW_EPISODES:-100}" # 可选: 成功率统计窗口
export USE_SYNTHETIC_OBS_WHEN_MISSING="${USE_SYNTHETIC_OBS_WHEN_MISSING:-true}"  # 可选: 观测缺失时回退
export TORCH_DEVICE="${TORCH_DEVICE:-auto}"               # 可选: auto/cpu/cuda/cuda:0

# ---------- 24_training_coordinator_node.sh ----------
export MAX_EPISODES="${MAX_EPISODES:-4000}"               # 可选: 最大训练回合数
export MAX_STEPS="${MAX_STEPS:-500}"                      # 可选: 每回合最大步数
export RESET_POLICY="${RESET_POLICY:-2}"                  # 可选: 1固定重置/2安全随机/3重置世界再随机
export CAPTURE_STEPS="${CAPTURE_STEPS:-5}"                # 可选: 连续捕获步数阈值
export INITIAL_RESET_DELAY_SEC="${INITIAL_RESET_DELAY_SEC:-5.0}" # 可选: 启动后首次 reset 延迟
export ALWAYS_RESET_WORLD="${ALWAYS_RESET_WORLD:-false}"   # 可选: 每轮 done 后是否重置障碍；论文复现默认不换地图
