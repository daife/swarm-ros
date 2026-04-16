# game_of_drones_rl ROS 2 仿真与训练系统说明

本系统用于多无人机追捕-逃避博弈训练，支持两种后端：

- Gazebo Harmonic 真仿真后端
- Mock 运动学后端（用于快速迭代训练）

核心目标：

1. world_manager 维护障碍物拓扑并广播。
2. agent_manager 基于障碍物做安全采样并重置无人机。
3. rl_policy 运行 MADDPG（追捕者）+ DDPG（逃避者）策略。
4. training_coordinator 负责 done 判定和回合重置时序。
5. sim_bridge 统一 Gazebo 话题，并把加速度指令落实到 Gazebo 实体运动。
6. swarm_bringup 统一启动、仿真桥接、训练参数透传，并存放 world/model 资产。

补充：

1. `bash/30_training_launch.sh` 与 `src/swarm_bringup/launch/training_launch.py` 参数一一对齐。
2. 手动分步脚本 `bash/*.sh` 与一键 launch 使用同一套默认参数源 `bash/env_defaults.sh`。

## 1. 逻辑闭环

系统遵循如下回合闭环，避免出生点卡墙并保证训练状态一致：

1. training_coordinator 判定 done（追捕成功、碰撞、步数超限）。
2. 每轮 reset 先发布 /scheduler/reset_world（只重置障碍位置，不重启 Gazebo）。
3. world_manager 完成障碍物刷新并发布 /world_manager/obstacles。
4. training_coordinator 延迟发布 /scheduler/reset_agents（模式 1 或 2）。
5. agent_manager 基于障碍物/边界/捕获半径 + collision_distance 采样并调用 SetEntityPose。
6. rl_policy 以 reset 消息作为回合边界，写入 replay 与 checkpoint。

## 2. 功能包与参数

### 2.1 world_manager

功能：维护固定/随机障碍物，发布可缓存障碍物列表。

参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| world_mode | string | random | fixed 或 random |
| seed_mode | string | none | single / sequence / none |
| seed_value | int | 42 | single 模式固定种子 |
| seed_sequence | int[] | [42,43,44,45] | sequence 模式种子序列 |
| obstacle_density | float | 0.3 | 随机障碍密度 |
| world_half_extent | float | 10.0 | 世界半边长 |
| world_name | string | urban | Gazebo world 名 |
| obstacle_model_package | string | swarm_bringup | 障碍模型来源包 |
| obstacle_pool_box | int | 8 | 方形障碍模型池数量 |
| obstacle_pool_rect | int | 8 | 长方体障碍模型池数量 |
| obstacle_pool_cylinder | int | 8 | 圆柱障碍模型池数量 |
| obstacle_spawn_attempts | int | 400 | 障碍布局采样尝试次数 |
| obstacle_min_clearance | float | 0.2 | 障碍最小间距 |
| obstacle_staging_x | float | 1000.0 | 障碍 staging 区 x |
| obstacle_staging_y | float | 1000.0 | 障碍 staging 区 y |
| obstacle_staging_stride | float | 1.5 | staging 间距 |

参数细化说明：

1. `world_mode`:
  - `fixed`: 使用固定布局，便于算法对比、回归测试。
  - `random`: 每次重置可变化，泛化更好，但训练波动会更大。
2. `seed_mode`:
  - `single`: 每轮都用同一个种子，适合复现实验。
  - `sequence`: 在 `seed_sequence` 中循环，兼顾稳定性与多样性。
  - `none`: 完全随机，探索性最强。
3. `seed_value` / `seed_sequence`:
  - 当你需要“可重复”结果时，优先固定这两个参数。
4. `obstacle_density`:
  - 典型可用范围 `0.1 ~ 0.5`。
  - 越大表示障碍越密，碰撞概率上升，训练初期可能更难收敛。
5. `obstacle_pool_*`:
  - 启动时不把障碍放入训练区，而是放到世界外 staging 区。
  - 若本轮密度所需数量超过池大小，world_manager 会自动扩池并生成新障碍。
6. `world_half_extent`:
  - 世界边界为 `[-world_half_extent, +world_half_extent]`。
  - 建议与 `spawn_half_extent`、`sim_bridge.world_half_extent` 保持一致或前者更小。

话题：

- Sub: /scheduler/reset_world (std_msgs/Empty)
- Pub: /world_manager/obstacles (visualization_msgs/MarkerArray, transient_local)

### 2.2 agent_manager

功能：缓存障碍物并执行安全重置；调用 Gazebo SetEntityPose 服务。

参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| num_pursuers | int | 5 | 追捕者数量 |
| num_evaders | int | 1 | 逃避者数量 |
| collision_distance | float | 0.8 | 统一碰撞距离约束 |
| capture_radius | float | 2.0 | reset 时追逃最小间隔约束 |
| world_half_extent | float | 10.0 | 重置采样边界 |
| spawn_half_extent | float | 9.0 | 采样区域半边长 |
| spawn_attempts | int | 300 | 最大采样尝试次数 |
| spawn_z | float | 1.0 | 重置高度 |
| world_name | string | urban | Gazebo world 名，用于 set_pose 服务路径 |
| pursuer_name_prefix | string | pursuer | 追捕者模型名前缀 |
| evader_name_prefix | string | evader | 逃避者模型名前缀 |
| agent_model_package | string | swarm_bringup | agent 模型来源包 |
| agent_staging_x | float | 1000.0 | agent staging 区 x |
| agent_staging_y | float | 1200.0 | agent staging 区 y |
| agent_staging_stride | float | 1.5 | agent staging 间距 |

参数细化说明：

1. `collision_distance`:
  - 用于 agent-agent、agent-obstacle、agent-boundary 的统一最小间隔。
  - 重置阶段碰撞不计分，训练阶段恢复正常计分。
2. `capture_radius`:
  - 采样时会额外保证 pursuer 和 evader 初始间距大于 `capture_radius + collision_distance`。
3. `spawn_half_extent`:
  - 推荐满足 `spawn_half_extent <= world_half_extent - 0.5`。
  - 太大时可能贴近边界，影响初始动作质量。
4. `spawn_attempts`:
  - 常用 `200 ~ 1000`。
  - 障碍密度高时建议增大，避免频繁触发原点回退。
5. `world_name`:
  - 必须与 Gazebo 实际 world 名一致，否则 `create/set_pose` 会失败。
6. `pursuer_name_prefix` / `evader_name_prefix`:
  - agent_manager 会按此前缀在启动时生成模型（如 `pursuer_0`、`evader_0`），初始位于 staging。

话题与服务：

- Sub: /world_manager/obstacles
- Sub: /scheduler/reset_agents (std_msgs/Int8)
- Pub: /agent_manager/reset_poses (geometry_msgs/PoseArray)
- Client: /world/<world_name>/create (ros_gz_interfaces/srv/SpawnEntity)
- Client: /world/<world_name>/set_pose (ros_gz_interfaces/srv/SetEntityPose)

### 2.3 sim_bridge

功能：

1. 把 Gazebo odom/contacts 转发为统一 ROS 语义话题。
2. 订阅 /agent_i/cmd_accel，积分速度并回写 Gazebo 实体位姿，实现 Gazebo 后端可训练闭环。

参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| num_pursuers | int | 5 | 追捕者数量 |
| num_evaders | int | 1 | 逃避者数量 |
| world_name | string | urban | Gazebo world 名 |
| controller_enabled | bool | true | 是否启用加速度控制器 |
| controller_dt | float | 0.02 | 控制积分周期 |
| max_speed | float | 3.0 | 速度上限 |
| pursuer_max_speed | float | -1.0 | 追捕者速度上限（<=0 回退 max_speed） |
| evader_max_speed | float | -1.0 | 逃避者速度上限（<=0 回退 max_speed） |
| linear_drag | float | 0.15 | 线性阻尼 |
| world_half_extent | float | 10.0 | 位置边界裁剪 |
| fixed_z | float | 1.0 | 固定高度 |
| control_requires_reset | bool | true | 首次 reset 前不执行控制回写 |

参数细化说明：

1. `controller_enabled`:
  - `true`: 使用加速度积分控制并回写 Gazebo 位姿（训练必需）。
  - `false`: 仅桥接，不做控制回写。
2. `controller_dt`:
  - 控制周期，常用 `0.02 ~ 0.1` 秒。
  - 越小越平滑，但计算频率更高。
3. `max_speed`:
  - 建议范围 `2.0 ~ 5.0`。
  - 过大可能导致追捕行为过于激进，训练不稳定。
4. `pursuer_max_speed` / `evader_max_speed`:
  - 可独立设置追捕者与逃避者最大速度。
  - 取值 <= 0 时回退 `max_speed`。
5. `linear_drag`:
  - 建议范围 `0.05 ~ 0.4`。
  - 较大时速度衰减快，轨迹更稳但机动性降低。
6. `world_half_extent`:
  - 控制器会对位置做边界裁剪，防止实体越界。
  - 应与环境边界一致，避免“看起来卡墙/瞬移”的现象。
7. `fixed_z`:
  - 控制层会强制 z 恒定，并将发布 odom 的 z 速度置 0。
8. `ground_plane` 接触过滤：
  - sim_bridge 会过滤与地面的接触，不将其计入训练碰撞事件。

话题：

- Sub: /model/<name>/odometry
- Sub: /model/<name>/contacts
- Sub: /world/<world>/model/<name>/link/base_link/sensor/contact_sensor/contact
- Sub: /agent_i/cmd_accel
- Sub: /agent_manager/reset_poses
- Pub: /agent_i/odom
- Pub: /agent_i/collision

### 2.4 mock_env

功能：无 Gazebo 时提供运动学环境，便于快速训练与回归。

参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| num_pursuers | int | 5 | 追捕者数量 |
| num_evaders | int | 1 | 逃避者数量 |
| dt | float | 0.05 | 积分周期 |
| world_half_extent | float | 10.0 | 世界边界 |
| max_speed | float | 3.0 | 速度上限 |
| linear_drag | float | 0.15 | 阻尼 |
| spawn_half_extent | float | 8.0 | 初始采样范围 |

参数细化说明：

1. `dt`:
  - 常用 `0.03 ~ 0.1`。
  - 过大时积分误差明显，过小时训练速度变慢。
2. `spawn_half_extent`:
  - 建议略小于 `world_half_extent`，给边界留缓冲。
3. `max_speed` 与 `linear_drag`:
  - 与 Gazebo 端尽量对齐，便于从 mock 迁移到真仿真。

### 2.5 rl_policy

功能：

1. 追捕者：MADDPG（多 Actor + 集中式 Critic）。
2. 逃避者：DDPG。
3. 支持 train_scratch / train_load / eval_only。
4. 支持 replay buffer、target 网络软更新、latest/best 权重保存。

算法说明：

- pursuer_algo 支持 maddpg，cbc_tp_net 当前仅占位（未实现网络结构）。
- evader_algo 当前支持 ddpg。

关键参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| num_pursuers | int | 5 | 追捕者数量 |
| num_evaders | int | 1 | 逃避者数量 |
| pursuer_algo | string | maddpg | 追捕者算法 |
| pursuer_train_mode | string | train_scratch | train_scratch / train_load / eval_only |
| pursuer_weights | string | "" | 追捕者权重路径 |
| evader_algo | string | ddpg | 逃避者算法 |
| evader_train_mode | string | eval_only | train_scratch / train_load / eval_only |
| evader_weights | string | "" | 逃避者权重路径 |
| max_accel | float | 1.0 | 输出加速度缩放 |
| pursuer_max_accel | float | -1.0 | 追捕者最大加速度（<=0 回退 max_accel） |
| evader_max_accel | float | -1.0 | 逃避者最大加速度（<=0 回退 max_accel） |
| capture_radius | float | 2.0 | 捕获判定半径 |
| collision_penalty | float | -2.0 | 碰撞惩罚 |
| discount_gamma | float | 0.99 | 折扣因子 |
| target_tau | float | 0.01 | 软更新系数 |
| pursuer_actor_lr | float | 3e-4 | 追捕者 Actor 学习率 |
| pursuer_critic_lr | float | 1e-3 | 追捕者 Critic 学习率 |
| evader_actor_lr | float | 3e-4 | 逃避者 Actor 学习率 |
| evader_critic_lr | float | 1e-3 | 逃避者 Critic 学习率 |
| buffer_capacity | int | 200000 | 回放池容量 |
| batch_size | int | 128 | 采样批大小 |
| warmup_steps | int | 500 | 起训前最少样本步数 |
| train_updates_per_step | int | 1 | 每步训练轮数 |
| exploration_noise_std | float | 0.2 | 探索噪声标准差 |
| policy_update_interval | int | 1 | 策略延迟更新步长 |
| optimize_horizon_steps | int | 120 | 优化时域参数 |
| checkpoint_dir | string | artifacts/checkpoints | 权重输出目录 |
| checkpoint_prefix | string | pursuer_policy | 权重前缀 |
| use_synthetic_obs_when_missing | bool | true | 缺失观测时是否回退合成观测 |

参数细化说明：

1. 训练模式参数：
  - `train_scratch`: 从零训练，不加载旧权重。
  - `train_load`: 读取已有权重后继续训练（fine-tune）。
  - `eval_only`: 仅推理，不更新参数。
2. 学习率参数（`*_actor_lr`, `*_critic_lr`）：
  - 常用级别 `1e-4 ~ 1e-3`。
  - 过大易震荡，过小收敛慢。
3. `discount_gamma`:
  - 常用 `0.95 ~ 0.995`。
  - 越接近 `1.0` 越重视长期回报。
4. `target_tau`:
  - 常用 `0.005 ~ 0.02`。
  - 越小越稳定，但目标网络更新更慢。
5. `buffer_capacity` 与 `batch_size`:
  - 容量大有助于去相关，但占内存更高。
  - `batch_size` 常用 `64/128/256`，越大梯度更平滑但更慢。
6. `warmup_steps`:
  - 训练前仅收集样本的步数。
  - 环境难度高时可适当增大（如 `1000+`）。
7. `train_updates_per_step`:
  - 每个环境步触发的参数更新次数。
  - 大于 `1` 可加快学习，但也可能过拟合近期样本。
8. `exploration_noise_std`:
  - 常用 `0.1 ~ 0.3`。
  - 太小探索不足，太大动作抖动严重。
9. `max_accel`:
  - 动作尺度上限，应与控制器和场景大小匹配。
10. `pursuer_max_accel` / `evader_max_accel`:
  - 允许追捕者和逃避者使用不同机动能力上限。
11. `capture_radius` 与 `collision_penalty`:
  - 前者影响 done 判定难度，后者影响风险规避倾向。
12. `use_synthetic_obs_when_missing`:
  - Gazebo 启动早期观测未齐时，`true` 能降低训练中断风险。

checkpoint 产物：

- <prefix>_latest.pth
- <prefix>_best.pth
- <prefix>_evader_best.pth

### 2.6 training_coordinator

功能：episode/step 调度、done 判定、重置时序下发、指标发布。

参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| num_pursuers | int | 5 | 追捕者数量 |
| num_evaders | int | 1 | 逃避者数量 |
| max_episodes | int | 4000 | 最大回合数 |
| max_steps | int | 500 | 每回合最大步数 |
| reset_policy | int | 1 | 1 固定恢复; 2 安全随机; 3 先重置世界再随机 |
| collision_penalty | float | -2.0 | 碰撞罚分累计到指标 |
| capture_radius | float | 2.0 | 追捕半径 |
| capture_steps | int | 5 | 连续满足追捕步数 |
| initial_reset_delay_sec | float | 5.0 | 启动后首次 reset 延迟 |
| reset_agents_delay_sec | float | 0.4 | world reset 到 agent reset 的延迟 |
| reset_grace_steps | int | 12 | reset 后碰撞忽略步数 |
| always_reset_world | bool | true | 每轮 done 后都刷新障碍 |

说明：一键 launch 的默认 `max_episodes=60`、`max_steps=120`，会覆盖节点内部默认值。

参数细化说明：

1. `max_episodes`:
  - 整体训练预算，建议先小规模验证（如 `20~100`），再扩到 `1000+`。
2. `max_steps`:
  - 单回合时长上限。
  - 过小会频繁超时，过大则训练单回合成本高。
3. `reset_policy`:
  - `1`: 固定重置到初始点，训练最稳定。
  - `2`: 安全随机重置，泛化更好（推荐默认）。
  - `3`: 先重置世界再随机，难度最高、适合后期增强。
4. `initial_reset_delay_sec` / `reset_agents_delay_sec`:
  - 保证“先 spawn，再 reset，再训练”，减少初始化竞争条件。
5. `reset_grace_steps`:
  - reset 后短窗口内碰撞不计分，避免重置瞬态干扰训练指标。
6. `capture_radius` 与 `capture_steps`:
  - 两者共同决定捕获成功门槛。
  - 半径更大、连续步数更小会更容易判定捕获。
7. `collision_penalty`:
  - 该值越负，策略越倾向规避碰撞。

指标消息 /rl/episode_metrics:

- [episode, step, done_reason, capture_streak, collision_events, collision_penalty_total]

训练指标输出详解：

| 下标 | 字段名 | 示例值 | 含义 | 典型范围/判读 |
| --- | --- | --- | --- | --- |
| 0 | episode | 128 | 当前回合序号（从 0 开始） | 递增到 `max_episodes` |
| 1 | step | 97 | 当前回合结束时的步数 | `1 ~ max_steps` |
| 2 | done_reason | 1 | 回合结束原因 | `1=捕获`, `2=碰撞`, `3=超时` |
| 3 | capture_streak | 5 | 结束时连续满足捕获条件的步数 | 若常达 `capture_steps`，说明捕获效率在提升 |
| 4 | collision_events | 2 | 该回合累计碰撞事件数 | 趋势下降通常表示避障能力提升 |
| 5 | collision_penalty_total | -4.0 | 该回合碰撞惩罚累计值 | 约等于 `collision_events * collision_penalty` |

补充说明：

1. `done_reason` 分布是最直接的训练健康度信号。
2. 如果长期大量 `done_reason=2`，优先调大 `collision_distance` 或提高碰撞惩罚绝对值。
3. 如果长期大量 `done_reason=3`，可适当增大 `max_steps`，或提高探索噪声与动作上限。
4. 建议同时观察 `rl_policy_node` 日志中的 `pursuer_return`、`evader_return`、`replay`，与本话题联合判断收敛。

## 3. Gazebo 资产

已提供如下文件：

- src/swarm_bringup/worlds/urban.world
- src/swarm_bringup/models/pursuer/model.sdf
- src/swarm_bringup/models/evader/model.sdf
- src/swarm_bringup/models/obstacle_box/model.sdf
- src/swarm_bringup/models/obstacle_rect/model.sdf
- src/swarm_bringup/models/obstacle_cylinder/model.sdf

SDF 已包含：

1. 无人机刚体模型（box 机体 + 惯量）。
2. OdometryPublisher 插件。
3. Contact sensor。
4. 已禁用重力（适配 2D 控制与固定高度训练）。

world 已包含：

1. Physics、Sensors、Contact 等系统插件。
2. 地面与边界墙（训练边界）。
3. 无预置无人机实体；训练启动时由 agent_manager 动态生成到 staging（pursuer_0..4, evader_0）。
4. 无预置障碍物；障碍由 world_manager 在 reset 时从模型池移入训练区。

## 4. Launch 参数

入口：src/swarm_bringup/launch/training_launch.py

关键参数：

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| env_backend | mock | mock 或 gazebo |
| gazebo_gui | false | true 开图形界面，false 无图形 |
| world_name | urban | Gazebo world 名 |
| world_sdf | 内置 urban.world | Gazebo world 文件路径 |
| num_pursuers | 5 | 追捕者数量 |
| num_evaders | 1 | 逃避者数量 |
| max_episodes | 60 | 回合数 |
| max_steps | 120 | 每回合步数 |
| reset_policy | 2 | 重置策略 |
| world_mode | random | 世界布局模式 |
| seed_mode | none | 随机种子模式 |
| seed_value | 42 | 固定种子 |
| obstacle_density | 0.3 | 障碍密度 |
| collision_distance | 0.8 | 碰撞约束采样间距 |
| pursuer_algo | maddpg | 追捕算法 |
| pursuer_train_mode | train_scratch | 追捕训练模式 |
| pursuer_weights | "" | 追捕权重 |
| evader_algo | ddpg | 逃避算法 |
| evader_train_mode | eval_only | 逃避训练模式 |
| evader_weights | "" | 逃避权重 |
| max_accel | 1.0 | 全体基础最大加速度 |
| agent_staging_x | 1000.0 | agent staging 区 x |
| agent_staging_y | 1200.0 | agent staging 区 y |
| agent_staging_stride | 1.5 | agent staging 间距 |
| obstacle_model_package | swarm_bringup | 障碍模型来源包 |
| obstacle_spawn_attempts | 400 | 障碍布局采样尝试次数 |
| obstacle_min_clearance | 0.2 | 障碍最小间距 |
| spawn_half_extent | 9.0 | agent 重置采样半径 |
| spawn_attempts | 300 | agent 重置采样尝试次数 |
| checkpoint_prefix | pursuer_policy | 权重名前缀 |
| pursuer_max_accel | -1.0 | 追捕者最大加速度（<=0 回退 max_accel） |
| evader_max_accel | -1.0 | 逃避者最大加速度（<=0 回退 max_accel） |
| checkpoint_dir | artifacts/checkpoints | 权重目录 |
| gazebo_controller_enabled | true | Gazebo运动控制是否启用 |
| gazebo_controller_dt | 0.02 | Gazebo控制积分周期 |
| gazebo_max_speed | 3.0 | Gazebo速度上限 |
| gazebo_pursuer_max_speed | -1.0 | 追捕者速度上限（<=0 回退 gazebo_max_speed） |
| gazebo_evader_max_speed | -1.0 | 逃避者速度上限（<=0 回退 gazebo_max_speed） |
| gazebo_linear_drag | 0.15 | Gazebo阻尼 |
| fixed_z | 1.0 | 固定高度 |
| initial_reset_delay_sec | 5.0 | 启动后首次 reset 延迟 |
| reset_agents_delay_sec | 0.4 | world reset 到 agent reset 延迟 |
| reset_grace_steps | 12 | reset 后碰撞忽略步数 |
| mock_dt | 0.05 | mock 后端积分步长 |
| mock_max_speed | 3.0 | mock 后端速度上限 |
| mock_linear_drag | 0.15 | mock 后端阻尼 |
| mock_spawn_half_extent | 8.0 | mock 初始采样半径 |

Launch 参数细化说明：

1. `env_backend`:
  - `mock`: 纯运动学，训练快，适合算法迭代。
  - `gazebo`: 真仿真，结果更贴近真实动力学。
2. `gazebo_gui`:
  - `true` 方便观察行为与调试。
  - `false` 适合长时训练与服务器运行。
3. `pursuer_*` 与 `evader_*` 参数：
  - 两组参数可独立设置，支持“一方训练一方冻结”。
4. `checkpoint_dir`:
  - 建议为实验分目录，如 `artifacts/checkpoints/exp_20260415`，避免覆盖。
5. `gazebo_controller_*`:
  - 这组参数直接透传给 `sim_bridge` 控制器，建议与 mock 端动力学参数保持一致。


