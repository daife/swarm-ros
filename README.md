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
6. swarm_bringup 统一启动、仿真桥接、模型注入、训练参数透传。

## 1. 逻辑闭环

系统遵循如下回合闭环，避免出生点卡墙并保证训练状态一致：

1. training_coordinator 判定 done（追捕成功、碰撞、步数超限）。
2. 若 reset_policy=3，则先发布 /scheduler/reset_world。
3. world_manager 完成障碍物刷新并发布 /world_manager/obstacles。
4. training_coordinator 发布 /scheduler/reset_agents（模式 1 或 2）。
5. agent_manager 基于障碍物边界 + safe_spawn_distance 安全采样并调用 SetEntityPose。
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
| safe_spawn_distance | float | 1.5 | 与障碍物最小缓冲距离 |
| spawn_half_extent | float | 9.0 | 采样区域半边长 |
| spawn_attempts | int | 300 | 最大采样尝试次数 |
| world_name | string | urban | Gazebo world 名，用于 set_pose 服务路径 |
| pursuer_name_prefix | string | pursuer | 追捕者模型名前缀 |
| evader_name_prefix | string | evader | 逃避者模型名前缀 |

话题与服务：

- Sub: /world_manager/obstacles
- Sub: /scheduler/reset_agents (std_msgs/Int8)
- Pub: /agent_manager/reset_poses (geometry_msgs/PoseArray)
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
| controller_dt | float | 0.05 | 控制积分周期 |
| max_speed | float | 3.0 | 速度上限 |
| linear_drag | float | 0.15 | 线性阻尼 |
| world_half_extent | float | 10.0 | 位置边界裁剪 |

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
| checkpoint_dir | string | artifacts/checkpoints | 权重输出目录 |
| checkpoint_prefix | string | pursuer_policy | 权重前缀 |
| use_synthetic_obs_when_missing | bool | true | 缺失观测时是否回退合成观测 |

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

指标消息 /rl/episode_metrics:

- [episode, step, done_reason, capture_streak, collision_events, collision_penalty_total]

## 3. Gazebo 资产

已提供如下文件：

- src/swarm_bringup/worlds/urban.world
- src/swarm_bringup/models/pursuer/model.sdf
- src/swarm_bringup/models/evader/model.sdf

SDF 已包含：

1. 无人机刚体模型（box 机体 + 惯量）。
2. OdometryPublisher 插件。
3. Contact sensor。

world 已包含：

1. Physics、Sensors、Contact 等系统插件。
2. 地面与固定建筑物（与 world_manager fixed 障碍布局一致）。
3. 6 架预置无人机实体（pursuer_0..4, evader_0）。

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
| safe_spawn_distance | 1.5 | 安全采样间距 |
| pursuer_algo | maddpg | 追捕算法 |
| pursuer_train_mode | train_scratch | 追捕训练模式 |
| pursuer_weights | "" | 追捕权重 |
| evader_algo | ddpg | 逃避算法 |
| evader_train_mode | eval_only | 逃避训练模式 |
| evader_weights | "" | 逃避权重 |
| checkpoint_dir | artifacts/checkpoints | 权重目录 |
| gazebo_controller_enabled | true | Gazebo运动控制是否启用 |
| gazebo_controller_dt | 0.05 | Gazebo控制积分周期 |
| gazebo_max_speed | 3.0 | Gazebo速度上限 |
| gazebo_linear_drag | 0.15 | Gazebo阻尼 |

## 5. 运行示例

### 5.1 Mock 后端训练

```bash
cd ~/swarm-ros
source ./env.sh
colcon build --base-paths src
source install/setup.bash
ros2 launch swarm_bringup training_launch.py \
  env_backend:=mock \
  max_episodes:=20 \
  max_steps:=100 \
  reset_policy:=2 \
  pursuer_algo:=maddpg \
  pursuer_train_mode:=train_scratch \
  evader_algo:=ddpg \
  evader_train_mode:=train_scratch \
  checkpoint_dir:=artifacts/checkpoints
```

### 5.2 Gazebo 后端训练（无图形）

```bash
cd ~/swarm-ros
source ./env.sh
colcon build --base-paths src
source install/setup.bash
ros2 launch swarm_bringup training_launch.py \
  env_backend:=gazebo \
  gazebo_gui:=false \
  max_episodes:=200 \
  max_steps:=120 \
  reset_policy:=2 \
  pursuer_algo:=maddpg \
  pursuer_train_mode:=train_load \
  pursuer_weights:=artifacts/checkpoints/pursuer_policy_best.pth \
  evader_algo:=ddpg \
  evader_train_mode:=eval_only \
  checkpoint_dir:=artifacts/checkpoints
```

### 5.3 Gazebo 后端训练（有图形）

```bash
ros2 launch swarm_bringup training_launch.py \
  env_backend:=gazebo \
  gazebo_gui:=true
```

## 6. Gazebo 链路验证（必须执行）

先启动 Gazebo 后端（见 5.2），再在另一个终端执行：

```bash
cd ~/swarm-ros
source ./env.sh
source install/setup.bash

ros2 topic list | grep -E "agent_0/odom|agent_0/collision|model/pursuer_0/odometry"
ros2 topic echo --no-daemon --once /agent_0/odom
ros2 topic echo --no-daemon --once /agent_0/collision
ros2 topic echo --no-daemon --once /model/pursuer_0/odometry
```

若需要主动验证碰撞，可将实体移动到建筑物附近后再观察 collision：

```bash
ros2 service call /world/urban/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: pursuer_0, type: 2}, pose: {position: {x: -5.0, y: -1.5, z: 1.0}}}"
ros2 topic echo --no-daemon --once /agent_0/collision
```

## 7. 参数覆盖核查结论

README 中列出的参数均已在对应节点声明并具备实际逻辑：

1. world_manager: 世界生成/随机种子/障碍发布均已实现。
2. agent_manager: 安全采样 + SetEntityPose 重置已实现。
3. rl_policy: MADDPG/DDPG 训练与权重读写已实现；cbc_tp_net 当前明确为未实现占位。
4. training_coordinator: done 判定与 reset 时序已实现，collision_penalty 已纳入指标统计。
5. launch: gazebo_gui 有图形/无图形模式切换已实现。
