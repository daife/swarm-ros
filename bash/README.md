# 多终端手动启动与参数配置

本目录提供两种等效入口：

1. 一键 launch 包装脚本：`./bash/30_training_launch.sh [配置文件]`
2. 手动多终端分步脚本：`./bash/*.sh [配置文件]`

两者共享同一套参数默认值与配置文件体系。

## 0. 先修改参数

统一参数默认值集中在同目录的 `env_defaults.sh`。

所有脚本都支持可选配置文件参数：

```bash
./bash/23_rl_policy_node.sh
./bash/23_rl_policy_node.sh bash/configs/train_pursuer_only.env.sh
```

如果不传参数，默认加载 `bash/env_defaults.sh`。

你通常只需要改以下变量：

1. `NUM_PURSUERS` / `NUM_EVADERS`
2. `PURSUER_TRAIN_MODE` / `EVADER_TRAIN_MODE`
3. `PURSUER_WEIGHTS` / `EVADER_WEIGHTS`
4. `MAX_EPISODES` / `MAX_STEPS`
5. `CHECKPOINT_DIR`
6. `TORCH_DEVICE`（`auto/cpu/cuda/cuda:0`）

边界参数注意：

1. `WORLD_HALF_EXTENT` 需要与 world 文件中的实际围墙一致（`urban.world` 当前建议保持 `7.0`）。
2. `AGENT_STAGING_X/Y`、`OBSTACLE_STAGING_X/Y` 建议放在边界外，避免与训练区混淆。

## 1. 一键启动（推荐）

```bash
cd ~/swarm-ros
./bash/30_training_launch.sh
```

带预设配置：

```bash
./bash/30_training_launch.sh bash/configs/train_from_scratch.env.sh
./bash/30_training_launch.sh bash/configs/train_from_best.env.sh
./bash/30_training_launch.sh bash/configs/train_pursuer_only.env.sh
./bash/30_training_launch.sh bash/configs/train_evader_only.env.sh
./bash/30_training_launch.sh bash/configs/eval_only.env.sh
```

## 2. 手动分步启动（调试）

终端 T1：

```bash
cd ~/swarm-ros
./bash/10_gazebo_gui.sh
```

终端 T2：

```bash
cd ~/swarm-ros
./bash/11_parameter_bridge.sh
```

说明：agent 现在由 `agent_manager` 自动生成到 staging 区。

## 3. 训练节点

终端 T4：

```bash
cd ~/swarm-ros
./bash/20_world_manager_node.sh
```

终端 T5：

```bash
cd ~/swarm-ros
./bash/21_agent_manager_node.sh
```

终端 T6（Gazebo 后端）：

```bash
cd ~/swarm-ros
./bash/22_sim_bridge_node.sh
```

终端 T7：

```bash
cd ~/swarm-ros
./bash/23_rl_policy_node.sh
```

终端 T8：

```bash
cd ~/swarm-ros
./bash/24_training_coordinator_node.sh
```

## 4. 快速停止

每个终端 `Ctrl+C` 即可；建议先停 `24_training_coordinator_node.sh`，再停 `23_rl_policy_node.sh`，最后停仿真相关进程。

## 5. 终端手动发布控制命令

以下命令可在任意已 `source env.sh && source install/setup.bash` 的终端执行。

手动刷新世界（触发 world_manager 重采样/重发布障碍）：

```bash
ros2 topic pub --once /scheduler/reset_world std_msgs/msg/Int32 "{data: 1}"
```

手动重置 agent 位置（触发 agent_manager 重置，data=[reset_id, mode]）：

```bash
# mode=1: 重置到初始位置
ros2 topic pub --once /scheduler/reset_agents_cmd std_msgs/msg/Int32MultiArray "{data: [1, 1]}"

# mode=2: 安全随机重置（推荐）
ros2 topic pub --once /scheduler/reset_agents_cmd std_msgs/msg/Int32MultiArray "{data: [2, 2]}"
```

说明：训练时由 training_coordinator 自动执行 reset_id 三方握手（world_done、agent_done、policy_ready）并发布 /scheduler/episode_start。

快速查看命令是否生效：

```bash
ros2 topic echo --once /agent_manager/reset_poses
```
