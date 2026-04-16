# 多终端手动启动（调试用途）

本目录仅用于调试节点，不是推荐训练入口。推荐入口是 `ros2 launch swarm_bringup training_launch.py`。

## 0. 先修改参数

统一参数集中在同目录的 `env_defaults.sh`。

你通常只需要改以下变量：

1. `NUM_PURSUERS` / `NUM_EVADERS`
2. `PURSUER_TRAIN_MODE` / `EVADER_TRAIN_MODE`
3. `PURSUER_WEIGHTS` / `EVADER_WEIGHTS`
4. `MAX_EPISODES` / `MAX_STEPS`
5. `CHECKPOINT_DIR`

## 1. Gazebo + 桥接

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

## 2. 训练节点

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

## 3. Mock 后端替换说明

如果不用 Gazebo，而是 mock 环境：

1. 不启动 `10_gazebo_gui.sh`、`11_parameter_bridge.sh`
2. 用 `22_mock_env_node.sh` 替换 `22_sim_bridge_node.sh`

## 4. 快速停止

每个终端 `Ctrl+C` 即可；建议先停 `24_training_coordinator_node.sh`，再停 `23_rl_policy_node.sh`，最后停仿真相关进程。

## 5. 终端手动发布控制命令

以下命令可在任意已 `source env.sh && source install/setup.bash` 的终端执行。

手动刷新世界（触发 world_manager 重采样/重发布障碍）：

```bash
ros2 topic pub --once /scheduler/reset_world std_msgs/msg/Empty "{}"
```

手动重置 agent 位置（触发 agent_manager 重置）：

```bash
# mode=1: 重置到初始位置
ros2 topic pub --once /scheduler/reset_agents std_msgs/msg/Int8 "{data: 1}"

# mode=2: 安全随机重置（推荐）
ros2 topic pub --once /scheduler/reset_agents std_msgs/msg/Int8 "{data: 2}"
```

说明：当前 mode=2 使用 collision_distance 约束采样，同时约束边界、障碍和追逃初始距离。

快速查看命令是否生效：

```bash
ros2 topic echo --once /agent_manager/reset_poses
```
