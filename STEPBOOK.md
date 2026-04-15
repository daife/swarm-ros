# swarm-ros 详细步骤书（按 README 执行并落地）

本文档将 README 的目标拆成可执行步骤，并给出本仓库当前已完成状态。

## 0. 前置条件

1. 已按 environment.md 完成 ROS 2 Jazzy、Gazebo Harmonic、venv、依赖安装。
2. 在仓库根目录执行：

```bash
source ./env.sh
```

## 1. 目标拆解（对应 README 四大模块）

1. world_manager：管理世界/障碍物，发布 `/world_manager/obstacles`，订阅 `/scheduler/reset_world`。
2. agent_manager：缓存障碍物，订阅 `/scheduler/reset_agents`，执行安全重置逻辑入口。
3. rl_policy：订阅 `/agent_{i}/odom`，发布 `/agent_{i}/cmd_accel`。
4. training_coordinator：负责 episode/step，发布重置指令。
5. swarm_bringup：统一 launch，串联以上四个模块。

## 2. 代码落地步骤（本次已完成）

### 步骤 2.1 创建功能包

已在 `src/` 新增包：

- `world_manager`
- `agent_manager`
- `rl_policy`
- `training_coordinator`
- `swarm_bringup`

### 步骤 2.2 实现节点骨架与参数

已实现并对齐 README 参数与话题命名：

1. `world_manager/world_manager_node.py`
2. `agent_manager/agent_manager_node.py`
3. `rl_policy/rl_policy_node.py`
4. `training_coordinator/training_coordinator_node.py`

### 步骤 2.3 实现统一启动

已新增：

- `swarm_bringup/launch/training_launch.py`

默认参数已覆盖 README 运行示例中的关键字段（`reset_policy`、`safe_spawn_distance`、训练模式参数等）。

## 3. 构建与验证步骤（本次已执行）

### 步骤 3.1 识别包

```bash
source ./env.sh
colcon list --base-paths src
```

结果：识别到 6 个包（含原有 `demo_pubsub`）。

### 步骤 3.2 全量构建

```bash
source ./env.sh
colcon build --base-paths src
```

结果：6 个包构建完成。

说明：`demo_pubsub` 仍会出现 `tests_require` 的 setuptools 警告，不影响本次功能包运行。

### 步骤 3.3 启动联调

```bash
source ./env.sh
source install/setup.bash
ros2 launch swarm_bringup training_launch.py
```

结果：四个核心节点均成功拉起并输出日志，`world_manager_node` 已发布障碍物。

## 4. 日常运行手册

### 4.1 重新构建

```bash
cd ~/swarm-ros
source ./env.sh
colcon build --base-paths src
```

### 4.2 启动系统

```bash
cd ~/swarm-ros
source ./env.sh
source install/setup.bash
ros2 launch swarm_bringup training_launch.py
```

### 4.3 单节点调试

```bash
source ./env.sh
source install/setup.bash
ros2 run world_manager world_manager_node
ros2 run agent_manager agent_manager_node
ros2 run rl_policy rl_policy_node
ros2 run training_coordinator training_coordinator_node
```

## 5. 当前完成度说明

1. 已完成 README 级别的软件架构骨架、参数接口、重置话题联动、统一启动与可构建可运行状态。
2. 尚未接入 Gazebo 实体操作、SDF 传感器、`ros_gz_bridge`、真实 DRL 训练循环（PyTorch 权重读写与 reward/backprop）。
3. 下一阶段可在现有骨架上逐步替换 stub 逻辑为真实仿真与训练逻辑。

## 6. 下一阶段完成说明

第二阶段已完成并记录在 `NEXT_STAGE_STEPBOOK.md`，覆盖了 Gazebo 桥接、安全出生点采样、SetEntityPose 重置、追捕/碰撞 Done 判定和训练指标发布。
