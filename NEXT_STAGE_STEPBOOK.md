# swarm-ros 下一阶段步骤书（结合 README 落地）

本阶段目标：把第一阶段的骨架升级为可用闭环，覆盖 README 第 1~5 节的关键能力。

## A. 阶段目标

1. 实现世界重置后障碍物稳定广播（支持随机/固定/种子序列）。
2. 实现智能体安全出生点采样，并通过 Gazebo SetEntityPose 服务执行重置。
3. 实现 Gazebo 到 ROS 侧的里程计和碰撞桥接，统一为 `/agent_{i}` 语义话题。
4. 实现训练调度 Done 判定：步数上限、碰撞触发、追捕持续步数触发。
5. 支持策略 train_mode 与权重加载前置校验。
6. 完成构建、启动和基础话题联调验证。

## B. 逐步执行清单（本次执行）

### B1. 新增桥接包 `sim_bridge`

- [x] 新增 Python 包与入口节点 `sim_bridge_node`。
- [x] 订阅 `/model/*/odometry` 与 `/model/*/contacts`。
- [x] 发布 `/agent_{i}/odom` 与 `/agent_{i}/collision`。

### B2. 升级 `world_manager`

- [x] 新增参数：`seed_sequence`、`world_half_extent`。
- [x] 支持 `seed_mode=sequence`。
- [x] 支持 `world_mode=fixed` 固定障碍拓扑。
- [x] `/world_manager/obstacles` 使用 transient local QoS，保障晚到订阅者可拿到障碍物。

### B3. 升级 `agent_manager`

- [x] 实现障碍物约束下的安全位置采样。
- [x] 支持重置模式：`1` 恢复初始位姿，`2` 重新安全采样。
- [x] 对接 `/world/urban/set_pose`（`ros_gz_interfaces/srv/SetEntityPose`）服务。
- [x] 发布 `/agent_manager/reset_poses` 调试信息。

### B4. 升级 `training_coordinator`

- [x] 订阅 `/agent_{i}/odom` 与 `/agent_{i}/collision`。
- [x] 实现追捕判定（`capture_radius` + `capture_steps`）。
- [x] 实现碰撞判定（碰撞即 Done）。
- [x] 发布 `/rl/episode_metrics` 指标。

### B5. 升级 `rl_policy`

- [x] 校验 train_mode 合法性。
- [x] 对 `train_load`/`eval_only` 检查权重路径与 PyTorch 可用性。
- [x] 增加基础追捕-逃逸控制输出（作为可运行默认策略）。

### B6. 升级 `training_launch.py`

- [x] 增加 `ros_gz_bridge parameter_bridge` 启动。
- [x] 增加 `sim_bridge_node` 启动。
- [x] 参数贯通：`num_pursuers`/`num_evaders` 到各节点。

### B7. 验证

- [x] `colcon list --base-paths src` 能识别全部包。
- [x] `colcon build --base-paths src` 构建通过。
- [x] `ros2 launch swarm_bringup training_launch.py` 启动通过。
- [x] 从启动日志验证关键链路已生效：`/world_manager/obstacles` 发布、`/scheduler/reset_agents` 周期触发、`/rl/episode_metrics` 发布器创建。

## C. 运行命令

```bash
cd ~/swarm-ros
source ./env.sh
colcon list --base-paths src
colcon build --base-paths src
source install/setup.bash
ros2 launch swarm_bringup training_launch.py
```

## D. 完成判定

满足以下条件即视为本项目在 README 范围内完成：

1. 四模块 + 桥接模块构建通过。
2. 启动后可观察到世界障碍发布与重置链路日志。
3. 训练调度可发布 episode 指标并驱动重置指令。
4. Gazebo 连接存在时，可通过 SetEntityPose 触发实体重置；Gazebo 未连接时系统保持可运行并给出降级日志。
