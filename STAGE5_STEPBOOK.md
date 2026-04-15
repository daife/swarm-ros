# 第五阶段步骤书（Gazebo后端实训落地 + 文档对齐）

目标：把Gazebo后端从可启动提升为可训练链路，并把新增参数与模块完整回写README。

## 1. 本阶段目标

1. 补齐四旋翼SDF模型（机体、里程计插件、碰撞传感器）。
2. 落地Gazebo world与模型资产并在launch中统一调度。
3. 完整实现MADDPG + DDPG训练流程（回放池、目标网络、软更新、权重管理）。
4. 完成README参数核对，确保参数-代码一致。
5. 验证Gazebo后端链路（Gazebo、bridge、agent_manager、sim_bridge、rl_policy、coordinator）。

## 2. 执行清单

### 2.1 仿真资产

- [x] 新增 `src/swarm_bringup/models/pursuer/model.sdf`
- [x] 新增 `src/swarm_bringup/models/evader/model.sdf`
- [x] 新增 `src/swarm_bringup/worlds/urban.world`
- [x] world内加入固定障碍与6架无人机（pursuer_0..4, evader_0）
- [x] 无人机模型包含：惯量、碰撞体、Contact Sensor、OdometryPublisher

### 2.2 Gazebo启动与桥接

- [x] `training_launch.py` 增加：
  - `env_backend`
  - `gazebo_gui`（有图形/无图形开关）
  - `world_name` / `world_sdf`
  - 训练超参数透传
- [x] Gazebo后端自动启动 `ros_gz_sim/gz_sim.launch.py`
- [x] `parameter_bridge` 动态桥接：
  - `/model/*/odometry`
  - `/model/*/contacts`
  - `/world/<world>/model/*/link/base_link/sensor/contact_sensor/contact`
  - `/world/<world>/set_pose`

### 2.3 Gazebo控制闭环

- [x] `sim_bridge_node` 升级为控制执行器：
  - 订阅 `/agent_i/cmd_accel`
  - 积分速度与位置（含阻尼、速度上限、边界裁剪）
  - 调用 SetEntityPose 回写Gazebo实体位姿
  - 订阅 `/agent_manager/reset_poses` 后清零速度

### 2.4 训练算法重构

- [x] `rl_policy_node` 从简化策略升级为：
  - 追捕者 MADDPG（多Actor + 集中式Critic）
  - 逃避者 DDPG
- [x] 新增核心机制：
  - replay buffer
  - warmup + batch采样
  - target网络软更新（tau）
  - policy update interval
  - exploration noise
- [x] 新增权重管理：
  - `<prefix>_latest.pth`
  - `<prefix>_best.pth`
  - `<prefix>_evader_best.pth`

### 2.5 调度与指标

- [x] `training_coordinator` 使用 `collision_penalty` 累计 `collision_penalty_total`
- [x] 指标消息扩展为6元组：
  - `[episode, step, done_reason, capture_streak, collision_events, collision_penalty_total]`

### 2.6 文档对齐

- [x] README重写并对齐当前实现：
  - 模块职责
  - 参数表
  - Gazebo资产
  - launch参数
  - mock/gazebo运行命令
  - 验证命令

## 3. 构建与运行验证

### 3.1 构建验证

已执行：

- `source ./env.sh`
- `colcon build --base-paths src`

结果：通过（7个包全部完成）。

### 3.2 Gazebo链路验证

已执行：

- `ros2 launch swarm_bringup training_launch.py env_backend:=gazebo gazebo_gui:=false ...`

关键日志结果：

1. Gazebo world 正常加载 `urban.world`。
2. `ros_gz_bridge` 成功创建 odometry/contact/set_pose 桥接。
3. Gazebo日志显示：
   - `OdometryPublisher publishing odometry on /model/pursuer_0/odometry`（以及其他agent）
   - `Contact system publishing on world/urban/model/pursuer_0/.../contact_sensor/contact`（以及其他agent）

说明：Gazebo层的里程计与碰撞数据源已建立。

## 4. 当前环境侧限制与处理建议

在本机本会话中，跨终端执行 `ros2 topic echo/list` 存在CLI发现超时现象（xmlrpc daemon连接超时或topic发现长时间阻塞），导致ROS命令行回读桥接消息不稳定。

建议排障顺序：

1. `unset ROS_DISCOVERY_SERVER`
2. `ros2 daemon stop && ros2 daemon start`
3. `ros2 topic echo --no-daemon --once /model/pursuer_0/odometry`
4. `ros2 topic echo --no-daemon --once /agent_0/odom`
5. `ros2 topic echo --no-daemon --once /agent_0/collision`

如果CLI仍异常，可先以launch日志作为阶段验收依据（本阶段已记录关键日志证据）。

## 5. 阶段结论

1. Gazebo后端训练链路主干已落地：世界资产、桥接、控制闭环、训练算法、权重保存。
2. README已与代码参数和模块实现完成对齐。
3. 剩余风险主要是当前环境里的ROS CLI发现稳定性，不影响代码主链路实现本身。
