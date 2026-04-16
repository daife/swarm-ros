# STAGE5 步骤书：Agent 启动入场机制重构

目标：统一 world_manager 与 agent_manager 的启动行为，均采用“先 staging，再 reset 入场”。

## 1. 方案定稿

1. 删除 launch 侧 agent 直接 spawn 逻辑。
2. 在 agent_manager 内实现 agent 模型初始化 spawn：
   - 启动时将 pursuer/evader 全部生成到世界外 staging 区。
   - 首次收到 reset_agents 后才进入训练区。
3. world_manager 保持同一原则：
   - 启动只完成障碍模型池 staging。
   - 仅在 reset_world 时激活障碍到训练区。
4. 维持 reset 流程：training_coordinator 首轮先 reset_world，再 reset_agents。

## 2. 验证标准

1. Gazebo 启动后可见 agent/obstacle 在边界外 staging。
2. 首次 reset 前训练区内无活动 agent 与障碍布局。
3. 首次 reset 后 agent/obstacle 正常进入训练区。

## 3. 执行清单

- [x] agent spawn 逻辑迁移到 agent_manager
- [x] launch 删除 agent 直接 spawn 逻辑
- [x] world_manager 启动阶段改为仅 staging
- [x] 保持 reset_world -> reset_agents 的时序兼容

## 4. 运行中修正

1. launch 删除 spawn 后，agent_manager 新增 create 服务调用与异步完成状态机。
2. 为避免 reset 早于 spawn 完成，agent_manager 将 reset 请求改为排队执行。
