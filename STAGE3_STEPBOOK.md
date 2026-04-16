# STAGE3 步骤书：Agent 重置约束与首轮训练门控

目标：保证 reset 流程严格、可复现，且重置期碰撞不污染训练指标。

## 1. 代码改造点

1. agent_manager 重构采样约束：
   - 新参数 `collision_distance`。
   - 约束 agent-agent 最小间距。
   - 约束 agent-obstacle 表面间距。
   - 约束 agent-boundary 间距。
   - 约束 pursuer 与 evader 初始距离 > capture_radius。
2. training_coordinator 增加首轮 reset 流程：
   - 启动后先发布 reset_world + reset_agents，再开始 episode 计步。
3. training_coordinator 增加 reset 宽限期：
   - 宽限窗口内碰撞不计分，不触发 done。

## 2. 验证标准

1. 第一轮训练开始前日志可见 reset 触发。
2. reset 宽限期内碰撞不计入 metrics。
3. 训练仍可在 done 后进入 reset -> 下一轮。

## 3. 执行清单

- [x] 改造 agent_manager 采样器
- [x] 改造 training_coordinator 首轮 reset
- [x] 改造 training_coordinator reset 宽限期
- [x] 完成 mock 与 Gazebo 双后端验证

## 4. 运行中修正

1. 首版首轮 reset 时序过早（agent 还在 spawn），引发 set_pose 失败。
2. 已加入 `initial_reset_delay_sec` 与 `reset_agents_delay_sec`，确保先 spawn 再 reset。
