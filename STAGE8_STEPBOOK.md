# STAGE8 步骤书：文档同步与验收收口

目标：将新行为完整落到 README 与步骤书，形成可执行、可复核的闭环。

## 1. 方案定稿

1. README 对齐以下事实：
   - agent/obstacle 启动先 staging，首次 reset 后入场。
   - agent_manager 持有 create + set_pose 双服务职责。
   - sim_bridge 的 z 轴稳定化默认参数已调整。
2. 更新步骤书状态，记录本轮运行中修正。
3. 执行静态错误检查与构建验证。

## 2. 验证标准

1. README 参数默认值与代码一致。
2. STAGE5~8 清晰描述方案、执行与修正。
3. get_errors 与 colcon build 均通过。

## 3. 执行清单

- [x] 更新 README 关键章节
- [x] 回填 STAGE5~8 完成状态
- [x] 完成错误检查
- [x] 完成 colcon build 验证

## 4. 运行中修正

1. 已完成 `colcon build --base-paths src`，7 个包全部通过。
2. 已完成 `env_backend:=mock` 冒烟启动，首轮 reset 与回合推进正常。
