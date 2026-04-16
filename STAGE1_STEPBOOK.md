# STAGE1 步骤书：需求澄清与方案落地

目标：把 TODO 中“不确定/不明白”的点全部收敛成可执行技术方案。

## 1. 关键结论（已定稿）

1. 世界不重启：`/scheduler/reset_world` 只触发障碍物重定位，不重启 Gazebo。
2. 世界模型简化：`urban.world` 保持地面与边界语义，不预置障碍物和无人机。
3. 障碍物采用模型池：
   - 预置三类 SDF：圆柱、正方体、长方体。
   - 启动时统一生成到 staging 区（世界外，默认 x=1000, y=1000）。
   - 重置时将启用障碍移入世界内，未启用障碍移回 staging 区。
4. 若障碍池不足：world_manager 通过 `/world/<world>/create` 追加生成。
5. 训练前强制一次 reset：
   - 首回合前必做“障碍+agent 重置”。
   - 重置窗口内碰撞不计入得分。
6. agent 随机重置约束：
   - 用 `collision_distance` 统一替代 `safe_spawn_distance`。
   - 同时约束 agent-agent、agent-obstacle、agent-boundary、pursuer-evader(capture_radius)。
7. 动力学参数解耦：
   - pursuer/evader 分别设置 max_speed 与 max_accel。
8. 高度速度锁定：
   - 控制回写阶段强制 z 恒定，z 方向速度恒为 0。

## 2. 实施清单

- [x] 删除旧步骤书并替换为新版（本阶段）
- [x] 形成后续 STAGE2~STAGE4 的代码改造任务分解
