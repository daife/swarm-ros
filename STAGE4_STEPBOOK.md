# STAGE4 步骤书：追逃参数解耦、高度锁定与文档收敛

目标：完成 pursuer/evader 动力学参数解耦，并把脚本与文档同步到新行为。

## 1. 代码改造点

1. sim_bridge 参数扩展：
   - `pursuer_max_speed`、`evader_max_speed`。
2. rl_policy 参数扩展：
   - `pursuer_max_accel`、`evader_max_accel`。
3. z 轴锁定：
   - 控制回写固定 z。
   - 对外发布 odom 时将 z 速度置 0。
4. launch 与 bash 参数透传：
   - 新增以上 4 个参数并写入脚本。
5. README 对齐：
   - 更新参数表与 reset 逻辑说明。

## 2. 验证标准

1. pursuer/evader 可设置不同 max_speed 和 max_accel。
2. 运行中 z 方向速度保持 0（至少对外 odom 与控制逻辑层面为 0）。
3. 文档命令与脚本可直接启动。

## 3. 执行清单

- [x] 改造 sim_bridge 速度参数
- [x] 改造 rl_policy 加速度参数
- [x] 更新 launch/bash/README
- [x] 完整构建与训练冒烟测试

## 4. 运行中修正

1. sim_bridge 增加 `control_requires_reset` 与“全 agent odom 就绪”门控，避免启动期姿态回写竞争。
2. rl_policy 不再把 `/scheduler/reset_agents` 直接当作 done 信号，避免启动期出现虚假 `reason=0` 回合。
