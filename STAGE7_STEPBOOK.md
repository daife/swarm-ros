# STAGE7 步骤书：Z 轴稳定性修复

目标：消除 Gazebo 中 z 方向抖动导致的伪碰撞与误终止（reason=2）。

## 1. 方案定稿

采用“物理层 + 控制层”组合方案：

1. 物理层：pursuer/evader 模型关闭重力。
2. 控制层：
   - 提高 sim_bridge 控制频率（controller_dt 默认 0.02）。
   - fixed_z 默认提高到 1.0，减少贴地接触风险。
   - 过滤 ground_plane 接触，不将其计为训练碰撞。
3. 启动门控：reset 前不输出有效碰撞信号。

## 2. 验证标准

1. GUI 观察 z 方向无明显上下抖动。
2. 非真实障碍接触场景下，done_reason=2 明显下降。
3. 仍可保留与障碍、边界、对手碰撞的检测能力。

## 3. 执行清单

- [x] 更新 pursuer/evader SDF（gravity=false）
- [x] 更新 sim_bridge 默认控制参数
- [x] 增加 ground_plane 接触过滤
- [x] 保留 reset 前碰撞门控

## 4. 运行中修正

1. 单纯固定 z 不足以抑制物理波动，最终采用禁重力+接触过滤组合方案。
