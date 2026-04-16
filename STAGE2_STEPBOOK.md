# STAGE2 步骤书：障碍物模型池与世界重置

目标：实现“重置世界=重置障碍物位置（不重启 Gazebo）”。

## 1. 代码改造点

1. 新增障碍物模型资产：
   - `models/obstacle_box`
   - `models/obstacle_rect`
   - `models/obstacle_cylinder`
2. 启动流程改造：
   - bridge 增加障碍物必要通道（create/set_pose）。
   - world_manager 在运行期按需创建障碍物池并放入 staging 区。
3. world_manager 升级：
   - 增加 obstacle pool 参数。
   - `fixed/random + single/sequence/none + obstacle_density` 控制激活障碍数量与布局。
   - 不足则自动追加 spawn。
   - 重置时发布 MarkerArray（供采样）并调用 SetEntityPose（供 Gazebo）。

## 2. 验证标准

1. 启动后可见障碍初始不在训练区域（在 staging）。
2. 发布一次 `/scheduler/reset_world` 后，障碍被移动到训练区。
3. 第二次 `/scheduler/reset_world` 可产生新布局（当 world_mode=random）。

## 3. 执行清单

- [x] 新增障碍物 SDF 资产
- [x] 改造 bridge/参数透传支持障碍池重置
- [x] 改造 world_manager 障碍重置逻辑
- [x] 完成一次 Gazebo 运行验证

## 4. 运行中修正

1. 首版 world_manager 要求“整个障碍池都完成生成”才执行 reset，导致完成时序偏慢。
2. 已修正为“仅要求本轮激活障碍已生成”即可完成 reset，其余已生成障碍统一回 staging。
