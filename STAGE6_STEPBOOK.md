# STAGE6 步骤书：参数体系清理与职责边界收敛

目标：移除旧参数兼容路径，收敛到当前有效参数集合，保持职责清晰。

## 1. 方案定稿

1. agent_manager 删除 safe_spawn_distance 兼容分支，仅保留 collision_distance。
2. bash 参数文件同步删除旧兼容变量，新增 agent staging 参数。
3. swarm_bringup 职责定位：
   - 一键 launch 编排训练链路。
   - 保存 world 与 model 资产。
   - 不再承担手工分步 spawn 的核心路径。

## 2. 验证标准

1. 代码中不再读取 safe_spawn_distance。
2. 手动脚本参数与 launch 参数语义一致。
3. 训练主路径可仅通过 training_launch.py 完整运行。

## 3. 执行清单

- [x] 删除 agent_manager 的旧参数兼容
- [x] 更新 bash/env_defaults 参数集
- [x] 更新 agent_manager 脚本参数透传
- [x] 同步 launch 参数默认值

## 4. 运行中修正

1. 清理兼容参数后，补充 agent_model_package 与 agent_staging_* 参数，保障模型来源与初始布局可配置。
