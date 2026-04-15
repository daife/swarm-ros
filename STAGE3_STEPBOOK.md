# 第三阶段步骤书（真实训练流水线落地）

目标：把第二阶段闭环升级为“可训练策略 + 权重持久化”。

## 1. 范围目标

1. 在 `rl_policy` 中实现在线训练（REINFORCE）而非仅规则控制。
2. 在回合结束时执行策略优化，并保存 latest/best 权重。
3. 引入可独立运行的 mock 环境，保证无 Gazebo 时也能真实训练。
4. 升级 launch，支持 `mock` 和 `gazebo` 双后端。

## 2. 执行清单

### 2.1 训练策略实现

- [x] `rl_policy_node` 支持 PyTorch 高斯策略网络。
- [x] 支持 `train_scratch` / `train_load` / `eval_only`。
- [x] 回合结束（监听 `/scheduler/reset_agents`）触发优化。
- [x] 保存 latest 权重和 best 权重（`.pth`）。

### 2.2 训练环境实现

- [x] 新增 `mock_env` 包。
- [x] 发布 `/agent_{i}/odom`、`/agent_{i}/collision`。
- [x] 订阅 `/agent_{i}/cmd_accel` 并做简化动力学积分。

### 2.3 启动体系升级

- [x] `training_launch.py` 增加 `env_backend` 参数。
- [x] `mock` 后端启动 `mock_env_node`。
- [x] `gazebo` 后端启动 `ros_gz_bridge + sim_bridge`。
- [x] 增加训练相关启动参数：`pursuer_train_mode`、`pursuer_weights`、`checkpoint_dir`。

### 2.4 验证

- [x] 安装训练依赖 `torch`。
- [x] 构建通过。
- [x] mock 后端启动并能持续触发 episode。
- [x] 生成 `.pth` 权重文件。

## 3. 产物结论

1. 已在 `artifacts/checkpoints` 生成 `pursuer_policy_latest.pth`。
2. 已在 `artifacts/checkpoints` 生成 `pursuer_policy_best.pth`。
3. checkpoint 可被 `torch.load` 正常加载，包含 `actor_state_dict`、`optimizer_state_dict`、`episode_index`、`best_episode_return`。
