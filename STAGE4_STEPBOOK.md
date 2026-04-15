# 第四阶段步骤书（训练产物验收与最优权重固化）

目标：在第三阶段基础上，完成可复现训练与最优权重验证，达到“可真正落地训练”。

## 1. 执行目标

1. 在 mock 后端运行有限轮训练（例如 8~20 episodes）。
2. 验证日志出现策略优化和 best checkpoint 刷新。
3. 验证磁盘产物包含 `*_latest.pth` 与 `*_best.pth`。
4. 输出训练运行命令与后续迭代方式。

## 2. 执行清单

### 2.1 训练执行

- [x] 使用 `env_backend:=mock` 运行训练。
- [x] 指定 `max_episodes` 和 `max_steps` 使训练在短时间完成。

### 2.2 产物验收

- [x] 检查 `artifacts/checkpoints` 目录。
- [x] 确认 `pursuer_policy_latest.pth` 存在。
- [x] 确认 `pursuer_policy_best.pth` 存在。

### 2.3 结论与下一迭代

- [x] 记录本轮可复现命令。
- [x] 记录下一阶段（真实 Gazebo 训练）入口。

## 3. 可复现命令

```bash
cd ~/swarm-ros
source ./env.sh
colcon build --base-paths src
source install/setup.bash
ros2 launch swarm_bringup training_launch.py \
	env_backend:=mock \
	max_episodes:=6 \
	max_steps:=40 \
	reset_policy:=2 \
	pursuer_train_mode:=train_scratch \
	checkpoint_dir:=artifacts/checkpoints
```

## 4. 下一阶段入口（Gazebo 实训）

```bash
cd ~/swarm-ros
source ./env.sh
source install/setup.bash
ros2 launch swarm_bringup training_launch.py \
	env_backend:=gazebo \
	max_episodes:=200 \
	max_steps:=120 \
	reset_policy:=2 \
	pursuer_train_mode:=train_load \
	pursuer_weights:=artifacts/checkpoints/pursuer_policy_best.pth \
	checkpoint_dir:=artifacts/checkpoints
```
