
-----

# `game_of_drones_rl` ROS 2 仿真与训练系统使用说明

本系统用于实现多无人机追捕-逃避博弈的 DRL 训练。系统由四大核心模块构成，通过 ROS 2 话题与服务实现高度解耦。

-----

## 1\. 核心运行逻辑与时序（逻辑闭环说明）

为了保证“智能体随机生成绝对不会卡在建筑物内”，系统遵循严格的重置握手协议：

1.  **回合结束 (Done)**：`training_coordinator` 判定回合结束。
2.  **世界重置 (如有需要)**：如果策略配置为刷新世界，调度器优先发送 `/scheduler/reset_world`。
3.  **障碍物更新**：`world_manager` 完成地图刷新后，广播包含所有建筑物边界框的 `/world_manager/obstacles` 消息。
4.  **智能体安全重置**：调度器随后发送 `/scheduler/reset_agents`（携带重置模式参数）。`agent_manager` 收到指令后，**结合最新收到的障碍物边界数据**，在安全距离 `safe_spawn_distance` 之外采样合法的 $(x, y)$ 坐标，并通过 Gazebo Harmonic 底层接口瞬间重置无人机的位置与速度（设为0），从而完成逻辑闭环。

-----

## 2\. 功能包及节点详细设定

### 模块一：世界管理器 (`world_manager_node`)

**功能**：负责初始化时构建城市，以及在接收到重置指令时动态重建障碍物拓扑结构。

**可传入参数 (Parameters)**:
| 参数名 | 类型 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `world_mode` | string | `random` | `fixed` (固定地图) / `random` (随机生成) |
| `seed_mode` | string | `none` | `single` (固定种子) / `sequence` (按回合读取) / `none` (完全随机) |
| `seed_value` | int/list| `42` | 配合 `seed_mode` 使用的具体种子值 |
| `obstacle_density`| float | `0.3` | 建筑物的覆盖密度（0.0 \~ 1.0） |

**发布与订阅 (Pub/Sub)**:

  * **Sub**: `/scheduler/reset_world` (`std_msgs/Empty`) -\> 触发重置。
  * **Pub**: `/world_manager/obstacles` (`visualization_msgs/MarkerArray` 或自定义格式) -\> 向外界广播当前所有障碍物的几何中心和包围盒(Bounding Box)尺寸。

-----

### 模块二：智能体管理器 (`agent_manager_node`)

**功能**：在启动时将无人机模型注入 Gazebo；在训练过程中，负责通过改变 Pose 和 Twist（速度/加速度清零）来**重置状态，而非销毁模型**。同时负责安全出生点的碰撞检测。

**可传入参数 (Parameters)**:
| 参数名 | 类型 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `num_pursuers` | int | `5` | 追捕者数量 |
| `num_evaders` | int | `1` | 逃跑者数量 |
| `safe_spawn_distance`| float| `1.5` | **核心参数**：随机生成位置时，距离任何建筑物表面的最小安全缓冲距离（单位：米）。此距离应当大于无人机碰撞检测判定的半径。 |

**发布与订阅 (Pub/Sub)**:

  * **Sub**: `/world_manager/obstacles` -\> 缓存环境障碍物数据，用于执行避障采样。
  * **Sub**: `/scheduler/reset_agents` (`std_msgs/Int8`) -\> 接收重置指令。消息值为重置模式：`1` (恢复初始位置)，`2` (安全随机位置)。
  * **Client**: 调用 Gazebo Harmonic 服务 (例如 `/world/urban/set_pose`) 重置无人机位姿与线/角速度。

-----

### 模块三：强化学习策略网络 (`rl_policy_node`)

**功能**：运行 PyTorch 模型，接收状态，输出控制指令。追捕者和逃跑者可以完全独立地配置为“训练”或“评估”模式。

**可传入参数 (Parameters)**:
| 参数名 | 类型 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `pursuer_algo` | string | `cbc_tp_net` | 追捕者使用的网络结构（如 `maddpg`, `cbc_tp_net`）。 |
| `pursuer_train_mode`| string| `train_scratch`| **追捕者模式**：<br>- `train_scratch`: 不加载权重，从头训练<br>- `train_load`: 加载现有权重，继续 Fine-tune<br>- `eval_only`: 加载现有权重，冻结参数不训练 |
| `pursuer_weights` | string | `""` | 追捕者模型权重文件（.pth）的绝对路径。 |
| `evader_algo` | string | `ddpg` | 逃跑者使用的网络结构。 |
| `evader_train_mode` | string | `eval_only` | **逃跑者模式**：同上（论文中逃跑者通常是预训练好的静态策略）。 |
| `evader_weights` | string | `""` | 逃跑者模型权重文件路径。 |

**发布与订阅 (Pub/Sub)**:

  * **Sub**: `/agent_{i}/odom` (`nav_msgs/Odometry`) -\> 智能体运动学状态。
  * **Pub**: `/agent_{i}/cmd_accel` (`geometry_msgs/Accel`) -\> 输出期望加速度给底层的 PID 节点。

-----

### 模块四：训练调度器 (`training_coordinator_node`)

**功能**：整个 DRL 系统的“大脑”，负责计步、结算 Reward、判断 Done，并下发各级别的重置时序。

**可传入参数 (Parameters)**:
| 参数名 | 类型 | 默认值 | 描述 |
| :--- | :--- | :--- | :--- |
| `max_episodes` | int | `4000` | 最大训练回合数。 |
| `max_steps` | int | `500` | 每回合最大步数。 |
| `reset_policy` | int | `1` | **回合结束重置策略**：<br>`1`: 世界不变，智能体恢复初始固定状态。<br>`2`: 世界不变，智能体在安全区内随机刷新。<br>`3`: 世界重新随机生成，随后智能体在安全区内随机刷新。 |
| `collision_penalty` | float| `-2.0` | 发生碰撞时的惩罚值。 |
| `capture_radius` | float| `2.0` | 追捕成功判定半径。 |
| `capture_steps` | int | `5` | 需要持续处于追捕半径内的时间步才算 Done。 |

**发布与订阅 (Pub/Sub)**:

  * **Sub**: `/agent_{i}/collision` (`std_msgs/Bool`) -\> 监听碰撞事件（用于计算个体碰撞惩罚）。
  * **Pub**: `/scheduler/reset_world` (`std_msgs/Empty`) -\> 触发策略 3 时的世界重置。
  * **Pub**: `/scheduler/reset_agents` (`std_msgs/Int8`) -\> 触发智能体位姿重置，传参为 1 或 2。
  * **Pub**: `/rl/episode_metrics` -\> 记录并输出给 TensorBoard 的监控指标。

-----

## 3\. 运行示例 (Launch 演示)

一个典型的 Launch 启动配置流（在 `training_launch.py` 中体现）如下：

```python
# 场景：追捕者加载已有权重继续训练，逃跑者固定不训练，每一局随机刷新出生点，世界不刷新。
parameters={
    'reset_policy': 2, 
    'safe_spawn_distance': 1.5,
    'pursuer_train_mode': 'train_load',
    'pursuer_weights': '/path/to/cbc_tp_net.pth',
    'evader_train_mode': 'eval_only',
    'evader_weights': '/path/to/ddpg_evader.pth'
}
```

## 4\. 逻辑闭环检查清单 (Checklist)

1.  **资源复用**：`agent_manager` 不再销毁模型，仅使用 Gazebo 服务覆盖位姿和速度（Twist=0），解决了 ROS 2 频繁生成/销毁实体可能导致的 DDS 通信死锁问题。
2.  **防卡墙机制**：`world_manager` 会先发布带有体积参数的障碍物列表；`agent_manager` 订阅此列表，在随机采样 $(x_0, y_0)$ 时计算离所有边界的最短距离，若 `< safe_spawn_distance` 则重新采样，确保生成合法。
3.  **模型训练灵活性**：通过增加 `train_mode` 细分状态，现在可以轻松实现“双盲训练”（都从头训练）、“打怪升级”（加载强力逃跑者训练追捕者）、“策略评估”（双方均 eval）等完全符合论文复现要求的消融实验（Ablation Study）配置。