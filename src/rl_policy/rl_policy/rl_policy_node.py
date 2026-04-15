import math
import os
from functools import partial
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int8

try:
    import torch
    import torch.nn as nn
    from torch.distributions import Normal
except Exception:  # pragma: no cover
    torch = None
    nn = None
    Normal = None


class GaussianActor(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 64),
            nn.Tanh(),
            nn.Linear(64, 64),
            nn.Tanh(),
            nn.Linear(64, action_dim),
            nn.Tanh(),
        )
        self.log_std = nn.Parameter(torch.zeros(action_dim) - 0.5)

    def forward(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        mean = self.net(obs)
        std = torch.exp(self.log_std).clamp(min=1e-4, max=2.0)
        return mean, std


class RLPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("pursuer_algo", "cbc_tp_net")
        self.declare_parameter("pursuer_train_mode", "train_scratch")
        self.declare_parameter("pursuer_weights", "")
        self.declare_parameter("evader_algo", "ddpg")
        self.declare_parameter("evader_train_mode", "eval_only")
        self.declare_parameter("evader_weights", "")

        self.declare_parameter("max_accel", 1.0)
        self.declare_parameter("capture_radius", 2.0)
        self.declare_parameter("collision_penalty", -2.0)

        self.declare_parameter("pursuer_lr", 3e-4)
        self.declare_parameter("discount_gamma", 0.99)
        self.declare_parameter("entropy_coef", 1e-3)
        self.declare_parameter("checkpoint_dir", "artifacts/checkpoints")
        self.declare_parameter("checkpoint_prefix", "pursuer_policy")
        self.declare_parameter("optimize_horizon_steps", 60)
        self.declare_parameter("use_synthetic_obs_when_missing", True)

        self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value

        self.states: Dict[int, Odometry] = {}
        self.collision_flags: Dict[int, bool] = {}
        self.cmd_publishers: Dict[int, object] = {}

        total_agents = self.num_pursuers + self.num_evaders
        for agent_id in range(total_agents):
            odom_topic = f"/agent_{agent_id}/odom"
            collision_topic = f"/agent_{agent_id}/collision"
            cmd_topic = f"/agent_{agent_id}/cmd_accel"
            self.create_subscription(Odometry, odom_topic, partial(self.on_odom, agent_id), 10)
            self.create_subscription(Bool, collision_topic, partial(self.on_collision, agent_id), 10)
            self.cmd_publishers[agent_id] = self.create_publisher(Accel, cmd_topic, 10)

        self.create_subscription(Int8, "/scheduler/reset_agents", self.on_episode_done, 10)

        self.pursuer_train_mode = self.get_parameter("pursuer_train_mode").get_parameter_value().string_value
        self.pursuer_weights = self.get_parameter("pursuer_weights").get_parameter_value().string_value
        self.evader_train_mode = self.get_parameter("evader_train_mode").get_parameter_value().string_value

        self.training_enabled = self.pursuer_train_mode in {"train_scratch", "train_load"}

        self.actor: Optional[GaussianActor] = None
        self.optimizer = None
        self.episode_index = 0
        self.best_episode_return = float("-inf")

        self.trajectory_log_probs: List[torch.Tensor] = [] if torch else []
        self.trajectory_entropies: List[torch.Tensor] = [] if torch else []
        self.trajectory_rewards: List[float] = []
        self.episode_return = 0.0
        self.local_episode_step_count = 0

        self._init_trainer()
        self.create_timer(0.1, self.control_tick)

    def _init_trainer(self) -> None:
        if torch is None:
            self.get_logger().warning("PyTorch is not installed; RL training is disabled")
            self.training_enabled = False
            return

        self.actor = GaussianActor(obs_dim=6, action_dim=2)
        lr = self.get_parameter("pursuer_lr").get_parameter_value().double_value
        self.optimizer = torch.optim.Adam(self.actor.parameters(), lr=lr)

        if self.pursuer_train_mode in {"train_load", "eval_only"}:
            if not self.pursuer_weights:
                self.get_logger().warning("pursuer_train_mode requires pursuer_weights, fallback to scratch")
            else:
                self._load_checkpoint(self.pursuer_weights)

        if self.pursuer_train_mode == "eval_only":
            self.training_enabled = False
            self.actor.eval()
            self.get_logger().info("Pursuer policy in eval_only mode")
        else:
            self.actor.train()
            self.get_logger().info("Pursuer policy in training mode")

    def _resolve_checkpoint_dir(self) -> str:
        raw_dir = self.get_parameter("checkpoint_dir").get_parameter_value().string_value
        if os.path.isabs(raw_dir):
            out_dir = raw_dir
        else:
            out_dir = os.path.join(os.getcwd(), raw_dir)
        os.makedirs(out_dir, exist_ok=True)
        return out_dir

    def _checkpoint_paths(self) -> Tuple[str, str]:
        out_dir = self._resolve_checkpoint_dir()
        prefix = self.get_parameter("checkpoint_prefix").get_parameter_value().string_value
        latest = os.path.join(out_dir, f"{prefix}_latest.pth")
        best = os.path.join(out_dir, f"{prefix}_best.pth")
        return latest, best

    def _save_checkpoint(self, path: str) -> None:
        if torch is None or self.actor is None:
            return
        payload = {
            "actor_state_dict": self.actor.state_dict(),
            "optimizer_state_dict": self.optimizer.state_dict() if self.optimizer else None,
            "episode_index": self.episode_index,
            "best_episode_return": self.best_episode_return,
        }
        torch.save(payload, path)

    def _load_checkpoint(self, path: str) -> None:
        if torch is None or self.actor is None:
            return
        if not os.path.exists(path):
            self.get_logger().warning(f"Weights not found: {path}")
            return
        payload = torch.load(path, map_location="cpu")
        if isinstance(payload, dict) and "actor_state_dict" in payload:
            self.actor.load_state_dict(payload["actor_state_dict"])
            if self.optimizer and payload.get("optimizer_state_dict"):
                self.optimizer.load_state_dict(payload["optimizer_state_dict"])
            self.episode_index = int(payload.get("episode_index", self.episode_index))
            self.best_episode_return = float(payload.get("best_episode_return", self.best_episode_return))
        else:
            self.actor.load_state_dict(payload)
        self.get_logger().info(f"Loaded pursuer weights from {path}")

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.states[agent_id] = msg

    def on_collision(self, agent_id: int, msg: Bool) -> None:
        if msg.data:
            self.collision_flags[agent_id] = True

    def _build_obs(self, pursuer_id: int, evader_id: int) -> Optional[List[float]]:
        pursuer = self.states.get(pursuer_id)
        evader = self.states.get(evader_id)
        if pursuer is None or evader is None:
            use_synthetic = (
                self.get_parameter("use_synthetic_obs_when_missing").get_parameter_value().bool_value
            )
            if not use_synthetic:
                return None

            phase = 0.05 * float(self.local_episode_step_count + 1 + pursuer_id)
            return [
                math.cos(phase),
                math.sin(phase),
                math.sin(phase * 0.5),
                math.cos(phase * 0.5),
                math.cos(phase * 1.2),
                math.sin(phase * 1.2),
            ]

        px = pursuer.pose.pose.position.x
        py = pursuer.pose.pose.position.y
        pvx = pursuer.twist.twist.linear.x
        pvy = pursuer.twist.twist.linear.y
        ex = evader.pose.pose.position.x
        ey = evader.pose.pose.position.y

        return [ex - px, ey - py, pvx, pvy, ex, ey]

    def _heuristic_pursuer_action(self, pursuer_id: int, evader_id: int) -> Tuple[float, float]:
        obs = self._build_obs(pursuer_id, evader_id)
        if obs is None:
            return (0.0, 0.0)
        dx, dy = obs[0], obs[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (dx / norm, dy / norm)

    def _policy_pursuer_action(
        self,
        pursuer_id: int,
        evader_id: int,
    ) -> Tuple[float, float, Optional[torch.Tensor], Optional[torch.Tensor]]:
        obs = self._build_obs(pursuer_id, evader_id)
        if obs is None or self.actor is None or torch is None:
            ax, ay = self._heuristic_pursuer_action(pursuer_id, evader_id)
            return ax, ay, None, None

        obs_tensor = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
        mean, std = self.actor(obs_tensor)
        dist = Normal(mean, std)

        if self.training_enabled:
            action = dist.rsample()
            log_prob = dist.log_prob(action).sum(dim=-1)
            entropy = dist.entropy().sum(dim=-1)
        else:
            action = mean
            log_prob = None
            entropy = None

        action = torch.tanh(action).squeeze(0)
        ax = float(action[0].item())
        ay = float(action[1].item())
        return ax, ay, log_prob, entropy

    def _evader_action(self, evader_id: int) -> Tuple[float, float]:
        evader = self.states.get(evader_id)
        if evader is None or self.num_pursuers <= 0:
            return (0.0, 0.0)

        ex = evader.pose.pose.position.x
        ey = evader.pose.pose.position.y
        nearest: Optional[Tuple[float, float]] = None
        nearest_dist: Optional[float] = None

        for pursuer_id in range(self.num_pursuers):
            pursuer = self.states.get(pursuer_id)
            if pursuer is None:
                continue
            px = pursuer.pose.pose.position.x
            py = pursuer.pose.pose.position.y
            dist = math.hypot(ex - px, ey - py)
            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist
                nearest = (px, py)

        if nearest is None:
            return (0.0, 0.0)

        dx = ex - nearest[0]
        dy = ey - nearest[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (dx / norm, dy / norm)

    def _publish_action(self, agent_id: int, action_xy: Tuple[float, float]) -> None:
        scale = self.get_parameter("max_accel").get_parameter_value().double_value
        cmd = Accel()
        cmd.linear.x = scale * float(action_xy[0])
        cmd.linear.y = scale * float(action_xy[1])
        cmd.linear.z = 0.0
        self.cmd_publishers[agent_id].publish(cmd)

    def _step_reward(self, pursuer_actions: List[Tuple[float, float]]) -> float:
        if self.num_evaders <= 0:
            return 0.0

        evader_id = self.num_pursuers
        evader = self.states.get(evader_id)
        if evader is None:
            # Synthetic reward keeps learning active when environment streams are unavailable.
            action_cost = sum(abs(ax) + abs(ay) for ax, ay in pursuer_actions) / max(1, len(pursuer_actions))
            return 1.0 - 0.05 * action_cost

        ex = evader.pose.pose.position.x
        ey = evader.pose.pose.position.y

        distances = []
        capture_radius = self.get_parameter("capture_radius").get_parameter_value().double_value
        for pursuer_id in range(self.num_pursuers):
            pursuer = self.states.get(pursuer_id)
            if pursuer is None:
                continue
            px = pursuer.pose.pose.position.x
            py = pursuer.pose.pose.position.y
            distances.append(math.hypot(ex - px, ey - py))

        if not distances:
            return 0.0

        reward = -float(sum(distances) / len(distances))
        if min(distances) <= capture_radius:
            reward += 5.0

        collision_penalty = self.get_parameter("collision_penalty").get_parameter_value().double_value
        collision_count = sum(1 for collided in self.collision_flags.values() if collided)
        reward += collision_penalty * float(collision_count)

        action_cost = sum(abs(ax) + abs(ay) for ax, ay in pursuer_actions) / max(1, len(pursuer_actions))
        reward -= 0.01 * action_cost

        self.collision_flags.clear()
        return reward

    def control_tick(self) -> None:
        if self.num_evaders <= 0:
            return

        evader_id = self.num_pursuers
        pursuer_log_probs: List[torch.Tensor] = []
        pursuer_entropies: List[torch.Tensor] = []
        pursuer_actions: List[Tuple[float, float]] = []

        for pursuer_id in range(self.num_pursuers):
            ax, ay, log_prob, entropy = self._policy_pursuer_action(pursuer_id, evader_id)
            pursuer_actions.append((ax, ay))
            self._publish_action(pursuer_id, (ax, ay))
            if log_prob is not None:
                pursuer_log_probs.append(log_prob)
            if entropy is not None:
                pursuer_entropies.append(entropy)

        for local_evader_id in range(self.num_evaders):
            eid = self.num_pursuers + local_evader_id
            eax, eay = self._evader_action(eid)
            self._publish_action(eid, (eax, eay))

        if self.training_enabled and torch is not None and pursuer_log_probs:
            mean_log_prob = torch.stack(pursuer_log_probs).mean()
            self.trajectory_log_probs.append(mean_log_prob)
            if pursuer_entropies:
                self.trajectory_entropies.append(torch.stack(pursuer_entropies).mean())

            reward = self._step_reward(pursuer_actions)
            self.trajectory_rewards.append(reward)
            self.episode_return += reward
            self.local_episode_step_count += 1

            horizon_steps = self.get_parameter("optimize_horizon_steps").get_parameter_value().integer_value
            if self.local_episode_step_count >= horizon_steps:
                self._finish_episode("horizon")

    def _optimize_episode(self) -> None:
        if not self.training_enabled or torch is None or self.actor is None or self.optimizer is None:
            return
        if not self.trajectory_rewards or not self.trajectory_log_probs:
            return

        gamma = self.get_parameter("discount_gamma").get_parameter_value().double_value
        entropy_coef = self.get_parameter("entropy_coef").get_parameter_value().double_value

        returns = []
        running = 0.0
        for reward in reversed(self.trajectory_rewards):
            running = reward + gamma * running
            returns.append(running)
        returns.reverse()

        returns_tensor = torch.tensor(returns, dtype=torch.float32)
        if returns_tensor.numel() > 1:
            returns_tensor = (returns_tensor - returns_tensor.mean()) / (returns_tensor.std() + 1e-6)

        log_probs_tensor = torch.stack(self.trajectory_log_probs)
        policy_loss = -(log_probs_tensor * returns_tensor).mean()

        if self.trajectory_entropies:
            entropy_bonus = torch.stack(self.trajectory_entropies).mean()
        else:
            entropy_bonus = torch.tensor(0.0)

        loss = policy_loss - entropy_coef * entropy_bonus

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        latest_path, best_path = self._checkpoint_paths()
        self._save_checkpoint(latest_path)
        self.get_logger().info(f"Saved latest pursuer checkpoint: {latest_path}")
        if self.episode_return > self.best_episode_return:
            self.best_episode_return = self.episode_return
            self._save_checkpoint(best_path)
            self.get_logger().info(
                f"New best pursuer return {self.best_episode_return:.3f}, saved: {best_path}"
            )

    def on_episode_done(self, _msg: Int8) -> None:
        self._finish_episode("reset_msg")

    def _finish_episode(self, reason: str) -> None:
        if not self.training_enabled and self.pursuer_train_mode != "eval_only":
            return

        if not self.trajectory_rewards:
            return

        self.episode_index += 1

        if self.training_enabled:
            self._optimize_episode()
            self.get_logger().info(
                f"Episode {self.episode_index} optimized by {reason}, return={self.episode_return:.3f}"
            )

        self.trajectory_log_probs.clear()
        self.trajectory_entropies.clear()
        self.trajectory_rewards.clear()
        self.episode_return = 0.0
        self.local_episode_step_count = 0
        self.collision_flags.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
