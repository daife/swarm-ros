import copy
import math
import os
import random
from collections import deque
from functools import partial
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Int8

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
except Exception:  # pragma: no cover
    torch = None
    nn = None
    F = None


class ReplayBuffer:
    def __init__(self, capacity: int) -> None:
        self.storage: Deque[Dict[str, np.ndarray]] = deque(maxlen=capacity)

    def add(self, transition: Dict[str, np.ndarray]) -> None:
        self.storage.append(transition)

    def __len__(self) -> int:
        return len(self.storage)

    def sample(self, batch_size: int) -> Dict[str, np.ndarray]:
        batch = random.sample(self.storage, batch_size)
        out: Dict[str, List[np.ndarray]] = {}
        for item in batch:
            for key, value in item.items():
                out.setdefault(key, []).append(value)
        return {key: np.stack(value, axis=0) for key, value in out.items()}


class Actor(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)


class Critic(nn.Module):
    def __init__(self, input_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 192),
            nn.ReLU(),
            nn.Linear(192, 192),
            nn.ReLU(),
            nn.Linear(192, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class RLPolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("rl_policy_node")
        self.declare_parameter("num_pursuers", 5)
        self.declare_parameter("num_evaders", 1)
        self.declare_parameter("pursuer_algo", "maddpg")
        self.declare_parameter("pursuer_train_mode", "train_scratch")
        self.declare_parameter("pursuer_weights", "")
        self.declare_parameter("evader_algo", "ddpg")
        self.declare_parameter("evader_train_mode", "eval_only")
        self.declare_parameter("evader_weights", "")

        self.declare_parameter("max_accel", 1.0)
        self.declare_parameter("pursuer_max_accel", -1.0)
        self.declare_parameter("evader_max_accel", -1.0)
        self.declare_parameter("capture_radius", 2.0)
        self.declare_parameter("collision_penalty", -2.0)
        self.declare_parameter("discount_gamma", 0.99)
        self.declare_parameter("target_tau", 0.01)
        self.declare_parameter("pursuer_actor_lr", 3e-4)
        self.declare_parameter("pursuer_critic_lr", 1e-3)
        self.declare_parameter("evader_actor_lr", 3e-4)
        self.declare_parameter("evader_critic_lr", 1e-3)
        self.declare_parameter("buffer_capacity", 200000)
        self.declare_parameter("batch_size", 128)
        self.declare_parameter("warmup_steps", 500)
        self.declare_parameter("train_updates_per_step", 1)
        self.declare_parameter("exploration_noise_std", 0.2)
        self.declare_parameter("policy_update_interval", 1)
        self.declare_parameter("optimize_horizon_steps", 120)
        self.declare_parameter("checkpoint_dir", "artifacts/checkpoints")
        self.declare_parameter("checkpoint_prefix", "pursuer_policy")
        self.declare_parameter("use_synthetic_obs_when_missing", True)
        self.declare_parameter("torch_device", "auto")

        self.num_pursuers = self.get_parameter("num_pursuers").get_parameter_value().integer_value
        self.num_evaders = self.get_parameter("num_evaders").get_parameter_value().integer_value
        self.total_agents = self.num_pursuers + self.num_evaders
        self.local_obs_dim = 6
        self.global_state_dim = 4 * self.total_agents

        self.pursuer_algo = self.get_parameter("pursuer_algo").get_parameter_value().string_value
        self.pursuer_train_mode = self.get_parameter("pursuer_train_mode").get_parameter_value().string_value
        self.pursuer_weights = self.get_parameter("pursuer_weights").get_parameter_value().string_value
        self.evader_algo = self.get_parameter("evader_algo").get_parameter_value().string_value
        self.evader_train_mode = self.get_parameter("evader_train_mode").get_parameter_value().string_value
        self.evader_weights = self.get_parameter("evader_weights").get_parameter_value().string_value

        self.use_synthetic_obs = (
            self.get_parameter("use_synthetic_obs_when_missing").get_parameter_value().bool_value
        )

        self.states: Dict[int, Odometry] = {}
        self.collision_flags: Dict[int, bool] = {i: False for i in range(self.total_agents)}
        self.cmd_publishers: Dict[int, object] = {}

        for agent_id in range(self.total_agents):
            self.create_subscription(Odometry, f"/agent_{agent_id}/odom", partial(self.on_odom, agent_id), 10)
            self.create_subscription(
                Bool,
                f"/agent_{agent_id}/collision",
                partial(self.on_collision, agent_id),
                10,
            )
            self.cmd_publishers[agent_id] = self.create_publisher(Accel, f"/agent_{agent_id}/cmd_accel", 10)

        self.create_subscription(Int8, "/scheduler/reset_agents", self.on_episode_done, 10)
        self.create_subscription(Float32MultiArray, "/rl/episode_metrics", self.on_episode_metrics, 10)

        self.done_latched = False
        self.latest_done_reason = 0
        self.total_env_steps = 0
        self.episode_index = 0
        self.episode_return_p = 0.0
        self.episode_return_e = 0.0
        self.best_pursuer_return = float("-inf")
        self.best_evader_return = float("-inf")

        capacity = self.get_parameter("buffer_capacity").get_parameter_value().integer_value
        self.replay = ReplayBuffer(capacity=max(1000, int(capacity)))

        self.prev_snapshot: Optional[Dict[str, np.ndarray]] = None
        self.step_counter_for_policy_delay = 0
        self.local_episode_step_count = 0

        self.device = self._select_torch_device()
        self._init_models()

        self.create_timer(0.1, self.control_tick)

    def _select_torch_device(self):
        if torch is None:
            return None

        requested = self.get_parameter("torch_device").get_parameter_value().string_value.strip().lower()
        if requested in {"", "auto"}:
            requested = "cuda" if torch.cuda.is_available() else "cpu"

        try:
            device = torch.device(requested)
        except Exception:
            self.get_logger().warning(f"Invalid torch_device={requested}, fallback to cpu")
            device = torch.device("cpu")

        if device.type == "cuda" and not torch.cuda.is_available():
            self.get_logger().warning("torch_device requests CUDA but CUDA is unavailable, fallback to cpu")
            device = torch.device("cpu")

        if device.type == "cuda":
            gpu_name = torch.cuda.get_device_name(device)
            self.get_logger().info(f"RL torch device: {device} ({gpu_name})")
            torch.backends.cudnn.benchmark = True
        else:
            self.get_logger().info("RL torch device: cpu")

        return device

    def _init_models(self) -> None:
        self.pursuer_training_enabled = self.pursuer_train_mode in {"train_scratch", "train_load"}
        self.evader_training_enabled = self.evader_train_mode in {"train_scratch", "train_load"}

        if torch is None:
            self.get_logger().warning("PyTorch unavailable, fallback to heuristic controllers")
            self.pursuer_training_enabled = False
            self.evader_training_enabled = False
            self.pursuer_actors = []
            self.pursuer_target_actors = []
            self.pursuer_critics = []
            self.pursuer_target_critics = []
            self.pursuer_actor_optim = []
            self.pursuer_critic_optim = []
            self.evader_actor = None
            self.evader_target_actor = None
            self.evader_critic = None
            self.evader_target_critic = None
            self.evader_actor_optim = None
            self.evader_critic_optim = None
            return

        if self.pursuer_algo == "cbc_tp_net":
            self.get_logger().warning("cbc_tp_net暂未实现，追捕者将使用启发式控制")
            self.pursuer_training_enabled = False

        if self.pursuer_algo not in {"maddpg", "cbc_tp_net"}:
            self.get_logger().warning(f"Unknown pursuer_algo={self.pursuer_algo}, fallback to maddpg")
            self.pursuer_algo = "maddpg"

        if self.evader_algo not in {"ddpg"}:
            self.get_logger().warning(f"Unknown evader_algo={self.evader_algo}, fallback to ddpg")
            self.evader_algo = "ddpg"

        pursuer_actor_lr = self.get_parameter("pursuer_actor_lr").get_parameter_value().double_value
        pursuer_critic_lr = self.get_parameter("pursuer_critic_lr").get_parameter_value().double_value
        evader_actor_lr = self.get_parameter("evader_actor_lr").get_parameter_value().double_value
        evader_critic_lr = self.get_parameter("evader_critic_lr").get_parameter_value().double_value

        joint_action_dim = 2 * self.num_pursuers
        critic_input_dim = self.global_state_dim + joint_action_dim

        self.pursuer_actors = [Actor(self.local_obs_dim, 2).to(self.device) for _ in range(self.num_pursuers)]
        self.pursuer_target_actors = [copy.deepcopy(actor) for actor in self.pursuer_actors]
        self.pursuer_critics = [Critic(critic_input_dim).to(self.device) for _ in range(self.num_pursuers)]
        self.pursuer_target_critics = [copy.deepcopy(critic) for critic in self.pursuer_critics]

        self.pursuer_actor_optim = [
            torch.optim.Adam(actor.parameters(), lr=pursuer_actor_lr)
            for actor in self.pursuer_actors
        ]
        self.pursuer_critic_optim = [
            torch.optim.Adam(critic.parameters(), lr=pursuer_critic_lr)
            for critic in self.pursuer_critics
        ]

        self.evader_actor = Actor(self.local_obs_dim, 2).to(self.device)
        self.evader_target_actor = copy.deepcopy(self.evader_actor)
        self.evader_critic = Critic(self.local_obs_dim + 2).to(self.device)
        self.evader_target_critic = copy.deepcopy(self.evader_critic)
        self.evader_actor_optim = torch.optim.Adam(self.evader_actor.parameters(), lr=evader_actor_lr)
        self.evader_critic_optim = torch.optim.Adam(self.evader_critic.parameters(), lr=evader_critic_lr)

        if self.pursuer_train_mode in {"train_load", "eval_only"} and self.pursuer_weights:
            self._load_checkpoint(self.pursuer_weights)
        if self.evader_train_mode in {"train_load", "eval_only"} and self.evader_weights:
            self._load_checkpoint(self.evader_weights)

        if self.pursuer_train_mode == "eval_only":
            self.pursuer_training_enabled = False
        if self.evader_train_mode == "eval_only":
            self.evader_training_enabled = False

        if self.pursuer_train_mode == "train_load" and not self.pursuer_weights:
            self.get_logger().warning("pursuer_train_mode=train_load 但未提供 pursuer_weights")
        if self.evader_train_mode == "train_load" and not self.evader_weights:
            self.get_logger().warning("evader_train_mode=train_load 但未提供 evader_weights")

        self.get_logger().info(
            "RL policy ready: pursuer_algo=%s, pursuer_mode=%s, evader_algo=%s, evader_mode=%s"
            % (
                self.pursuer_algo,
                self.pursuer_train_mode,
                self.evader_algo,
                self.evader_train_mode,
            )
        )

    def _resolve_checkpoint_dir(self) -> str:
        raw_dir = self.get_parameter("checkpoint_dir").get_parameter_value().string_value
        out_dir = raw_dir if os.path.isabs(raw_dir) else os.path.join(os.getcwd(), raw_dir)
        os.makedirs(out_dir, exist_ok=True)
        return out_dir

    def _checkpoint_paths(self) -> Tuple[str, str, str]:
        out_dir = self._resolve_checkpoint_dir()
        prefix = self.get_parameter("checkpoint_prefix").get_parameter_value().string_value
        latest = os.path.join(out_dir, f"{prefix}_latest.pth")
        pursuer_best = os.path.join(out_dir, f"{prefix}_best.pth")
        evader_best = os.path.join(out_dir, f"{prefix}_evader_best.pth")
        return latest, pursuer_best, evader_best

    def _save_checkpoint(self, path: str) -> None:
        if torch is None:
            return

        payload = {
            "episode_index": self.episode_index,
            "best_pursuer_return": self.best_pursuer_return,
            "best_evader_return": self.best_evader_return,
            "num_pursuers": self.num_pursuers,
            "num_evaders": self.num_evaders,
            "pursuer_algo": self.pursuer_algo,
            "evader_algo": self.evader_algo,
        }

        if self.pursuer_actors:
            payload["pursuer_actor_state_dicts"] = [m.state_dict() for m in self.pursuer_actors]
            payload["pursuer_critic_state_dicts"] = [m.state_dict() for m in self.pursuer_critics]
            payload["pursuer_target_actor_state_dicts"] = [m.state_dict() for m in self.pursuer_target_actors]
            payload["pursuer_target_critic_state_dicts"] = [m.state_dict() for m in self.pursuer_target_critics]
            payload["pursuer_actor_optim_state_dicts"] = [o.state_dict() for o in self.pursuer_actor_optim]
            payload["pursuer_critic_optim_state_dicts"] = [o.state_dict() for o in self.pursuer_critic_optim]

        if self.evader_actor is not None:
            payload["evader_actor_state_dict"] = self.evader_actor.state_dict()
            payload["evader_target_actor_state_dict"] = self.evader_target_actor.state_dict()
            payload["evader_critic_state_dict"] = self.evader_critic.state_dict()
            payload["evader_target_critic_state_dict"] = self.evader_target_critic.state_dict()
            payload["evader_actor_optim_state_dict"] = self.evader_actor_optim.state_dict()
            payload["evader_critic_optim_state_dict"] = self.evader_critic_optim.state_dict()

        torch.save(payload, path)

    def _load_checkpoint(self, path: str) -> None:
        if torch is None:
            return
        if not os.path.exists(path):
            self.get_logger().warning(f"Checkpoint not found: {path}")
            return

        payload = torch.load(path, map_location=self.device)
        if not isinstance(payload, dict):
            self.get_logger().warning(f"Unsupported checkpoint format: {path}")
            return

        # Backward compatibility with previous single-actor checkpoint format.
        if "actor_state_dict" in payload and self.pursuer_actors:
            self.pursuer_actors[0].load_state_dict(payload["actor_state_dict"])
            self.pursuer_target_actors[0].load_state_dict(payload["actor_state_dict"])
            if payload.get("optimizer_state_dict") and self.pursuer_actor_optim:
                self.pursuer_actor_optim[0].load_state_dict(payload["optimizer_state_dict"])
            self.episode_index = int(payload.get("episode_index", self.episode_index))
            self.best_pursuer_return = float(
                payload.get("best_episode_return", self.best_pursuer_return)
            )
            self.get_logger().info(f"Loaded legacy checkpoint: {path}")
            return

        if payload.get("pursuer_actor_state_dicts") and self.pursuer_actors:
            actor_states = payload["pursuer_actor_state_dicts"]
            for model, state in zip(self.pursuer_actors, actor_states):
                model.load_state_dict(state)
            target_actor_states = payload.get("pursuer_target_actor_state_dicts", actor_states)
            for model, state in zip(self.pursuer_target_actors, target_actor_states):
                model.load_state_dict(state)

            critic_states = payload.get("pursuer_critic_state_dicts", [])
            for model, state in zip(self.pursuer_critics, critic_states):
                model.load_state_dict(state)
            target_critic_states = payload.get("pursuer_target_critic_state_dicts", critic_states)
            for model, state in zip(self.pursuer_target_critics, target_critic_states):
                model.load_state_dict(state)

            actor_optim_states = payload.get("pursuer_actor_optim_state_dicts", [])
            for optim, state in zip(self.pursuer_actor_optim, actor_optim_states):
                optim.load_state_dict(state)
            critic_optim_states = payload.get("pursuer_critic_optim_state_dicts", [])
            for optim, state in zip(self.pursuer_critic_optim, critic_optim_states):
                optim.load_state_dict(state)

        if payload.get("evader_actor_state_dict") and self.evader_actor is not None:
            self.evader_actor.load_state_dict(payload["evader_actor_state_dict"])
            self.evader_target_actor.load_state_dict(
                payload.get("evader_target_actor_state_dict", payload["evader_actor_state_dict"])
            )
            if payload.get("evader_critic_state_dict"):
                self.evader_critic.load_state_dict(payload["evader_critic_state_dict"])
                self.evader_target_critic.load_state_dict(
                    payload.get("evader_target_critic_state_dict", payload["evader_critic_state_dict"])
                )
            if payload.get("evader_actor_optim_state_dict"):
                self.evader_actor_optim.load_state_dict(payload["evader_actor_optim_state_dict"])
            if payload.get("evader_critic_optim_state_dict"):
                self.evader_critic_optim.load_state_dict(payload["evader_critic_optim_state_dict"])

        self.episode_index = int(payload.get("episode_index", self.episode_index))
        self.best_pursuer_return = float(payload.get("best_pursuer_return", self.best_pursuer_return))
        self.best_evader_return = float(payload.get("best_evader_return", self.best_evader_return))
        self.get_logger().info(f"Loaded checkpoint: {path}")

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.states[agent_id] = msg

    def on_collision(self, agent_id: int, msg: Bool) -> None:
        if msg.data:
            self.collision_flags[agent_id] = True

    def on_episode_done(self, _msg: Int8) -> None:
        # Do not treat reset command itself as episode boundary.
        # Episode boundary is decided by /rl/episode_metrics (or horizon fallback).
        return

    def on_episode_metrics(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self.latest_done_reason = int(msg.data[2])
            self.done_latched = True

    def _synthetic_kinematics(self, agent_id: int) -> Tuple[float, float, float, float]:
        phase = 0.05 * float(self.total_env_steps + 1 + agent_id)
        x = 4.0 * math.cos(phase + 0.2 * agent_id)
        y = 4.0 * math.sin(phase + 0.2 * agent_id)
        vx = -0.2 * math.sin(phase)
        vy = 0.2 * math.cos(phase)
        return x, y, vx, vy

    def _agent_kinematics(self, agent_id: int) -> Optional[Tuple[float, float, float, float]]:
        msg = self.states.get(agent_id)
        if msg is not None:
            return (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
            )
        if self.use_synthetic_obs:
            return self._synthetic_kinematics(agent_id)
        return None

    def _build_global_state(self) -> Optional[np.ndarray]:
        values: List[float] = []
        for aid in range(self.total_agents):
            kin = self._agent_kinematics(aid)
            if kin is None:
                return None
            values.extend([kin[0], kin[1], kin[2], kin[3]])
        return np.asarray(values, dtype=np.float32)

    def _nearest_evader_for_pursuer(self, pursuer_id: int) -> Optional[Tuple[float, float, float, float]]:
        pkin = self._agent_kinematics(pursuer_id)
        if pkin is None:
            return None
        px, py, _, _ = pkin
        nearest = None
        nearest_dist = None
        for eidx in range(self.num_evaders):
            eid = self.num_pursuers + eidx
            ekin = self._agent_kinematics(eid)
            if ekin is None:
                continue
            ex, ey, evx, evy = ekin
            dist = math.hypot(ex - px, ey - py)
            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist
                nearest = (ex, ey, evx, evy)
        return nearest

    def _nearest_pursuer_for_evader(self, evader_id: int) -> Optional[Tuple[float, float, float, float]]:
        ekin = self._agent_kinematics(evader_id)
        if ekin is None:
            return None
        ex, ey, _, _ = ekin
        nearest = None
        nearest_dist = None
        for pid in range(self.num_pursuers):
            pkin = self._agent_kinematics(pid)
            if pkin is None:
                continue
            px, py, pvx, pvy = pkin
            dist = math.hypot(ex - px, ey - py)
            if nearest_dist is None or dist < nearest_dist:
                nearest_dist = dist
                nearest = (px, py, pvx, pvy)
        return nearest

    def _build_pursuer_obs(self) -> Optional[np.ndarray]:
        obs_rows = []
        for pid in range(self.num_pursuers):
            pkin = self._agent_kinematics(pid)
            nearest = self._nearest_evader_for_pursuer(pid)
            if pkin is None or nearest is None:
                return None
            px, py, pvx, pvy = pkin
            ex, ey, _, _ = nearest
            obs_rows.append([ex - px, ey - py, pvx, pvy, ex, ey])
        if not obs_rows:
            return np.zeros((0, self.local_obs_dim), dtype=np.float32)
        return np.asarray(obs_rows, dtype=np.float32)

    def _build_evader_obs(self) -> Optional[np.ndarray]:
        obs_rows = []
        for eidx in range(self.num_evaders):
            eid = self.num_pursuers + eidx
            ekin = self._agent_kinematics(eid)
            nearest = self._nearest_pursuer_for_evader(eid)
            if ekin is None or nearest is None:
                return None
            ex, ey, evx, evy = ekin
            px, py, _, _ = nearest
            obs_rows.append([ex - px, ey - py, evx, evy, px, py])
        if not obs_rows:
            return np.zeros((0, self.local_obs_dim), dtype=np.float32)
        return np.asarray(obs_rows, dtype=np.float32)

    def _heuristic_pursuer_action(self, obs_row: np.ndarray) -> np.ndarray:
        dx = obs_row[0]
        dy = obs_row[1]
        norm = math.hypot(float(dx), float(dy))
        if norm < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return np.asarray([dx / norm, dy / norm], dtype=np.float32)

    def _heuristic_evader_action(self, obs_row: np.ndarray) -> np.ndarray:
        dx = obs_row[0]
        dy = obs_row[1]
        norm = math.hypot(float(dx), float(dy))
        if norm < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return np.asarray([dx / norm, dy / norm], dtype=np.float32)

    def _policy_pursuer_actions(self, pursuer_obs: np.ndarray) -> np.ndarray:
        actions = []
        noise_std = self.get_parameter("exploration_noise_std").get_parameter_value().double_value

        for idx in range(self.num_pursuers):
            obs_row = pursuer_obs[idx]
            if (
                torch is None
                or self.pursuer_algo != "maddpg"
                or not self.pursuer_actors
                or idx >= len(self.pursuer_actors)
            ):
                action = self._heuristic_pursuer_action(obs_row)
            else:
                obs_tensor = torch.tensor(obs_row, dtype=torch.float32, device=self.device).unsqueeze(0)
                with torch.no_grad():
                    action_tensor = self.pursuer_actors[idx](obs_tensor)
                action = action_tensor.squeeze(0).cpu().numpy()
                if self.pursuer_training_enabled:
                    action = action + np.random.normal(0.0, noise_std, size=2)
                action = np.clip(action, -1.0, 1.0)
            actions.append(action.astype(np.float32))

        if not actions:
            return np.zeros((0, 2), dtype=np.float32)
        return np.asarray(actions, dtype=np.float32)

    def _policy_evader_actions(self, evader_obs: np.ndarray) -> np.ndarray:
        actions = []
        noise_std = self.get_parameter("exploration_noise_std").get_parameter_value().double_value

        for idx in range(self.num_evaders):
            obs_row = evader_obs[idx]
            if torch is None or self.evader_algo != "ddpg" or self.evader_actor is None:
                action = self._heuristic_evader_action(obs_row)
            else:
                obs_tensor = torch.tensor(obs_row, dtype=torch.float32, device=self.device).unsqueeze(0)
                with torch.no_grad():
                    action_tensor = self.evader_actor(obs_tensor)
                action = action_tensor.squeeze(0).cpu().numpy()
                if self.evader_training_enabled:
                    action = action + np.random.normal(0.0, noise_std, size=2)
                action = np.clip(action, -1.0, 1.0)
            actions.append(action.astype(np.float32))

        if not actions:
            return np.zeros((0, 2), dtype=np.float32)
        return np.asarray(actions, dtype=np.float32)

    def _publish_action(self, agent_id: int, action_xy: np.ndarray) -> None:
        default_scale = self.get_parameter("max_accel").get_parameter_value().double_value
        pursuer_scale = self.get_parameter("pursuer_max_accel").get_parameter_value().double_value
        evader_scale = self.get_parameter("evader_max_accel").get_parameter_value().double_value

        if agent_id < self.num_pursuers:
            scale = pursuer_scale if pursuer_scale > 0.0 else default_scale
        else:
            scale = evader_scale if evader_scale > 0.0 else default_scale

        cmd = Accel()
        cmd.linear.x = float(scale * action_xy[0])
        cmd.linear.y = float(scale * action_xy[1])
        cmd.linear.z = 0.0
        self.cmd_publishers[agent_id].publish(cmd)

    def _compute_rewards(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        collision_penalty = self.get_parameter("collision_penalty").get_parameter_value().double_value
        capture_radius = self.get_parameter("capture_radius").get_parameter_value().double_value

        pursuer_rewards = np.zeros(self.num_pursuers, dtype=np.float32)
        evader_rewards = np.zeros(self.num_evaders, dtype=np.float32)
        captured = False

        if self.num_pursuers == 0 or self.num_evaders == 0:
            return pursuer_rewards, evader_rewards, captured

        distances: List[float] = []
        for pid in range(self.num_pursuers):
            pkin = self._agent_kinematics(pid)
            if pkin is None:
                continue
            px, py, _, _ = pkin
            min_dist = None
            for eidx in range(self.num_evaders):
                eid = self.num_pursuers + eidx
                ekin = self._agent_kinematics(eid)
                if ekin is None:
                    continue
                ex, ey, _, _ = ekin
                d = math.hypot(ex - px, ey - py)
                if min_dist is None or d < min_dist:
                    min_dist = d
            if min_dist is not None:
                distances.append(min_dist)

        if distances:
            mean_dist = float(sum(distances) / len(distances))
            captured = min(distances) <= capture_radius
            shared = -mean_dist + (5.0 if captured else 0.0)
            pursuer_rewards[:] = shared
            evader_rewards[:] = mean_dist - (5.0 if captured else 0.0)

        for pid in range(self.num_pursuers):
            if self.collision_flags.get(pid, False):
                pursuer_rewards[pid] += float(collision_penalty)
        for eidx in range(self.num_evaders):
            eid = self.num_pursuers + eidx
            if self.collision_flags.get(eid, False):
                evader_rewards[eidx] += float(collision_penalty)

        for aid in range(self.total_agents):
            self.collision_flags[aid] = False

        return pursuer_rewards, evader_rewards, captured

    def _to_tensor(self, array: np.ndarray) -> torch.Tensor:
        return torch.tensor(array, dtype=torch.float32, device=self.device)

    def _soft_update(self, src: nn.Module, dst: nn.Module, tau: float) -> None:
        for src_p, dst_p in zip(src.parameters(), dst.parameters()):
            dst_p.data.copy_(tau * src_p.data + (1.0 - tau) * dst_p.data)

    def _train_step(self) -> None:
        if torch is None:
            return

        batch_size = self.get_parameter("batch_size").get_parameter_value().integer_value
        warmup_steps = self.get_parameter("warmup_steps").get_parameter_value().integer_value
        if len(self.replay) < max(batch_size, warmup_steps):
            return

        batch_np = self.replay.sample(batch_size)
        batch = {key: self._to_tensor(value) for key, value in batch_np.items()}

        gamma = self.get_parameter("discount_gamma").get_parameter_value().double_value
        tau = self.get_parameter("target_tau").get_parameter_value().double_value
        policy_update_interval = (
            self.get_parameter("policy_update_interval").get_parameter_value().integer_value
        )

        done = batch["done"].view(-1)

        state = batch["state"]
        next_state = batch["next_state"]
        pursuer_obs = batch["pursuer_obs"]
        next_pursuer_obs = batch["next_pursuer_obs"]
        pursuer_actions = batch["pursuer_actions"].reshape(batch_size, -1)
        pursuer_rewards = batch["pursuer_rewards"]

        self.step_counter_for_policy_delay += 1
        policy_update_due = self.step_counter_for_policy_delay % max(1, policy_update_interval) == 0

        if self.pursuer_training_enabled and self.pursuer_algo == "maddpg" and self.num_pursuers > 0:
            with torch.no_grad():
                target_next_actions = []
                for i in range(self.num_pursuers):
                    target_next_actions.append(self.pursuer_target_actors[i](next_pursuer_obs[:, i, :]))
                target_next_actions_cat = torch.cat(target_next_actions, dim=-1)

            for i in range(self.num_pursuers):
                critic_in = torch.cat([state, pursuer_actions], dim=-1)
                q_value = self.pursuer_critics[i](critic_in).squeeze(-1)

                with torch.no_grad():
                    target_critic_in = torch.cat([next_state, target_next_actions_cat], dim=-1)
                    target_q = self.pursuer_target_critics[i](target_critic_in).squeeze(-1)
                    y = pursuer_rewards[:, i] + gamma * (1.0 - done) * target_q

                critic_loss = F.mse_loss(q_value, y)
                self.pursuer_critic_optim[i].zero_grad()
                critic_loss.backward()
                nn.utils.clip_grad_norm_(self.pursuer_critics[i].parameters(), 5.0)
                self.pursuer_critic_optim[i].step()

                if policy_update_due:
                    actor_joint = pursuer_actions.clone()
                    ai = self.pursuer_actors[i](pursuer_obs[:, i, :])
                    start = 2 * i
                    actor_joint[:, start : start + 2] = ai
                    actor_in = torch.cat([state, actor_joint], dim=-1)
                    actor_loss = -self.pursuer_critics[i](actor_in).mean()

                    self.pursuer_actor_optim[i].zero_grad()
                    actor_loss.backward()
                    nn.utils.clip_grad_norm_(self.pursuer_actors[i].parameters(), 5.0)
                    self.pursuer_actor_optim[i].step()

                    self._soft_update(self.pursuer_actors[i], self.pursuer_target_actors[i], tau)
                    self._soft_update(self.pursuer_critics[i], self.pursuer_target_critics[i], tau)

        if self.evader_training_enabled and self.evader_algo == "ddpg" and self.num_evaders > 0:
            evader_obs = batch["evader_obs"][:, 0, :]
            next_evader_obs = batch["next_evader_obs"][:, 0, :]
            evader_action = batch["evader_action"]
            evader_reward = batch["evader_reward"].view(-1)

            critic_in = torch.cat([evader_obs, evader_action], dim=-1)
            q_value = self.evader_critic(critic_in).squeeze(-1)

            with torch.no_grad():
                next_action = self.evader_target_actor(next_evader_obs)
                target_critic_in = torch.cat([next_evader_obs, next_action], dim=-1)
                target_q = self.evader_target_critic(target_critic_in).squeeze(-1)
                y = evader_reward + gamma * (1.0 - done) * target_q

            critic_loss = F.mse_loss(q_value, y)
            self.evader_critic_optim.zero_grad()
            critic_loss.backward()
            nn.utils.clip_grad_norm_(self.evader_critic.parameters(), 5.0)
            self.evader_critic_optim.step()

            if policy_update_due:
                pred_action = self.evader_actor(evader_obs)
                actor_in = torch.cat([evader_obs, pred_action], dim=-1)
                actor_loss = -self.evader_critic(actor_in).mean()

                self.evader_actor_optim.zero_grad()
                actor_loss.backward()
                nn.utils.clip_grad_norm_(self.evader_actor.parameters(), 5.0)
                self.evader_actor_optim.step()

                self._soft_update(self.evader_actor, self.evader_target_actor, tau)
                self._soft_update(self.evader_critic, self.evader_target_critic, tau)

    def _finalize_episode(self) -> None:
        self.episode_index += 1

        latest_path, pursuer_best_path, evader_best_path = self._checkpoint_paths()
        if self.pursuer_training_enabled or self.evader_training_enabled:
            self._save_checkpoint(latest_path)

        if self.episode_return_p > self.best_pursuer_return:
            self.best_pursuer_return = self.episode_return_p
            self._save_checkpoint(pursuer_best_path)

        if self.episode_return_e > self.best_evader_return:
            self.best_evader_return = self.episode_return_e
            self._save_checkpoint(evader_best_path)

        self.get_logger().info(
            "Episode %d done reason=%d, pursuer_return=%.3f, evader_return=%.3f, replay=%d"
            % (
                self.episode_index,
                self.latest_done_reason,
                self.episode_return_p,
                self.episode_return_e,
                len(self.replay),
            )
        )

        self.episode_return_p = 0.0
        self.episode_return_e = 0.0
        self.prev_snapshot = None
        self.done_latched = False
        self.local_episode_step_count = 0

    def control_tick(self) -> None:
        if self.num_evaders <= 0:
            return

        global_state = self._build_global_state()
        pursuer_obs = self._build_pursuer_obs()
        evader_obs = self._build_evader_obs()
        if global_state is None or pursuer_obs is None or evader_obs is None:
            return

        pursuer_actions = self._policy_pursuer_actions(pursuer_obs)
        evader_actions = self._policy_evader_actions(evader_obs)

        for pid in range(self.num_pursuers):
            self._publish_action(pid, pursuer_actions[pid])
        for eidx in range(self.num_evaders):
            eid = self.num_pursuers + eidx
            self._publish_action(eid, evader_actions[eidx])

        pursuer_rewards, evader_rewards, _captured = self._compute_rewards()

        if self.prev_snapshot is not None:
            transition = {
                "state": self.prev_snapshot["state"],
                "next_state": global_state,
                "pursuer_obs": self.prev_snapshot["pursuer_obs"],
                "next_pursuer_obs": pursuer_obs,
                "evader_obs": self.prev_snapshot["evader_obs"],
                "next_evader_obs": evader_obs,
                "pursuer_actions": self.prev_snapshot["pursuer_actions"],
                "evader_action": self.prev_snapshot["evader_action"],
                "pursuer_rewards": pursuer_rewards.astype(np.float32),
                "evader_reward": np.asarray(evader_rewards[0] if len(evader_rewards) else 0.0, dtype=np.float32),
                "done": np.asarray(1.0 if self.done_latched else 0.0, dtype=np.float32),
            }
            self.replay.add(transition)
            self.total_env_steps += 1
            self.local_episode_step_count += 1
            self.episode_return_p += float(np.mean(pursuer_rewards)) if len(pursuer_rewards) else 0.0
            self.episode_return_e += float(evader_rewards[0]) if len(evader_rewards) else 0.0

            if self.pursuer_training_enabled or self.evader_training_enabled:
                updates = self.get_parameter("train_updates_per_step").get_parameter_value().integer_value
                for _ in range(max(1, updates)):
                    self._train_step()

            horizon_steps = self.get_parameter("optimize_horizon_steps").get_parameter_value().integer_value
            if self.local_episode_step_count >= max(1, horizon_steps):
                self.latest_done_reason = 3
                self.done_latched = True

        self.prev_snapshot = {
            "state": global_state.astype(np.float32),
            "pursuer_obs": pursuer_obs.astype(np.float32),
            "evader_obs": evader_obs.astype(np.float32),
            "pursuer_actions": pursuer_actions.astype(np.float32),
            "evader_action": evader_actions[0].astype(np.float32)
            if len(evader_actions)
            else np.zeros(2, dtype=np.float32),
        }

        if self.done_latched:
            self._finalize_episode()


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
