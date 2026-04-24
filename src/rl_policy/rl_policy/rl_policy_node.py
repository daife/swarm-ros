import copy
import math
import os
import random
from dataclasses import dataclass
from collections import deque
from functools import partial
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32
from visualization_msgs.msg import Marker, MarkerArray

from .controllers import ControllerInput, EvaderController, PursuerController

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


@dataclass
class ObservationSnapshot:
    positions: np.ndarray
    velocities: np.ndarray
    obstacle_rel: np.ndarray


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
        self.obstacles: List[Marker] = []
        self.collision_flags: Dict[int, bool] = {i: False for i in range(self.total_agents)}
        self.cmd_publishers: Dict[int, object] = {}
        self.synthetic_obs_warned_ids = set()

        # Update observation dimensions based on TODO requirements
        # Evader: [pos(2), vel(2), rel_obs(2), rel_pursuers(2*num_p)]
        self.evader_obs_dim = 6 + 2 * self.num_pursuers
        # Pursuer: 
        # Individual: [pos(2), vel(2), rel_obs(2), rel_target(2)] -> 8
        # Collaborative: [rel_p_j(2), rvel_p_j(2), rel_obs_p_j(2), rel_target_p_j(2)] -> 8 * (num_p - 1)
        self.pursuer_obs_dim = 8 * self.num_pursuers
        self.global_state_dim = 4 * self.total_agents

        for agent_id in range(self.total_agents):
            self.create_subscription(Odometry, f"/agent_{agent_id}/odom", partial(self.on_odom, agent_id), 10)
            self.create_subscription(
                Bool,
                f"/agent_{agent_id}/collision",
                partial(self.on_collision, agent_id),
                10,
            )
            self.cmd_publishers[agent_id] = self.create_publisher(Accel, f"/agent_{agent_id}/cmd_accel", 10)

        obstacle_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MarkerArray, "/world_manager/obstacles", self.on_obstacles, obstacle_qos)
        self.create_subscription(Int32, "/agent_manager/reset_done", self.on_agent_reset_done, 10)
        self.create_subscription(Int32, "/scheduler/reset_begin", self.on_reset_begin, 10)
        self.create_subscription(Int32, "/scheduler/episode_start", self.on_episode_start, 10)
        self.create_subscription(Float32MultiArray, "/rl/episode_metrics", self.on_episode_metrics, 10)
        self.reset_ready_pub = self.create_publisher(Int32, "/rl_policy/reset_ready", 10)

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
        self.pending_ready_reset_id: Optional[int] = None
        self.last_ready_reset_id = 0
        self.last_agent_reset_id = 0
        self.last_episode_start_id = 0
        self.waiting_for_episode_start = True

        self.device = self._select_torch_device()
        self._init_models()
        self._init_controllers()

        self.create_timer(0.1, self.control_tick)

    def _init_controllers(self) -> None:
        self.pursuer_controller = PursuerController(
            algo=self.pursuer_algo,
            maddpg_forward=self._forward_pursuer_maddpg_action,
        )
        self.evader_controller = EvaderController(
            algo=self.evader_algo,
            ddpg_forward=self._forward_evader_ddpg_action,
        )

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

        self.pursuer_algo = self.pursuer_algo.strip().lower()
        self.evader_algo = self.evader_algo.strip().lower()

        if self.pursuer_algo == "cbc_tp_net":
            self.get_logger().warning("cbc_tp_net暂未实现，追捕者控制回退到flocking")
            self.pursuer_algo = "flocking"

        if self.pursuer_algo not in {"maddpg", "flocking"}:
            self.get_logger().warning(f"Unknown pursuer_algo={self.pursuer_algo}, fallback to maddpg")
            self.pursuer_algo = "maddpg"

        if self.evader_algo not in {"ddpg", "flocking"}:
            self.get_logger().warning(f"Unknown evader_algo={self.evader_algo}, fallback to ddpg")
            self.evader_algo = "ddpg"

        if self.pursuer_algo != "maddpg" and self.pursuer_training_enabled:
            self.get_logger().warning(
                f"pursuer_algo={self.pursuer_algo} 不支持参数训练，自动关闭 pursuer training"
            )
            self.pursuer_training_enabled = False

        if self.evader_algo != "ddpg" and self.evader_training_enabled:
            self.get_logger().warning(
                f"evader_algo={self.evader_algo} 不支持参数训练，自动关闭 evader training"
            )
            self.evader_training_enabled = False

        pursuer_actor_lr = self.get_parameter("pursuer_actor_lr").get_parameter_value().double_value
        pursuer_critic_lr = self.get_parameter("pursuer_critic_lr").get_parameter_value().double_value
        evader_actor_lr = self.get_parameter("evader_actor_lr").get_parameter_value().double_value
        evader_critic_lr = self.get_parameter("evader_critic_lr").get_parameter_value().double_value

        joint_action_dim = 2 * self.num_pursuers
        critic_input_dim = self.global_state_dim + joint_action_dim

        self.pursuer_actors = [Actor(self.pursuer_obs_dim, 2).to(self.device) for _ in range(self.num_pursuers)]
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

        self.evader_actor = Actor(self.evader_obs_dim, 2).to(self.device)
        self.evader_target_actor = copy.deepcopy(self.evader_actor)
        self.evader_critic = Critic(self.evader_obs_dim + 2).to(self.device)
        self.evader_target_critic = copy.deepcopy(self.evader_critic)
        self.evader_actor_optim = torch.optim.Adam(self.evader_actor.parameters(), lr=evader_actor_lr)
        self.evader_critic_optim = torch.optim.Adam(self.evader_critic.parameters(), lr=evader_critic_lr)

        pursuer_loaded = True
        evader_loaded = True
        if self.pursuer_algo == "maddpg" and self.pursuer_train_mode in {"train_load", "eval_only"}:
            if self.pursuer_weights:
                pursuer_loaded = self._load_checkpoint(self.pursuer_weights)
            else:
                pursuer_loaded = False

        if self.evader_algo == "ddpg" and self.evader_train_mode in {"train_load", "eval_only"}:
            if self.evader_weights:
                evader_loaded = self._load_checkpoint(self.evader_weights)
            else:
                evader_loaded = False

        if self.pursuer_algo == "maddpg" and self.pursuer_train_mode in {"train_load", "eval_only"} and not pursuer_loaded:
            self.get_logger().warning(
                "Pursuer checkpoint unavailable, fallback to flocking controller"
            )
            self.pursuer_algo = "flocking"
            self.pursuer_training_enabled = False

        if self.evader_algo == "ddpg" and self.evader_train_mode in {"train_load", "eval_only"} and not evader_loaded:
            self.get_logger().warning(
                "Evader checkpoint unavailable, fallback to flocking controller"
            )
            self.evader_algo = "flocking"
            self.evader_training_enabled = False

        if self.pursuer_algo != "maddpg" and self.pursuer_weights:
            self.get_logger().warning(
                f"pursuer_algo={self.pursuer_algo} 不使用 pursuer_weights，已忽略"
            )
        if self.evader_algo != "ddpg" and self.evader_weights:
            self.get_logger().warning(
                f"evader_algo={self.evader_algo} 不使用 evader_weights，已忽略"
            )

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

    def _load_checkpoint(self, path: str) -> bool:
        if torch is None:
            return False
        if not os.path.exists(path):
            self.get_logger().warning(f"Checkpoint not found: {path}")
            return False

        payload = torch.load(path, map_location=self.device)
        if not isinstance(payload, dict):
            self.get_logger().warning(f"Unsupported checkpoint format: {path}")
            return False

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
            return True

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
        return True

    def on_odom(self, agent_id: int, msg: Odometry) -> None:
        self.states[agent_id] = msg

    def on_obstacles(self, msg: MarkerArray) -> None:
        self.obstacles = list(msg.markers)

    def _get_closest_obstacle_surface_rel(self, agent_pos: Tuple[float, float]) -> Tuple[float, float]:
        if not self.obstacles:
            return 0.0, 0.0

        ax, ay = agent_pos
        min_dist_sq = float("inf")
        closest_rel = (0.0, 0.0)

        for mk in self.obstacles:
            ox = mk.pose.position.x
            oy = mk.pose.position.y
            sx = mk.scale.x
            sy = mk.scale.y

            if mk.type == Marker.CUBE:
                # Closest point on rectangle surface
                half_w = sx / 2.0
                half_l = sy / 2.0
                
                # Clamp agent position to rectangle bounds
                cx = max(ox - half_w, min(ax, ox + half_w))
                cy = max(oy - half_l, min(ay, oy + half_l))
                
                dist_sq = (ax - cx)**2 + (ay - cy)**2
                
                # If agent is inside, cx, cy == ax, ay. We need closest edge.
                if dist_sq < 1e-7:
                    dx_left = ax - (ox - half_w)
                    dx_right = (ox + half_w) - ax
                    dy_bottom = ay - (oy - half_l)
                    dy_top = (oy + half_l) - ay
                    min_edge_dist = min(dx_left, dx_right, dy_bottom, dy_top)
                    if min_edge_dist == dx_left:
                        cx, cy = ox - half_w, ay
                    elif min_edge_dist == dx_right:
                        cx, cy = ox + half_w, ay
                    elif min_edge_dist == dy_bottom:
                        cx, cy = ax, oy - half_l
                    else:
                        cx, cy = ax, oy + half_l
                    dist_sq = min_edge_dist**2

                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_rel = (cx - ax, cy - ay)

            elif mk.type == Marker.CYLINDER:
                # Closest point on cylinder surface
                radius = sx / 2.0
                dx = ax - ox
                dy = ay - oy
                dist_center = math.hypot(dx, dy)
                
                if dist_center < 1e-7:
                    cx, cy = ox + radius, oy # Arbitrary direction if exactly at center
                    dist_sq = radius**2
                else:
                    cx = ox + (dx / dist_center) * radius
                    cy = oy + (dy / dist_center) * radius
                    dist_sq = (ax - cx)**2 + (ay - cy)**2
                
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_rel = (cx - ax, cy - ay)

        return closest_rel

    def on_collision(self, agent_id: int, msg: Bool) -> None:
        if msg.data:
            self.collision_flags[agent_id] = True

    def _publish_zero_actions(self) -> None:
        cmd = Accel()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        for pub in self.cmd_publishers.values():
            pub.publish(cmd)

    def _pause_for_reset(self) -> None:
        self.waiting_for_episode_start = True
        self.prev_snapshot = None
        self._publish_zero_actions()

    def on_reset_begin(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if reset_id <= 0:
            return
        self._pause_for_reset()

    def on_agent_reset_done(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if reset_id <= 0:
            return

        self.last_agent_reset_id = max(self.last_agent_reset_id, reset_id)
        self.pending_ready_reset_id = reset_id
        self._pause_for_reset()

    def on_episode_start(self, msg: Int32) -> None:
        reset_id = int(msg.data)
        if reset_id <= 0:
            return
        if reset_id != self.last_agent_reset_id:
            return

        self.last_episode_start_id = reset_id
        self.waiting_for_episode_start = False
        self.prev_snapshot = None

    def _publish_reset_ready_if_needed(self) -> None:
        if self.pending_ready_reset_id is None:
            return

        reset_id = int(self.pending_ready_reset_id)
        if reset_id <= self.last_ready_reset_id:
            self.pending_ready_reset_id = None
            return

        msg = Int32()
        msg.data = reset_id
        self.reset_ready_pub.publish(msg)
        self.last_ready_reset_id = reset_id
        self.pending_ready_reset_id = None
        self.get_logger().info(f"Published /rl_policy/reset_ready reset_id={reset_id}")

    def on_episode_metrics(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self.latest_done_reason = int(msg.data[2])
            self.done_latched = True
            self._pause_for_reset()

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
            if agent_id not in self.synthetic_obs_warned_ids:
                self.synthetic_obs_warned_ids.add(agent_id)
                self.get_logger().warning(
                    f"agent_{agent_id} odom missing, fallback to synthetic observation"
                )
            return self._synthetic_kinematics(agent_id)
        return None

    def _build_observation_snapshot(self) -> Optional[ObservationSnapshot]:
        positions = np.zeros((self.total_agents, 2), dtype=np.float32)
        velocities = np.zeros((self.total_agents, 2), dtype=np.float32)
        obstacle_rel = np.zeros((self.total_agents, 2), dtype=np.float32)

        for aid in range(self.total_agents):
            kin = self._agent_kinematics(aid)
            if kin is None:
                return None
            x, y, vx, vy = kin
            positions[aid] = [x, y]
            velocities[aid] = [vx, vy]
            obstacle_rel[aid] = self._get_closest_obstacle_surface_rel((x, y))

        return ObservationSnapshot(positions=positions, velocities=velocities, obstacle_rel=obstacle_rel)

    def _build_global_state(self, snapshot: ObservationSnapshot) -> np.ndarray:
        return np.concatenate([snapshot.positions, snapshot.velocities], axis=1).reshape(-1).astype(np.float32)

    def _relative_position(self, snapshot: ObservationSnapshot, src_id: int, dst_id: int) -> np.ndarray:
        return (snapshot.positions[dst_id] - snapshot.positions[src_id]).astype(np.float32)

    def _relative_velocity(self, snapshot: ObservationSnapshot, src_id: int, dst_id: int) -> np.ndarray:
        return (snapshot.velocities[dst_id] - snapshot.velocities[src_id]).astype(np.float32)

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

    def _build_pursuer_obs(self, snapshot: ObservationSnapshot) -> np.ndarray:
        obs_rows = []
        target_id = self.num_pursuers  # Assume first evader is the target

        for i in range(self.num_pursuers):
            px_i, py_i = snapshot.positions[i]
            pvx_i, pvy_i = snapshot.velocities[i]

            # Individual: [pos(2), vel(2), rel_obs(2), rel_target(2)]
            rel_target_i = self._relative_position(snapshot, i, target_id)
            row = [px_i, py_i, pvx_i, pvy_i, snapshot.obstacle_rel[i, 0], snapshot.obstacle_rel[i, 1], rel_target_i[0], rel_target_i[1]]

            # Collaborative state
            for j in range(self.num_pursuers):
                if i == j:
                    continue
                px_j, py_j = snapshot.positions[j]
                rel_vel_j = self._relative_velocity(snapshot, i, j)

                # Collaborative: [rel_pos(2), rel_vel(2), rel_obs_j(2), rel_target_j(2)]
                rel_target_j = self._relative_position(snapshot, j, target_id)
                row.extend([px_j - px_i, py_j - py_i, rel_vel_j[0], rel_vel_j[1], snapshot.obstacle_rel[j, 0], snapshot.obstacle_rel[j, 1], rel_target_j[0], rel_target_j[1]])
            
            obs_rows.append(row)

        if not obs_rows:
            return np.zeros((0, self.pursuer_obs_dim), dtype=np.float32)
        return np.asarray(obs_rows, dtype=np.float32)

    def _build_evader_obs(self, snapshot: ObservationSnapshot) -> np.ndarray:
        obs_rows = []
        for eidx in range(self.num_evaders):
            eid = self.num_pursuers + eidx
            ex, ey = snapshot.positions[eid]
            evx, evy = snapshot.velocities[eid]

            # Evader: [pos(2), vel(2), rel_obs(2)]
            row = [ex, ey, evx, evy, snapshot.obstacle_rel[eid, 0], snapshot.obstacle_rel[eid, 1]]

            # Relative pursuers
            for pid in range(self.num_pursuers):
                rel_pos = self._relative_position(snapshot, eid, pid)
                row.extend([rel_pos[0], rel_pos[1]])
            
            obs_rows.append(row)

        if not obs_rows:
            return np.zeros((0, self.evader_obs_dim), dtype=np.float32)
        return np.asarray(obs_rows, dtype=np.float32)

    def _forward_pursuer_maddpg_action(self, idx: int, obs_row: np.ndarray) -> Optional[np.ndarray]:
        if torch is None or self.pursuer_algo != "maddpg" or not self.pursuer_actors:
            return None
        if idx >= len(self.pursuer_actors):
            return None

        obs_tensor = torch.tensor(obs_row, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            action_tensor = self.pursuer_actors[idx](obs_tensor)
        return action_tensor.squeeze(0).cpu().numpy().astype(np.float32)

    def _forward_evader_ddpg_action(self, obs_row: np.ndarray) -> Optional[np.ndarray]:
        if torch is None or self.evader_algo != "ddpg" or self.evader_actor is None:
            return None

        obs_tensor = torch.tensor(obs_row, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            action_tensor = self.evader_actor(obs_tensor)
        return action_tensor.squeeze(0).cpu().numpy().astype(np.float32)

    def _policy_pursuer_actions(self, pursuer_obs: np.ndarray) -> np.ndarray:
        noise_std = self.get_parameter("exploration_noise_std").get_parameter_value().double_value
        ctrl_input = ControllerInput(
            obs=pursuer_obs,
            training_enabled=self.pursuer_training_enabled,
            noise_std=float(noise_std),
        )
        return self.pursuer_controller.compute_actions(ctrl_input)

    def _policy_evader_actions(self, evader_obs: np.ndarray) -> np.ndarray:
        noise_std = self.get_parameter("exploration_noise_std").get_parameter_value().double_value
        ctrl_input = ControllerInput(
            obs=evader_obs,
            training_enabled=self.evader_training_enabled,
            noise_std=float(noise_std),
        )
        return self.evader_controller.compute_actions(ctrl_input)

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
        self._publish_reset_ready_if_needed()

        if self.waiting_for_episode_start:
            self._publish_zero_actions()
            if self.done_latched:
                self._finalize_episode()
            self.prev_snapshot = None
            return

        if self.num_evaders <= 0:
            return

        snapshot = self._build_observation_snapshot()
        if snapshot is None:
            return

        global_state = self._build_global_state(snapshot)
        pursuer_obs = self._build_pursuer_obs(snapshot)
        evader_obs = self._build_evader_obs(snapshot)

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
