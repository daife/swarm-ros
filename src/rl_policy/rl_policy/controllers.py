import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np


@dataclass
class ControllerInput:
    obs: np.ndarray
    training_enabled: bool
    noise_std: float


PursuerForwardFn = Callable[[int, np.ndarray], Optional[np.ndarray]]
EvaderForwardFn = Callable[[np.ndarray], Optional[np.ndarray]]


def _normalize(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm < 1e-6:
        return np.zeros(2, dtype=np.float32)
    return (v / norm).astype(np.float32)


def _sanitize_action(action: Optional[np.ndarray]) -> Optional[np.ndarray]:
    if action is None:
        return None
    arr = np.asarray(action, dtype=np.float32).reshape(-1)
    if arr.size < 2:
        return None
    return arr[:2]


def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


class PursuerController:
    def __init__(self, algo: str, maddpg_forward: PursuerForwardFn) -> None:
        normalized_algo = algo.strip().lower()
        if normalized_algo == "cbc_tp_net":
            normalized_algo = "flocking"
        if normalized_algo not in {"maddpg", "flocking"}:
            normalized_algo = "maddpg"

        self.algo = normalized_algo
        self._maddpg_forward = maddpg_forward

    def compute_actions(self, ctrl_input: ControllerInput) -> np.ndarray:
        obs = np.asarray(ctrl_input.obs, dtype=np.float32)
        if obs.size == 0:
            return np.zeros((0, 2), dtype=np.float32)

        actions = []
        for idx in range(obs.shape[0]):
            obs_row = obs[idx]

            action: Optional[np.ndarray] = None
            if self.algo == "maddpg":
                action = _sanitize_action(self._maddpg_forward(idx, obs_row))

            if action is None:
                action = self._flocking_action(obs_row)

            if self.algo == "maddpg" and ctrl_input.training_enabled and ctrl_input.noise_std > 0.0:
                action = action + np.random.normal(0.0, ctrl_input.noise_std, size=2).astype(np.float32)

            actions.append(np.clip(action, -1.0, 1.0).astype(np.float32))

        return np.asarray(actions, dtype=np.float32)

    def _flocking_action(self, obs_row: np.ndarray) -> np.ndarray:
        # Pursuer row format:
        # [px, py, pvx, pvy, rel_obs_x, rel_obs_y, rel_target_x, rel_target_y,
        #  (rel_pos_j_x, rel_pos_j_y, rel_vel_j_x, rel_vel_j_y, rel_obs_j_x, rel_obs_j_y, rel_target_j_x, rel_target_j_y)*]
        pos = np.asarray([obs_row[0], obs_row[1]], dtype=np.float32)
        vel = np.asarray([obs_row[2], obs_row[3]], dtype=np.float32)
        rel_obs = np.asarray([obs_row[4], obs_row[5]], dtype=np.float32)
        rel_target = np.asarray([obs_row[6], obs_row[7]], dtype=np.float32)

        chase = 1.15 * _normalize(rel_target)
        obstacle_avoid = self._obstacle_avoidance(rel_obs)
        teammate_avoid = np.zeros(2, dtype=np.float32)

        block_size = 8
        start = 8
        while start + block_size <= obs_row.size:
            rel_pos_j = np.asarray([obs_row[start], obs_row[start + 1]], dtype=np.float32)
            teammate_avoid += self._teammate_avoidance(rel_pos_j)
            start += block_size

        # Soft wall repulsion using known map scale from defaults ([-10, 10] region).
        wall = np.zeros(2, dtype=np.float32)
        wall_margin = 1.6
        half_extent = 10.0
        if pos[0] > half_extent - wall_margin:
            wall[0] -= (pos[0] - (half_extent - wall_margin)) / wall_margin
        if pos[0] < -half_extent + wall_margin:
            wall[0] += ((-half_extent + wall_margin) - pos[0]) / wall_margin
        if pos[1] > half_extent - wall_margin:
            wall[1] -= (pos[1] - (half_extent - wall_margin)) / wall_margin
        if pos[1] < -half_extent + wall_margin:
            wall[1] += ((-half_extent + wall_margin) - pos[1]) / wall_margin

        damping = -0.28 * vel
        blended = chase + obstacle_avoid + teammate_avoid + 0.9 * wall + damping
        if _norm(blended) < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return _normalize(blended)

    def _obstacle_avoidance(self, rel_obs: np.ndarray) -> np.ndarray:
        dist = _norm(rel_obs)
        if dist < 1e-6 or dist >= 2.6:
            return np.zeros(2, dtype=np.float32)
        weight = 1.1 * (1.0 / max(dist, 1e-3) - 1.0 / 2.6)
        return (-rel_obs / dist * weight).astype(np.float32)

    def _teammate_avoidance(self, rel_pos: np.ndarray) -> np.ndarray:
        dist = _norm(rel_pos)
        if dist < 1e-6 or dist >= 2.0:
            return np.zeros(2, dtype=np.float32)
        weight = 1.0 * (1.0 / max(dist, 1e-3) - 1.0 / 2.0)
        return (-rel_pos / dist * weight).astype(np.float32)


class EvaderController:
    def __init__(self, algo: str, ddpg_forward: EvaderForwardFn) -> None:
        normalized_algo = algo.strip().lower()
        if normalized_algo not in {"ddpg", "flocking"}:
            normalized_algo = "ddpg"

        self.algo = normalized_algo
        self._ddpg_forward = ddpg_forward

    def compute_actions(self, ctrl_input: ControllerInput) -> np.ndarray:
        obs = np.asarray(ctrl_input.obs, dtype=np.float32)
        if obs.size == 0:
            return np.zeros((0, 2), dtype=np.float32)

        actions = []
        for idx in range(obs.shape[0]):
            obs_row = obs[idx]

            action: Optional[np.ndarray] = None
            if self.algo == "ddpg":
                action = _sanitize_action(self._ddpg_forward(obs_row))

            if action is None:
                action = self._flocking_action(obs_row)

            if self.algo == "ddpg" and ctrl_input.training_enabled and ctrl_input.noise_std > 0.0:
                action = action + np.random.normal(0.0, ctrl_input.noise_std, size=2).astype(np.float32)

            actions.append(np.clip(action, -1.0, 1.0).astype(np.float32))

        return np.asarray(actions, dtype=np.float32)

    def _flocking_action(self, obs_row: np.ndarray) -> np.ndarray:
        # Evader row format: [ex, ey, evx, evy, rel_obs_x, rel_obs_y, rel_p1_x, rel_p1_y, ...]
        pos = np.asarray([obs_row[0], obs_row[1]], dtype=np.float32)
        vel = np.asarray([obs_row[2], obs_row[3]], dtype=np.float32)
        rel_obs = np.asarray([obs_row[4], obs_row[5]], dtype=np.float32)

        evade = np.zeros(2, dtype=np.float32)
        start = 6
        while start + 1 < obs_row.size:
            rel_p = np.asarray([obs_row[start], obs_row[start + 1]], dtype=np.float32)
            # rel_p is pursuer-relative (pursuer - evader), so use negative for evasion.
            evade += self._pursuer_repulsion(rel_p)
            start += 2

        obstacle_avoid = self._obstacle_avoidance(rel_obs)

        wall = np.zeros(2, dtype=np.float32)
        wall_margin = 1.7
        half_extent = 10.0
        if pos[0] > half_extent - wall_margin:
            wall[0] -= (pos[0] - (half_extent - wall_margin)) / wall_margin
        if pos[0] < -half_extent + wall_margin:
            wall[0] += ((-half_extent + wall_margin) - pos[0]) / wall_margin
        if pos[1] > half_extent - wall_margin:
            wall[1] -= (pos[1] - (half_extent - wall_margin)) / wall_margin
        if pos[1] < -half_extent + wall_margin:
            wall[1] += ((-half_extent + wall_margin) - pos[1]) / wall_margin

        damping = -0.18 * vel
        blended = 1.2 * evade + obstacle_avoid + 0.95 * wall + damping
        if math.hypot(float(blended[0]), float(blended[1])) < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return _normalize(blended)

    def _pursuer_repulsion(self, rel_p: np.ndarray) -> np.ndarray:
        dist = _norm(rel_p)
        if dist < 1e-6 or dist >= 6.0:
            return np.zeros(2, dtype=np.float32)
        weight = 1.35 * (1.0 / max(dist, 1e-3) - 1.0 / 6.0)
        return (-rel_p / dist * weight).astype(np.float32)

    def _obstacle_avoidance(self, rel_obs: np.ndarray) -> np.ndarray:
        dist = _norm(rel_obs)
        if dist < 1e-6 or dist >= 2.8:
            return np.zeros(2, dtype=np.float32)
        weight = 1.25 * (1.0 / max(dist, 1e-3) - 1.0 / 2.8)
        return (-rel_obs / dist * weight).astype(np.float32)
