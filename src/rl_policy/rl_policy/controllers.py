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
        # Input contract is identical to MADDPG local obs: [dx, dy, vx, vy, tx, ty].
        dx = float(obs_row[0])
        dy = float(obs_row[1])
        vx = float(obs_row[2])
        vy = float(obs_row[3])

        chase_vec = _normalize(np.asarray([dx, dy], dtype=np.float32))
        damping_vec = -0.2 * np.asarray([vx, vy], dtype=np.float32)
        blended = chase_vec + damping_vec
        norm = float(np.linalg.norm(blended))
        if norm > 1.0:
            blended = blended / norm
        return blended.astype(np.float32)


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
        # Input contract is identical to DDPG local obs: [dx, dy, vx, vy, px, py].
        dx = float(obs_row[0])
        dy = float(obs_row[1])
        vx = float(obs_row[2])
        vy = float(obs_row[3])

        evade_vec = _normalize(np.asarray([dx, dy], dtype=np.float32))
        inertia_vec = _normalize(np.asarray([vx, vy], dtype=np.float32))
        blended = 0.85 * evade_vec + 0.15 * inertia_vec
        if math.hypot(float(blended[0]), float(blended[1])) < 1e-6:
            return np.zeros(2, dtype=np.float32)
        return _normalize(blended)
