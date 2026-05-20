"""
MARL controller: wraps a trained SB3 PPO checkpoint as a `policy_fn`
matching the same `(env: CoverageEnv) -> actions` contract used by
`PotentialFieldsController` and `ConsensusController`.

The checkpoint must have been trained against `CoverageGymEnv` with the
same `n_drones` and grid scale — observation construction must agree
exactly between training and inference. We rebuild the observation here
in raw NumPy (no Gymnasium env on the inference path) so this controller
can be dropped into any `CoverageEnv` instance.

Yaw control is identical to PF / Consensus: heading tracks velocity so
the wedge sweeps the path. The trained policy only emits 2D acceleration.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from controllers.marl_env import LOCAL_DIM, LOCAL_WIN, PER_DRONE_OBS
from environment import CoverageEnv
from maze import WALL


class MARLController:
    """
    Loads a trained PPO model and runs it as a swarm policy.

    Args:
        checkpoint: path to a `model.zip` saved by `train_marl.py`. If
            `None`, defaults to `outputs/marl_ppo.zip`.
        deterministic: PPO action sampling. `True` (default) uses the
            mean of the policy distribution — better for evaluation.
        hover_drone_idx: forces this drone to zero acceleration so the
            demo's hover-verify sanity check still works. Set to `None`
            for full evaluation (every drone moves).
    """

    def __init__(
        self,
        checkpoint: Optional[str] = None,
        deterministic: bool = True,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        try:
            from stable_baselines3 import PPO
        except ImportError as e:  # pragma: no cover
            raise ImportError(
                "MARLController requires stable-baselines3. "
                "Install with: pip install stable-baselines3 torch"
            ) from e

        if checkpoint is None:
            from constants import MARL_DIR
            checkpoint = str(MARL_DIR / "marl_ppo.zip")
        path = Path(checkpoint)
        if not path.exists():
            raise FileNotFoundError(
                f"MARL checkpoint not found at {path}. "
                f"Run: python tools/train_marl.py"
            )
        self.model = PPO.load(str(path), device="cpu")
        self.deterministic = deterministic
        self.hover_drone_idx = hover_drone_idx

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        obs = self._observe(env)
        action, _ = self.model.predict(obs, deterministic=self.deterministic)
        # SB3 returns action in [-1, 1]; scale to [-max_accel, max_accel] and
        # reshape to (n_drones, 2). Yaw is computed below.
        a = np.asarray(action, dtype=np.float64).reshape(n, 2)
        a = a * env.drone_cfg.max_accel

        actions = np.zeros((n, 3), dtype=np.float64)
        actions[:, 0:2] = a

        # Force hover drone to zero (overrides whatever the policy emitted).
        if self.hover_drone_idx is not None and 0 <= self.hover_drone_idx < n:
            actions[self.hover_drone_idx] = 0.0

        # Yaw: same P-controller toward velocity direction as PF / Consensus.
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > 0.05:
                target_heading = math.atan2(v[1], v[0])
                err = (target_heading - drone.heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                actions[i, 2] = float(
                    np.clip(6.0 * err, -max_yaw_accel, max_yaw_accel)
                )
        return actions

    # ------------------------------------------------------------------
    # observation builder — must match controllers/marl_env.py exactly.
    # ------------------------------------------------------------------
    def _observe(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        out = np.zeros(n * PER_DRONE_OBS, dtype=np.float32)
        max_speed = env.drone_cfg.max_speed
        h, w = env.grid.shape
        global_cov = float(env.coverage_fraction())
        covered_or_wall = (env.covered | (env.grid == WALL)).astype(np.float32)
        cfg = env.battery_cfg
        spendable = cfg.initial_energy_j - cfg.cutoff_energy_j

        for i, drone in enumerate(env.drones):
            base = i * PER_DRONE_OBS
            out[base + 0] = drone.pos[0] / w
            out[base + 1] = drone.pos[1] / h
            out[base + 2] = drone.vel[0] / max_speed
            out[base + 3] = drone.vel[1] / max_speed
            out[base + 4] = drone.heading / np.pi
            remaining_above_cutoff = max(0.0, drone.battery_j - cfg.cutoff_energy_j)
            out[base + 5] = (
                remaining_above_cutoff / spendable if spendable > 0 else 0.0
            )

            cx, cy = int(drone.pos[0]), int(drone.pos[1])
            for dy in range(-LOCAL_WIN, LOCAL_WIN + 1):
                for dx in range(-LOCAL_WIN, LOCAL_WIN + 1):
                    yy, xx = cy + dy, cx + dx
                    idx = (dy + LOCAL_WIN) * (2 * LOCAL_WIN + 1) + (dx + LOCAL_WIN)
                    if 0 <= yy < h and 0 <= xx < w:
                        val = covered_or_wall[yy, xx]
                    else:
                        val = 1.0
                    out[base + 6 + idx] = val

            out[base + 6 + LOCAL_DIM] = global_cov
        return out
