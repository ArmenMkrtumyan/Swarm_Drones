"""
Gymnasium wrapper around `CoverageEnv` for MARL training.

Single-agent gym interface with a *centralized* joint policy: action and
observation tensors stack the n_drones drones along the leading axis.
This is the simplest path to use Stable-Baselines3 PPO directly without
multi-agent infrastructure (PettingZoo, RLlib). At inference the same
policy network is queried once per env.step with the joint observation;
parameters are shared across drones because the policy doesn't know
which drone is which — it sees a flattened vector.

Observation per drone (32 dims, all scaled to roughly [-1, 1]):
    [0, 1]    own pos / world size
    [2, 3]    own vel / max_speed
    [4]       own heading / π
    [5]       own battery fraction (1 = full, 0 = cutoff)
    [6, 30]   5×5 local coverage mask (centered on drone), 0/1
              with walls reported as 1 (so the agent treats them like
              "already covered" — neither penalty nor reward to revisit)
    [31]      global coverage fraction
Joint obs: shape (n_drones * 32,) — flat float32.

Action per drone: 2D acceleration in [-1, 1] (x and y), scaled to
[-max_accel, max_accel]. Yaw is tracked to velocity automatically (same
as PF / Consensus controllers — we don't ask the policy to learn yaw).
Joint action: shape (n_drones * 2,).

Reward per step:
    + coverage_delta_cells * 1.0       new free cells covered this step
    - 0.01                              time penalty (encourage finishing)
    + 100.0 on terminal if coverage = 100 %
    - 20.0 on terminal if every drone depleted before reaching 100 %

Episode terminates when env.is_terminal() OR step cap reached.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "MARL training requires gymnasium. Install with: "
        "pip install gymnasium stable-baselines3 torch"
    ) from e

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from environment import CoverageEnv, DroneConfig, SimConfig
from maze import FREE, WALL, random_obstacles, recursive_backtracker


# Local coverage window radius — drone sees a (2*WIN+1) × (2*WIN+1) grid
# around itself. WIN=2 → 5×5 = 25 cells.
LOCAL_WIN = 2
LOCAL_DIM = (2 * LOCAL_WIN + 1) ** 2

# Per-drone observation length: pos(2) + vel(2) + heading(1) + battery(1)
# + local_coverage(LOCAL_DIM) + global_coverage(1)
PER_DRONE_OBS = 2 + 2 + 1 + 1 + LOCAL_DIM + 1


class CoverageGymEnv(gym.Env):
    """
    Gymnasium wrapper. The underlying `CoverageEnv` is rebuilt each
    `reset()` so PPO sees random maps within the configured generator.

    Args:
        grid_size: side length of the square map (cells).
        n_drones:  swarm size. Action/obs shapes scale with this.
        max_steps: hard cap on steps per episode (truncates if env.step
                   doesn't terminate naturally first).
        map_kind:  "random" (random obstacles) or "maze" (recursive
                   backtracker) — generator used for each fresh reset.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        grid_size: int = 11,
        n_drones: int = 3,
        max_steps: int = 1500,
        map_kind: str = "random",
        seed: Optional[int] = None,
    ) -> None:
        super().__init__()
        self.grid_size = grid_size
        self.n_drones = n_drones
        self.max_steps = max_steps
        self.map_kind = map_kind
        self._rng = np.random.default_rng(seed)

        self.observation_space = spaces.Box(
            low=-3.0, high=3.0,
            shape=(n_drones * PER_DRONE_OBS,),
            dtype=np.float32,
        )
        # Per-drone 2D acceleration in [-1, 1]; scaled to max_accel inside step.
        self.action_space = spaces.Box(
            low=-1.0, high=1.0,
            shape=(n_drones * 2,),
            dtype=np.float32,
        )

        self.env: Optional[CoverageEnv] = None
        self.steps = 0
        self._covered_count_prev = 0

    # ------------------------------------------------------------------
    # gym API
    # ------------------------------------------------------------------

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None
              ) -> tuple[np.ndarray, dict[str, Any]]:
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        # Pick a per-episode map seed so PPO sees varied maps.
        map_seed = int(self._rng.integers(0, 2**31 - 1))
        if self.map_kind == "maze":
            grid = recursive_backtracker(self.grid_size, seed=map_seed)
        else:
            grid = random_obstacles(self.grid_size, density=0.20, seed=map_seed)
        self.env = CoverageEnv(
            grid=grid,
            n_drones=self.n_drones,
            sim=SimConfig(),
            drone=DroneConfig(),
        )
        self.env.reset(seed=map_seed)
        self.steps = 0
        self._covered_count_prev = int(self.env.covered.sum())
        return self._observe(), {}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        env = self.env
        assert env is not None, "step() before reset()"

        # Scale action from [-1, 1] to [-max_accel, max_accel] and reshape.
        a = np.asarray(action, dtype=np.float64).reshape(self.n_drones, 2)
        a = a * env.drone_cfg.max_accel

        # Pad with yaw=0 — the env accepts (n, 2) and treats yaw as 0.
        # Yaw tracking happens via the env's existing dynamics + we also
        # set heading = atan2(vel) before each step so the wedge sweeps
        # the path (keeps coverage aligned with motion, same as PF).
        for i, drone in enumerate(env.drones):
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > 0.05:
                drone.heading = float(np.arctan2(v[1], v[0]))
                drone.yaw_rate = 0.0

        env.step(a)
        self.steps += 1

        # ---- reward ----
        covered_now = int(env.covered.sum())
        delta_cells = covered_now - self._covered_count_prev
        self._covered_count_prev = covered_now
        reward = float(delta_cells) - 0.01  # coverage gain - time penalty

        terminated = bool(env.is_terminal())
        truncated = self.steps >= self.max_steps

        if terminated:
            if env.is_done():
                reward += 100.0
            elif env.all_depleted():
                # Penalty proportional to how far we got (less penalty if we
                # at least covered a lot before dying).
                cov_pct = env.coverage_fraction()
                reward += -20.0 * (1.0 - cov_pct)

        info = {
            "coverage": env.coverage_fraction(),
            "steps": self.steps,
            "delta_cells": delta_cells,
            "is_done": env.is_done(),
            "all_depleted": env.all_depleted(),
        }
        return self._observe(), reward, terminated, truncated, info

    # ------------------------------------------------------------------
    # observation builder
    # ------------------------------------------------------------------

    def _observe(self) -> np.ndarray:
        env = self.env
        assert env is not None
        out = np.zeros(self.n_drones * PER_DRONE_OBS, dtype=np.float32)
        max_speed = env.drone_cfg.max_speed
        h, w = env.grid.shape
        global_cov = float(env.coverage_fraction())
        # Coverage mask AS the agent should see it — covered or wall both
        # read as "skip". Both are useless to revisit; treating walls and
        # covered cells the same simplifies the learning problem.
        covered_or_wall = (env.covered | (env.grid == WALL)).astype(np.float32)

        for i, drone in enumerate(env.drones):
            base = i * PER_DRONE_OBS
            out[base + 0] = drone.pos[0] / w
            out[base + 1] = drone.pos[1] / h
            out[base + 2] = drone.vel[0] / max_speed
            out[base + 3] = drone.vel[1] / max_speed
            out[base + 4] = drone.heading / np.pi
            # Battery fraction over the *spendable* range (0 = cutoff, 1 = full).
            cfg = env.battery_cfg
            spendable = cfg.initial_energy_j - cfg.cutoff_energy_j
            remaining_above_cutoff = max(0.0, drone.battery_j - cfg.cutoff_energy_j)
            out[base + 5] = (
                remaining_above_cutoff / spendable if spendable > 0 else 0.0
            )

            # Local coverage mask (5×5 by default). Out-of-bounds cells read
            # as 1 (treat as already covered / unreachable).
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
