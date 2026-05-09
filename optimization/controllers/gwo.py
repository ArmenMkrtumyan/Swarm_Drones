"""
Grey Wolf Optimizer controller for swarm coverage.

Following Mirjalili, Mirjalili & Lewis (2014). Wolves are ranked by fitness
into a strict hierarchy:

    α (alpha)  — best wolf (lowest distance to its nearest uncovered cell)
    β (beta)   — second best
    δ (delta)  — third best
    ω (omega)  — everyone else; positions updated toward a weighted mean
                 of α/β/δ via GWO's `A` and `C` random coefficients.

Each step, every ω wolf computes three "pulls" using the canonical GWO update:

    X_p = X_p − A · |C · X_p − X_i|       for p ∈ {α, β, δ}
    X_new = (X_α' + X_β' + X_δ') / 3
    a_command = X_new − X_i   (used as acceleration on the env)

with `A = 2a·r1 − a`, `C = 2·r2`, `r1, r2 ~ U(0, 1)`. The control parameter
`a` decays linearly from `a_initial` to `a_final` over the configured
`decay_steps` — large `a` permits |A|>1 (exploration: wolves can step *past*
the leaders), small `a` forces |A|<1 (exploitation: wolves converge onto
the leaders).

We use each wolf's own nearest uncovered cell as its current "position" in
the GWO formula (re-elected each step, like in PSO/SA/GA), which sidesteps
the dynamic-fitness issue. Wall repulsion + yaw-tracks-velocity follow the
same recipe as the other controllers.

Stateful (`_step_count` for the `a` schedule). Instantiate fresh per
`env.reset()`.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from environment import CoverageEnv
from maze import FREE, WALL


@dataclass
class GWOConfig:
    """Tunable knobs for `GWOController`."""
    # GWO control parameter schedule (linear decay over `decay_steps` steps).
    # Defaults from `tools/grid_search_gwo.py` 27-config sweep. GWO was the
    # weakest Track 2 algorithm — the leader-following formula tends to flock
    # drones onto α/β/δ targets, hurting spread on a coverage objective. Best
    # config: high `a0` keeps |A|>1 longer (drones can step *past* leaders),
    # plus a long `decay_steps` keeps that exploration alive.
    a_initial: float = 5.0
    a_final: float = 0.5
    decay_steps: int = 3000

    # Movement gains (only wall repel + yaw — GWO formula already encodes
    # the leader-pulling. Drone repel kept off here to let the GWO update
    # be the primary force.)
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


class GWOController:
    """GWO controller. Each drone is a wolf; α/β/δ leaders pull the rest."""

    def __init__(
        self,
        cfg: Optional[GWOConfig] = None,
        hover_drone_idx: Optional[int] = 0,
        seed: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or GWOConfig()
        self.hover_drone_idx = hover_drone_idx
        self._rng = np.random.default_rng(seed)
        self._step_count = 0
        self._last_rank = None        # ranking of drones by fitness this step
        self._last_targets = None     # nearest-uncov target per drone

    def reset(self) -> None:
        self._step_count = 0
        self._last_rank = None
        self._last_targets = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        roles = [None] * env.n_drones
        if self._last_rank is not None:
            tags = ("α", "β", "δ")
            for k, idx in enumerate(self._last_rank[:3]):
                if 0 <= idx < env.n_drones:
                    roles[idx] = tags[k]
        return {
            "targets": self._last_targets,
            "roles": roles,
            "title_extra": "Grey Wolf Optimizer",
        }

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered
        if not uncov_mask.any():
            return actions

        uy, ux = np.where(uncov_mask)
        uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])  # (M, 2)

        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])

        # Each wolf's "fitness" = inverse of distance to nearest uncov cell.
        # We just track the nearest cell as the wolf's *target* in the GWO
        # update — α has the best target, β second, etc.
        nearest_dist = np.zeros(n)
        nearest_target = np.zeros((n, 2))
        for i in range(n):
            d2 = ((uncov_pos - positions[i]) ** 2).sum(axis=1)
            j = int(d2.argmin())
            nearest_dist[i] = math.sqrt(d2[j])
            nearest_target[i] = uncov_pos[j]

        rank = np.argsort(nearest_dist)  # ascending: best first
        alpha = nearest_target[rank[0]]
        beta = nearest_target[rank[1]] if n > 1 else alpha
        delta = nearest_target[rank[2]] if n > 2 else beta
        self._last_rank = rank.tolist()
        self._last_targets = nearest_target.copy()

        # Linear decay of control parameter `a`.
        cfg = self.cfg
        progress = min(1.0, self._step_count / max(1, cfg.decay_steps))
        a = cfg.a_initial - progress * (cfg.a_initial - cfg.a_final)
        self._step_count += 1

        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel

        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            pos = positions[i]
            # Three GWO pulls toward α / β / δ targets.
            X_pulls = []
            for leader in (alpha, beta, delta):
                r1 = self._rng.uniform(0.0, 1.0, size=2)
                r2 = self._rng.uniform(0.0, 1.0, size=2)
                A = 2 * a * r1 - a
                C = 2 * r2
                X = leader - A * np.abs(C * leader - pos)
                X_pulls.append(X)
            X_new = sum(X_pulls) / 3.0

            # Use the proposed displacement as acceleration command.
            accel = X_new - pos

            # Wall repel
            if has_walls:
                d_wall = pos - wall_pos
                wd = np.linalg.norm(d_wall, axis=1)
                in_w = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_w.any():
                    weights = 1.0 / np.maximum(wd[in_w] ** 2, 0.05)
                    accel = accel + cfg.wall_repel_gain * (
                        d_wall[in_w].T * weights
                    ).sum(axis=1)

            # Saturate
            a_norm = float(np.linalg.norm(accel))
            if a_norm > max_accel:
                accel = accel * (max_accel / a_norm)
            actions[i, :2] = accel

            # Yaw P-controller toward velocity direction
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > cfg.velocity_align_threshold:
                target_h = math.atan2(v[1], v[0])
                err = (target_h - drone.heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                actions[i, 2] = float(
                    np.clip(cfg.yaw_align_gain * err,
                            -max_yaw_accel, max_yaw_accel)
                )

        return actions
