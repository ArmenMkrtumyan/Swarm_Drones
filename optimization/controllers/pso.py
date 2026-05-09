"""
Particle Swarm Optimization controller for swarm coverage.

Adaptation of canonical PSO (Kennedy & Eberhart, 1995) to a *dynamic*
coverage objective: as drones cover cells, the fitness landscape
shifts, so a textbook PSO with stale `p_best`/`g_best` quickly stalls
(both bests record positions whose neighborhoods are now covered). To
keep the swarm responsive, we re-elect both bests **every step** from
the current state of the world:

    p_best_i  = the uncovered free cell nearest to drone i
    g_best    = the uncovered free cell nearest to the swarm's centroid
    fitness   = `−distance(pos, candidate_cell)` (closer = better)

Each drone's acceleration command is the canonical PSO velocity update
applied as acceleration:

    a = w·v  +  c1·r1·(p_best_i − pos)  +  c2·r2·(g_best − pos)

with `r1, r2 ~ U(0, 1)` per dimension. The cognitive term pulls each
drone toward *its own* nearest uncovered cell (greedy local coverage);
the social term pulls every drone toward the swarm's collective
nearest uncovered cell (group cohesion). Together they trade off local
greedy progress vs. swarm-wide flocking.

Wall repulsion (1/r², same form as PF) prevents particles from
driving into corners.

Yaw tracks velocity (same convention as PF / Consensus), so the wedge
sweeps the path the drone is actually walking.

**Stateless** between `__call__`s — `p_best`/`g_best` are recomputed
every step from current `env.covered`. No reset() needed across episodes.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Make project root importable when controllers are loaded from anywhere.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from environment import CoverageEnv
from maze import FREE, WALL


@dataclass
class PSOConfig:
    """
    Tunable knobs for `PSOController`.

    Hyperparameter ranges below are starting points for grid search — they
    map the PSO update onto the env's acceleration units, so they are not
    directly comparable to canonical PSO literature values.
    """
    # Inertia: weight on current velocity in the PSO update used as
    # acceleration. With env's natural momentum already present, low
    # values keep the drone responsive without overshoot.
    inertia: float = 0.5

    # Cognitive coefficient — pull toward this drone's nearest uncovered
    # cell (re-elected each step). Drives local greedy coverage.
    cognitive: float = 3.0

    # Social coefficient — pull toward the swarm's nearest uncovered cell
    # (relative to the swarm centroid). Defaults below come from a
    # two-stage grid search (`tools/grid_search_pso.py`, broad then
    # corner-pushed): the textbook PSO recipe with non-zero social pull
    # *hurts* coverage because it makes drones flock, so the empirically
    # best setting is `social = 0.0` — no flocking term at all. The
    # cognitive pull is what does all the work on this dynamic objective.
    social: float = 0.0

    # Wall repulsion (1/r²) — same form and defaults as PF.
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5

    # Yaw P-controller (same as PF).
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


class PSOController:
    """
    PSO controller for swarm coverage.

    Usage:
        pso = PSOController()
        actions = pso(env)
        env.step(actions)

    Drone 0 is held at zero acceleration by default so the demo's hover
    sanity check still works. Set `hover_drone_idx=None` for full evaluation.
    """

    def __init__(
        self,
        cfg: Optional[PSOConfig] = None,
        hover_drone_idx: Optional[int] = 0,
        seed: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or PSOConfig()
        self.hover_drone_idx = hover_drone_idx
        self._rng = np.random.default_rng(seed)
        self._last_pbest = None
        self._last_gbest = None

    def reset(self) -> None:
        self._last_pbest = None
        self._last_gbest = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        return {
            "targets": self._last_pbest,
            "title_extra": "PSO",
        }

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered
        if not uncov_mask.any():
            return actions

        uy, ux = np.where(uncov_mask)
        uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])    # (M, 2)

        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])    # (n, 2)
        velocities = np.array([d.vel for d in env.drones])   # (n, 2)

        # Re-elect p_best and g_best each step from the current world state.
        # p_best_i = nearest uncovered cell to drone i.
        # g_best   = nearest uncovered cell to the swarm centroid.
        d_pi = uncov_pos[None, :, :] - positions[:, None, :]   # (n, M, 2)
        dist2_pi = (d_pi ** 2).sum(axis=2)                     # (n, M)
        nearest_per_drone = dist2_pi.argmin(axis=1)            # (n,)
        pbest_pos = uncov_pos[nearest_per_drone]               # (n, 2)

        swarm_centroid = positions.mean(axis=0)
        d_centroid = uncov_pos - swarm_centroid
        gbest_pos = uncov_pos[int((d_centroid ** 2).sum(axis=1).argmin())]
        self._last_pbest = pbest_pos.copy()
        self._last_gbest = gbest_pos.copy()

        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        cfg = self.cfg

        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            r1 = self._rng.uniform(0.0, 1.0, size=2)
            r2 = self._rng.uniform(0.0, 1.0, size=2)
            v = velocities[i]
            pos = positions[i]

            # PSO velocity update used as acceleration command.
            a = (
                cfg.inertia * v
                + cfg.cognitive * r1 * (pbest_pos[i] - pos)
                + cfg.social * r2 * (gbest_pos - pos)
            )

            # Wall repulsion (same form as PF).
            if has_walls:
                d_wall = pos - wall_pos
                wd = np.linalg.norm(d_wall, axis=1)
                in_range = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_range.any():
                    d_in = d_wall[in_range]
                    dists_in = wd[in_range]
                    weights = 1.0 / np.maximum(dists_in ** 2, 0.05)
                    a += cfg.wall_repel_gain * (d_in.T * weights).sum(axis=1)

            # Saturate to max_accel.
            a_norm = float(np.linalg.norm(a))
            if a_norm > max_accel:
                a *= max_accel / a_norm
            actions[i, :2] = a

            # Yaw P-controller toward velocity direction.
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
