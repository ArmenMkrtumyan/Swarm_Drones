"""
Particle Swarm Optimization controller for swarm coverage.

Treats each drone as a PSO particle directly: per drone's velocity update is
the canonical PSO formula

    v_new = w·v  +  c1·r1·(p_best − pos)  +  c2·r2·(g_best − pos)

with `r1, r2 ~ U(0, 1)`. Since `CoverageEnv` accepts acceleration commands
(not velocities), we use `v_new` as the acceleration command and let the env's
existing forward-Euler integration of velocity handle momentum. The hyperparam
defaults are tuned for this acceleration-output interpretation, not textbook
PSO position updates.

    p_best     = position where this drone last saw its highest local fitness.
    g_best     = position of the highest fitness across the swarm so far.
    fitness(p) = count of uncovered free cells within `fitness_radius` of p.

The fitness landscape is **dynamic**: as drones cover cells, fitness at those
locations drops, so g_best naturally migrates to fresh territory. This is why
the swarm-intelligence interpretation works for coverage — the optimization
target itself decays as the optimum is exploited.

Wall repulsion (1/r², same form as PF) is added as a safety net to prevent
particles from driving into corners.

Yaw tracks velocity (same convention as PF / Consensus), so the wedge sweeps
the path the drone is actually walking.

**Stateful** — `_pbest_pos`, `_pbest_fitness`, `_gbest_pos`, `_gbest_fitness`
persist across `__call__` invocations within an episode. Instantiate a fresh
controller for each `env.reset()`.
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
    # Inertia: scales the contribution of the drone's current velocity to the
    # acceleration command. With env's natural momentum already present,
    # values around 0–0.5 keep the drone responsive without overshoot.
    inertia: float = 0.5

    # Cognitive coefficient — pull toward this drone's personal-best position.
    # Encourages "go back to where you saw uncovered territory".
    cognitive: float = 1.0

    # Social coefficient — pull toward the swarm's global-best position.
    # Higher than cognitive in many coverage formulations because the swarm
    # benefits from flocking to whichever drone found the best fresh area.
    social: float = 2.0

    # Fitness radius (cells): how far around `pos` we count uncovered free
    # cells. Larger = smoother fitness landscape but more compute. Default
    # 4.0 cells (= 20 m at 5 m/cell) — slightly more than sensor_range.
    fitness_radius: float = 4.0

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
        # Per-episode PSO memory; `None` until first __call__ initializes it.
        self._pbest_pos: Optional[np.ndarray] = None
        self._pbest_fitness: Optional[np.ndarray] = None
        self._gbest_pos: Optional[np.ndarray] = None
        self._gbest_fitness: Optional[float] = None

    def reset(self) -> None:
        """Clear PSO memory. Call before re-using the controller on a new env."""
        self._pbest_pos = None
        self._pbest_fitness = None
        self._gbest_pos = None
        self._gbest_fitness = None

    def _local_fitness(
        self, pos: np.ndarray, uncov_pos: np.ndarray
    ) -> float:
        """Count of uncovered free cells within `fitness_radius` of `pos`."""
        if len(uncov_pos) == 0:
            return 0.0
        d2 = ((uncov_pos - pos) ** 2).sum(axis=1)
        return float((d2 < self.cfg.fitness_radius ** 2).sum())

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered
        if not uncov_mask.any():
            return actions

        uy, ux = np.where(uncov_mask)
        uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])

        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])    # (n, 2)
        velocities = np.array([d.vel for d in env.drones])   # (n, 2)
        fitnesses = np.array([
            self._local_fitness(positions[i], uncov_pos) for i in range(n)
        ])

        # Initialize PSO memory on first call.
        if self._pbest_pos is None:
            self._pbest_pos = positions.copy()
            self._pbest_fitness = fitnesses.copy()
            best_i = int(np.argmax(fitnesses))
            self._gbest_pos = positions[best_i].copy()
            self._gbest_fitness = float(fitnesses[best_i])
        else:
            improved = fitnesses > self._pbest_fitness
            if improved.any():
                self._pbest_pos[improved] = positions[improved]
                self._pbest_fitness[improved] = fitnesses[improved]
            best_i = int(np.argmax(fitnesses))
            if fitnesses[best_i] > self._gbest_fitness:
                self._gbest_pos = positions[best_i].copy()
                self._gbest_fitness = float(fitnesses[best_i])

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
                + cfg.cognitive * r1 * (self._pbest_pos[i] - pos)
                + cfg.social * r2 * (self._gbest_pos - pos)
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
