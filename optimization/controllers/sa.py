"""
Simulated Annealing controller for swarm coverage.

Following Kirkpatrick, Gelatt, Vecchi (1983), adapted to runtime control:
each drone holds a current target cell (its "candidate solution") and runs
the Metropolis acceptance rule per env step:

    1. Cool down: T ← max(T_min, T · cooling_rate)
    2. Propose: target' = target + N(0, perturb_radius), snapped to the
       nearest uncovered cell.
    3. ΔE = fitness(target) − fitness(target')   (we maximize fitness, so
       this is "−Δfitness")
    4. If ΔE ≤ 0: accept (better or equal).
       Else: accept with probability exp(−ΔE / T).
    5. Movement is the same PF-like attract + drone-repel + wall-repel +
       yaw-track-velocity stack used by GA / ACO / PSO.

`fitness(target) = count of uncovered free cells within fitness_radius of
target`. The temperature schedule starts permissive (often-accept worse
moves → exploration) and tightens toward greedy hill-climbing.

Stateful — `_targets` and `_T` persist across `__call__`s. Instantiate
fresh per `env.reset()`.
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
class SAConfig:
    """Tunable knobs for `SAController`."""
    # SA core — defaults from `tools/grid_search_sa.py` 27-config sweep.
    # All top-10 configs have σ=3 (small perturbations win) plus moderate T0
    # and slow cooling. SA was the weakest Track 2 algorithm overall — the
    # Metropolis "sometimes accept worse" rule produces wandering on a
    # dynamic coverage objective.
    T_initial: float = 3.3145   # starting temperature  (BO; was 0.5)
    cooling_rate: float = 0.9616   # geometric: T_{k+1} = T_k · cooling_rate  (BO; was 0.999)
    T_min: float = 1e-3             # floor (effectively-greedy below this)
    perturb_radius: float = 1.2299   # cells; Gaussian σ on proposed offsets  (BO; was 3.0)
    fitness_radius: float = 4.0     # cells; uncov-cell count radius for fitness

    # Movement gains (same defaults as PF)
    attract_gain: float = 3.3618   # BO-tuned (was 1.5)
    attract_damp_gain: float = 0.3477   # see PFConfig.attract_damp_gain  (BO; was 0.2)
    drone_repel_gain: float = 4.4124   # BO-tuned (was 5.0)
    drone_repel_range: float = 2.5
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    yaw_damp_gain: float = 4.9   # critical damping K_d = 2·√K_p; see PFConfig
    velocity_align_threshold: float = 0.05

    # Stuck-detector + per-drone blacklist. See STCConfig.stuck_*.
    stuck_timeout_s: float = 5.0
    stuck_min_progress: float = 0.3
    blacklist_decay_distance: float = 5.0

    # Target-commitment radius — see ACOConfig.arrival_radius. Without
    # commitment, the Metropolis perturbation runs every step and the
    # target random-walks under the drone's nose; with it, the perturb
    # only fires once the drone has arrived (or its target was covered
    # / blacklisted), preserving SA's annealing character but pacing it
    # with the physics.
    arrival_radius: float = 0.6


class SAController:
    """Per-drone Simulated Annealing over target cells."""

    def __init__(
        self,
        cfg: Optional[SAConfig] = None,
        hover_drone_idx: Optional[int] = 0,
        seed: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or SAConfig()
        self.hover_drone_idx = hover_drone_idx
        self._rng = np.random.default_rng(seed)
        self._targets: Optional[np.ndarray] = None     # (n, 2)
        self._T: Optional[float] = None                # scalar
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        self._blacklist: Optional[list[set[tuple[int, int]]]] = None
        self._blacklist_anchor_pos: Optional[list[np.ndarray]] = None

    def reset(self) -> None:
        self._targets = None
        self._T = None
        self._stuck_best_dist = None
        self._stuck_anchor_t = None
        self._blacklist = None
        self._blacklist_anchor_pos = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        return {
            "targets": self._targets,
            "title_extra": "Simulated Annealing",
        }

    def _fitness(self, target: np.ndarray, uncov_pos: np.ndarray) -> float:
        if len(uncov_pos) == 0:
            return 0.0
        d2 = ((uncov_pos - target) ** 2).sum(axis=1)
        return float((d2 < self.cfg.fitness_radius ** 2).sum())

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

        # Initialize targets / temperature on first call.
        if self._targets is None or len(self._targets) != n:
            idxs = self._rng.choice(
                len(uncov_pos), size=min(n, len(uncov_pos)), replace=False
            )
            self._targets = uncov_pos[idxs].copy()
            self._stuck_best_dist = [float('inf')] * n
            self._stuck_anchor_t = [env.time_seconds] * n
            self._blacklist = [set() for _ in range(n)]
            self._blacklist_anchor_pos = [d.pos.copy() for d in env.drones]
        if self._T is None:
            self._T = self.cfg.T_initial

        # Stuck detection + stale-target cleanup.
        for i in range(n):
            tx, ty = int(self._targets[i][0]), int(self._targets[i][1])
            stale = not (0 <= ty < env.h and 0 <= tx < env.w
                         and uncov_mask[ty, tx])

            # Distance-to-target progress check.
            dist_prev = float(np.linalg.norm(env.drones[i].pos - self._targets[i]))
            if dist_prev < self._stuck_best_dist[i] - self.cfg.stuck_min_progress:
                self._stuck_best_dist[i] = dist_prev
                self._stuck_anchor_t[i] = env.time_seconds
            elif (env.time_seconds - self._stuck_anchor_t[i]
                  > self.cfg.stuck_timeout_s):
                self._blacklist[i].add((ty, tx))
                self._blacklist_anchor_pos[i] = env.drones[i].pos.copy()
                stale = True
                self._stuck_best_dist[i] = float('inf')
                self._stuck_anchor_t[i] = env.time_seconds

            # Position-based blacklist decay.
            moved = float(np.linalg.norm(
                env.drones[i].pos - self._blacklist_anchor_pos[i]
            ))
            if moved > self.cfg.blacklist_decay_distance:
                self._blacklist[i].clear()
                self._blacklist_anchor_pos[i] = env.drones[i].pos.copy()

            if stale:
                pool = uncov_pos
                if self._blacklist[i]:
                    keep = np.ones(len(uncov_pos), dtype=bool)
                    for k in range(len(uncov_pos)):
                        if (int(uncov_pos[k, 1]), int(uncov_pos[k, 0])) in self._blacklist[i]:
                            keep[k] = False
                    if keep.any():
                        pool = uncov_pos[keep]
                    else:
                        self._blacklist[i].clear()
                d2 = ((pool - positions[i]) ** 2).sum(axis=1)
                new_target = pool[int(d2.argmin())].copy()
                if not np.allclose(new_target, self._targets[i], atol=0.1):
                    self._stuck_best_dist[i] = float('inf')
                    self._stuck_anchor_t[i] = env.time_seconds
                self._targets[i] = new_target

        # Cool down.
        self._T = max(self.cfg.T_min, self._T * self.cfg.cooling_rate)

        # Per-drone Metropolis update. Gated on target arrival so the
        # perturbation doesn't reassign targets while drones are in flight.
        cfg = self.cfg
        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue
            # Target-commitment gate.
            dist_to_target = float(np.linalg.norm(drone.pos - self._targets[i]))
            if dist_to_target > cfg.arrival_radius:
                continue

            current_fitness = self._fitness(self._targets[i], uncov_pos)
            offset = self._rng.normal(0.0, cfg.perturb_radius, size=2)
            candidate = self._targets[i] + offset
            d2 = ((uncov_pos - candidate) ** 2).sum(axis=1)
            new_target = uncov_pos[int(d2.argmin())].copy()
            new_fitness = self._fitness(new_target, uncov_pos)

            delta = current_fitness - new_fitness  # ΔE = −Δfitness
            if delta <= 0:
                self._targets[i] = new_target
            else:
                if self._rng.uniform() < math.exp(-delta / max(self._T, 1e-9)):
                    self._targets[i] = new_target

        # Movement: PF-like attract toward target.
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            target = self._targets[i]
            offset_to_target = target - drone.pos
            dist = float(np.linalg.norm(offset_to_target))
            if dist > 1e-9:
                a = cfg.attract_gain * (offset_to_target / dist) - cfg.attract_damp_gain * drone.vel
            else:
                a = np.zeros(2)

            # Drone repel
            for j in range(n):
                if j == i:
                    continue
                off = drone.pos - positions[j]
                d = float(np.linalg.norm(off))
                if 1e-9 < d < cfg.drone_repel_range:
                    a += cfg.drone_repel_gain * off / max(d ** 2, 0.05)

            # Wall repel
            if has_walls:
                d_wall = drone.pos - wall_pos
                wd = np.linalg.norm(d_wall, axis=1)
                in_w = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_w.any():
                    weights = 1.0 / np.maximum(wd[in_w] ** 2, 0.05)
                    a += cfg.wall_repel_gain * (
                        d_wall[in_w].T * weights
                    ).sum(axis=1)

            a_norm = float(np.linalg.norm(a))
            if a_norm > max_accel:
                a *= max_accel / a_norm
            actions[i, :2] = a

            # Yaw P-controller
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > cfg.velocity_align_threshold:
                target_h = math.atan2(v[1], v[0])
                err = (target_h - drone.heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                actions[i, 2] = float(
                    np.clip(cfg.yaw_align_gain * err - cfg.yaw_damp_gain * drone.yaw_rate,
                            -max_yaw_accel, max_yaw_accel)
                )

        return actions
