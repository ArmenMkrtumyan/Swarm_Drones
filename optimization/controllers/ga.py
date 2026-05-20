"""
Genetic Algorithm controller for swarm coverage.

Following Holland (1975) / Goldberg (1989), but adapted to runtime control:
the **swarm is the population**, each drone holds one target cell (its
"chromosome"), and **each env step is one GA generation**.

Per step:
    1. Evaluate fitness of each drone's current target — count of uncovered
       free cells within `fitness_radius` of the target.
    2. Selection — sort drones by fitness; the top `elite_fraction · n` drones
       are "elite" and keep their targets unchanged.
    3. Crossover — each non-elite drone, with probability `p_crossover`, adopts
       a random elite drone's target with a small Gaussian perturbation, then
       snaps to the nearest uncovered cell.
    4. Mutation — independently with probability `p_mutation`, a non-elite
       drone jumps to a random uncovered cell within `mutation_jump_radius`.
    5. Stale-target cleanup — if a drone's target has since been covered, it
       falls back to the nearest uncovered cell (the same "default greedy" we
       use in PSO and PF as a baseline behavior).
    6. Movement — each drone heads toward its current target via the same
       attract + drone-repel + wall-repel + yaw-track-velocity stack as PF.

The hyperparameters worth tuning are the three GA operators
(`elite_fraction`, `p_crossover`, `p_mutation`); the movement gains share PF's
defaults. **Stateful** — `_targets` persists across `__call__`s; instantiate
fresh per episode.
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
class GAConfig:
    """Tunable knobs for `GAController`."""
    # GA operators — defaults from `tools/grid_search_ga.py` 27-config sweep
    # over the full (map × n_drones × seed) grid. Top-10 all had `p_crossover
    # ≤ 0.10`; the canonical "share information across the population" recipe
    # hurts coverage because it makes drones flock to the same elite targets.
    elite_fraction: float = 0.3994   # top fraction kept untouched  (BO; was 0.40)
    p_crossover: float = 0.1514   # prob non-elite adopts elite-derived target  (BO; was 0.10)
    p_mutation: float = 0.351   # prob non-elite jumps to random nearby uncov cell  (BO; was 0.30)

    # Fitness landscape
    fitness_radius: float = 4.0           # cells; uncov cells within this count toward fitness

    # Crossover perturbation: child target = parent target + N(0, σ)
    crossover_perturb_radius: float = 3.0

    # Mutation jump radius: random uncov cell within this distance of drone pos
    mutation_jump_radius: float = 8.0

    # Movement gains (same defaults as PF)
    attract_gain: float = 3.1629   # BO-tuned (was 1.5)
    attract_damp_gain: float = 0.999   # see PFConfig.attract_damp_gain  (BO; was 0.2)
    drone_repel_gain: float = 7.4842   # BO-tuned (was 5.0)
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

    # Target-commitment radius — see ACOConfig.arrival_radius. Without it,
    # the 10% mutation + 10% crossover per-step rates mean ~1 in 5 drones
    # gets a new random target every frame; the drone never has long
    # enough to actually reach a target before mutation reassigns it.
    # With commitment, mutation/crossover still run but apply only to
    # drones that have arrived (or whose target was covered/blacklisted)
    # — so a drone commits to one target, reaches it, then the next
    # generation picks where to go next.
    arrival_radius: float = 0.6


class GAController:
    """
    GA-flavored swarm controller.

    Usage:
        ga = GAController()
        actions = ga(env)
        env.step(actions)

    `hover_drone_idx=0` (default) keeps drone 0 stationary so the demo's hover
    sanity check still works.
    """

    def __init__(
        self,
        cfg: Optional[GAConfig] = None,
        hover_drone_idx: Optional[int] = 0,
        seed: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or GAConfig()
        self.hover_drone_idx = hover_drone_idx
        self._rng = np.random.default_rng(seed)
        self._targets: Optional[np.ndarray] = None  # (n, 2)
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        self._blacklist: Optional[list[set[tuple[int, int]]]] = None
        self._blacklist_anchor_pos: Optional[list[np.ndarray]] = None

    def reset(self) -> None:
        """Clear target memory. Call before re-using the controller on a new env."""
        self._targets = None
        self._last_elite_idx: set = set()
        self._stuck_best_dist = None
        self._stuck_anchor_t = None
        self._blacklist = None
        self._blacklist_anchor_pos = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        roles = [None] * env.n_drones
        for i in getattr(self, "_last_elite_idx", set()):
            if 0 <= i < env.n_drones:
                roles[i] = "elite"
        return {
            "targets": self._targets,
            "roles": roles,
            "title_extra": "Genetic Algorithm",
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
        uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])    # (M, 2)

        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])    # (n, 2)

        # ---- initialize targets if first call (random uncov cells) ----
        if self._targets is None or len(self._targets) != n:
            idxs = self._rng.choice(len(uncov_pos),
                                    size=min(n, len(uncov_pos)),
                                    replace=False)
            self._targets = uncov_pos[idxs].copy()
            self._stuck_best_dist = [float('inf')] * n
            self._stuck_anchor_t = [env.time_seconds] * n
            self._blacklist = [set() for _ in range(n)]
            self._blacklist_anchor_pos = [d.pos.copy() for d in env.drones]

        # ---- stuck detection + stale-target cleanup ----
        for i in range(n):
            tx, ty = int(self._targets[i][0]), int(self._targets[i][1])
            stale = not (0 <= ty < env.h and 0 <= tx < env.w
                         and uncov_mask[ty, tx])

            # Stuck detection: is the drone closing distance to its target?
            dist_prev = float(np.linalg.norm(env.drones[i].pos - self._targets[i]))
            if dist_prev < self._stuck_best_dist[i] - self.cfg.stuck_min_progress:
                self._stuck_best_dist[i] = dist_prev
                self._stuck_anchor_t[i] = env.time_seconds
            elif (env.time_seconds - self._stuck_anchor_t[i]
                  > self.cfg.stuck_timeout_s):
                # Stuck — blacklist this cell and force re-pick.
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
                # Pick nearest uncov excluding blacklist.
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
                # Reset tracker for new target.
                if not np.allclose(new_target, self._targets[i], atol=0.1):
                    self._stuck_best_dist[i] = float('inf')
                    self._stuck_anchor_t[i] = env.time_seconds
                self._targets[i] = new_target

        # ---- GA generation: evaluate fitness, select, crossover, mutate ----
        fitness = np.array([self._fitness(self._targets[i], uncov_pos)
                            for i in range(n)])
        n_elite = max(1, int(self.cfg.elite_fraction * n))
        elite_idx = set(np.argsort(-fitness)[:n_elite].tolist())
        elite_targets = self._targets[list(elite_idx)]
        self._last_elite_idx = elite_idx

        for i in range(n):
            if i in elite_idx:
                continue
            # Target-commitment gate: only mutate/crossover for drones that
            # have arrived at the current target (or whose target was just
            # replaced by stale-cleanup above, in which case dist≈0). Skipping
            # mutation/crossover for in-flight drones keeps them committed to
            # the cell they're heading toward — without this, the 10%+10%
            # per-step swap rate reassigns ~1 in 5 drones every frame and the
            # drone never has time to actually reach any cell.
            dist_to_target = float(np.linalg.norm(
                env.drones[i].pos - self._targets[i]
            ))
            if dist_to_target > self.cfg.arrival_radius:
                continue
            # Mutation has priority: random uncov cell within mutation radius
            if self._rng.uniform() < self.cfg.p_mutation:
                d2 = ((uncov_pos - positions[i]) ** 2).sum(axis=1)
                in_range = d2 < self.cfg.mutation_jump_radius ** 2
                cands = uncov_pos[in_range] if in_range.any() else uncov_pos
                self._targets[i] = cands[
                    int(self._rng.integers(len(cands)))
                ].copy()
                continue
            # Otherwise, crossover: adopt a perturbed elite target
            if self._rng.uniform() < self.cfg.p_crossover:
                parent = elite_targets[
                    int(self._rng.integers(len(elite_targets)))
                ]
                offset = self._rng.normal(
                    0.0, self.cfg.crossover_perturb_radius / 2.0, size=2
                )
                child = parent + offset
                d2 = ((uncov_pos - child) ** 2).sum(axis=1)
                self._targets[i] = uncov_pos[int(d2.argmin())].copy()
            # else: keep current target

        # ---- movement: head toward each drone's target via PF-like force ----
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        cfg = self.cfg

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

            # Drone repulsion (1/r²)
            for j in range(n):
                if j == i:
                    continue
                off = drone.pos - positions[j]
                d = float(np.linalg.norm(off))
                if 1e-9 < d < cfg.drone_repel_range:
                    a += cfg.drone_repel_gain * off / max(d ** 2, 0.05)

            # Wall repulsion
            if has_walls:
                d_wall = drone.pos - wall_pos
                wd = np.linalg.norm(d_wall, axis=1)
                in_range = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_range.any():
                    weights = 1.0 / np.maximum(wd[in_range] ** 2, 0.05)
                    a += cfg.wall_repel_gain * (
                        d_wall[in_range].T * weights
                    ).sum(axis=1)

            # Saturate
            a_norm = float(np.linalg.norm(a))
            if a_norm > max_accel:
                a *= max_accel / a_norm
            actions[i, :2] = a

            # Yaw P-controller toward velocity direction
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
