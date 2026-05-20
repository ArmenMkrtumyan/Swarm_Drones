"""
Ant Colony Optimization controller for swarm coverage.

Following Dorigo (1992) / Dorigo & Stützle (2004), adapted to runtime
control on a coverage objective:

    1. Each drone is an "ant" choosing a next-target cell probabilistically.
    2. A pheromone field τ[y, x] is deposited at *uncovered* free cells each
       step (Q per cell) and evaporated globally by `ρ` per step. Pheromone
       at a cell therefore tracks how long it has been an uncovered target —
       fresh uncovered cells have low τ; long-uncovered cells accumulate τ.
    3. Heuristic η[c] = 1 / max(distance(drone, c), 0.5) — closer is better.
    4. Probability of an ant picking cell c:
           P(c) ∝ τ(c)^α · η(c)^β
    5. The drone heads toward the chosen cell via the same attract + drone-
       repel + wall-repel + yaw-track-velocity stack used by PF/PSO/GA.

Stateful (the pheromone field persists across `__call__`s within an episode).
Instantiate fresh per `env.reset()`.

Hyperparameters worth tuning are the three ACO knobs (α, β, ρ); the movement
gains share PF's defaults.
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
class ACOConfig:
    """Tunable knobs for `ACOController`."""
    # Core ACO knobs — defaults from `tools/grid_search_aco.py` 27-config sweep.
    # The top-9 configs all share β=4 and tied at score +0.470; with β that
    # high, the heuristic (distance) term dominates and pheromone is washed
    # out. Picking the median of the tied top-9 here. β below ~2 lets
    # pheromone meaningfully affect choice but scored slightly worse overall.
    pheromone_weight: float = 0.9291   # α — how strongly τ biases the ant choice  (BO; was 1.0)
    heuristic_weight: float = 3.6153   # β — how strongly distance biases the ant choice  (BO; was 4.0)
    evaporation_rate: float = 0.0165   # ρ — fraction of pheromone removed each step  (BO; was 0.15)
    deposit_amount: float = 1.0       # Q — pheromone added at each uncov cell per step

    # Search horizon: ants only consider cells within this radius (cells).
    # Bounds the pickable target set; outside this, fall back to global.
    target_search_radius: float = 6.2982   # Target-commitment radius. Once an ant samples a target, it commits to  (BO; was 8.0)
    # it until reached (within this radius), covered by some drone, or
    # stuck-blacklisted. Without commitment, ACO's stochastic per-step
    # resampling makes the target teleport between adjacent cells each
    # frame — the drone has ~1 s of brake/reverse inertia but the target
    # changes every 0.1 s, so it perpetually tries to turn around and
    # never closes distance to anywhere. Commitment keeps the algorithm's
    # stochastic character (pheromone field still updates every step,
    # and the next sample is still drawn from `τ^α · η^β`) but the drone
    # actually gets to its chosen cell before the next draw.
    arrival_radius: float = 0.6

    # Movement gains (same defaults as PF)
    attract_gain: float = 3.6454   # BO-tuned (was 1.5)
    attract_damp_gain: float = 0.2895   # see PFConfig.attract_damp_gain  (BO; was 0.2)
    drone_repel_gain: float = 5.0
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


class ACOController:
    """ACO-flavored swarm controller. Pheromone-weighted target selection."""

    def __init__(
        self,
        cfg: Optional[ACOConfig] = None,
        hover_drone_idx: Optional[int] = 0,
        seed: Optional[int] = None,
    ) -> None:
        self.cfg = cfg or ACOConfig()
        self.hover_drone_idx = hover_drone_idx
        self._rng = np.random.default_rng(seed)
        self._pheromone: Optional[np.ndarray] = None  # (h, w)
        self._last_targets: Optional[list[Optional[np.ndarray]]] = None
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        self._blacklist: Optional[list[set[tuple[int, int]]]] = None
        self._blacklist_anchor_pos: Optional[list[np.ndarray]] = None

    def reset(self) -> None:
        self._pheromone = None
        self._last_targets = None
        self._stuck_best_dist = None
        self._stuck_anchor_t = None
        self._blacklist = None
        self._blacklist_anchor_pos = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        return {
            "heatmap": self._pheromone,
            "targets": self._last_targets,
            "title_extra": "ACO",
        }

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered

        # Initialize / update pheromone field.
        if self._pheromone is None:
            self._pheromone = np.zeros_like(env.grid, dtype=np.float64)
            self._last_targets = [None] * n
            self._stuck_best_dist = [float('inf')] * n
            self._stuck_anchor_t = [env.time_seconds] * n
            self._blacklist = [set() for _ in range(n)]
            self._blacklist_anchor_pos = [d.pos.copy() for d in env.drones]
        self._pheromone *= (1.0 - self.cfg.evaporation_rate)
        self._pheromone[uncov_mask] += self.cfg.deposit_amount

        if not uncov_mask.any():
            return actions

        # Precompute candidate positions for all uncovered cells.
        uy, ux = np.where(uncov_mask)
        uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])  # (M, 2)
        # Pheromone at each candidate cell.
        tau_all = self._pheromone[uy, ux]

        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        cfg = self.cfg

        targets_log = [None] * n
        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            # --- Stuck detection on previous step's target ---
            prev = self._last_targets[i]
            if prev is not None:
                dist_prev = float(np.linalg.norm(drone.pos - prev))
                if dist_prev < self._stuck_best_dist[i] - cfg.stuck_min_progress:
                    self._stuck_best_dist[i] = dist_prev
                    self._stuck_anchor_t[i] = env.time_seconds
                elif (env.time_seconds - self._stuck_anchor_t[i]
                      > cfg.stuck_timeout_s):
                    bx, by = int(prev[0]), int(prev[1])
                    self._blacklist[i].add((by, bx))
                    self._blacklist_anchor_pos[i] = drone.pos.copy()
                    self._stuck_best_dist[i] = float('inf')
                    self._stuck_anchor_t[i] = env.time_seconds

            # --- Position-based blacklist decay ---
            moved = float(np.linalg.norm(
                drone.pos - self._blacklist_anchor_pos[i]
            ))
            if moved > cfg.blacklist_decay_distance:
                self._blacklist[i].clear()
                self._blacklist_anchor_pos[i] = drone.pos.copy()

            # Target commitment: re-sample only when (a) no prior target,
            # (b) prior target reached, (c) prior target's cell got covered
            # (by us or any other drone), or (d) prior target was just
            # blacklisted by the stuck-detector above (its cell would now
            # be in self._blacklist[i]).
            need_resample = prev is None
            if not need_resample:
                px_i, py_i = int(prev[0]), int(prev[1])
                reached = float(np.linalg.norm(drone.pos - prev)) <= cfg.arrival_radius
                covered = (0 <= py_i < env.grid.shape[0]
                           and 0 <= px_i < env.grid.shape[1]
                           and bool(env.covered[py_i, px_i]))
                blacklisted_now = (py_i, px_i) in self._blacklist[i]
                need_resample = reached or covered or blacklisted_now

            if not need_resample:
                target = prev
            else:
                # Filter uncov_pos by this drone's blacklist (boolean mask).
                if self._blacklist[i]:
                    keep = np.ones(len(uncov_pos), dtype=bool)
                    for k in range(len(uncov_pos)):
                        if (int(uncov_pos[k, 1]), int(uncov_pos[k, 0])) in self._blacklist[i]:
                            keep[k] = False
                    if not keep.any():
                        # Every uncov cell blacklisted — clear and retry.
                        self._blacklist[i].clear()
                        keep = np.ones(len(uncov_pos), dtype=bool)
                    my_uncov_pos = uncov_pos[keep]
                    my_tau_all = tau_all[keep]
                else:
                    my_uncov_pos = uncov_pos
                    my_tau_all = tau_all

                d2 = ((my_uncov_pos - drone.pos) ** 2).sum(axis=1)
                in_range = d2 < cfg.target_search_radius ** 2
                if in_range.any():
                    cand_pos = my_uncov_pos[in_range]
                    cand_tau = my_tau_all[in_range]
                    cand_d = np.sqrt(d2[in_range])
                else:
                    # No uncov cells within search radius — fall back to all uncov
                    cand_pos = my_uncov_pos
                    cand_tau = my_tau_all
                    cand_d = np.sqrt(d2)

                # ACO probability:  P ∝ τ^α  ·  η^β   with η = 1 / max(d, 0.5)
                eta = 1.0 / np.maximum(cand_d, 0.5)
                scores = (np.maximum(cand_tau, 1e-9) ** cfg.pheromone_weight) \
                         * (eta ** cfg.heuristic_weight)
                total = scores.sum()
                if total <= 0 or not np.isfinite(total):
                    # Degenerate (all zero pheromone or numerical underflow):
                    # fall back to nearest uncov cell.
                    target = cand_pos[int(np.argmin(cand_d))]
                else:
                    probs = scores / total
                    idx = int(self._rng.choice(len(cand_pos), p=probs))
                    target = cand_pos[idx]
                # Reset progress tracker for new target.
                self._stuck_best_dist[i] = float('inf')
                self._stuck_anchor_t[i] = env.time_seconds
            targets_log[i] = target.copy()

            # ---- movement: head toward chosen target via PF-like force ----
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
                in_w = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_w.any():
                    weights = 1.0 / np.maximum(wd[in_w] ** 2, 0.05)
                    a += cfg.wall_repel_gain * (
                        d_wall[in_w].T * weights
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

        self._last_targets = targets_log
        return actions
