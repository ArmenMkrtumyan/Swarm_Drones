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
    pheromone_weight: float = 1.0     # α — how strongly τ biases the ant choice
    heuristic_weight: float = 4.0     # β — how strongly distance biases the ant choice
    evaporation_rate: float = 0.15    # ρ — fraction of pheromone removed each step
    deposit_amount: float = 1.0       # Q — pheromone added at each uncov cell per step

    # Search horizon: ants only consider cells within this radius (cells).
    # Bounds the pickable target set; outside this, fall back to global.
    target_search_radius: float = 8.0

    # Movement gains (same defaults as PF)
    attract_gain: float = 1.5
    drone_repel_gain: float = 5.0
    drone_repel_range: float = 2.5
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


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

    def reset(self) -> None:
        self._pheromone = None
        self._last_targets = None

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

            d2 = ((uncov_pos - drone.pos) ** 2).sum(axis=1)
            in_range = d2 < cfg.target_search_radius ** 2
            if in_range.any():
                cand_pos = uncov_pos[in_range]
                cand_tau = tau_all[in_range]
                cand_d = np.sqrt(d2[in_range])
            else:
                # No uncov cells within search radius — fall back to all uncov
                cand_pos = uncov_pos
                cand_tau = tau_all
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
            targets_log[i] = target.copy()

            # ---- movement: head toward chosen target via PF-like force ----
            offset_to_target = target - drone.pos
            dist = float(np.linalg.norm(offset_to_target))
            if dist > 1e-9:
                a = cfg.attract_gain * (offset_to_target / dist)
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
                    np.clip(cfg.yaw_align_gain * err,
                            -max_yaw_accel, max_yaw_accel)
                )

        self._last_targets = targets_log
        return actions
