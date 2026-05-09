"""
Static Voronoi-partition coverage controller.

Compute a Voronoi partition of the **free cells** by Euclidean distance
to each drone's *initial* position once at the first `__call__`, then
freeze it. Each drone's policy is "head to the nearest uncovered cell
*in your assigned partition*". When a drone has covered every free cell
in its partition, fall back to the nearest uncovered cell globally so it
keeps contributing.

This is the **static** counterpart to `ConsensusController`, which
re-elects the partition every step using current positions and a
`comm_range` neighbour set. Static partitioning is the textbook
"task allocation" approach — assign work upfront, drones execute
locally without renegotiation.

Movement: same attract + drone-repel + wall-repel + yaw-tracks-velocity
stack as PF.

Stateful — instantiate fresh per `env.reset()` (the partition is built
on first call and frozen).
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
class VoronoiPartitionConfig:
    """Tunable knobs for `VoronoiPartitionController`."""
    fallback_to_global: bool = True   # pursue global nearest uncov when partition done

    # Movement gains — defaults from `tools/grid_search_voronoi_partition.py`.
    # Top-9 of 10 had attract=1.0; the clean static partition removes the need
    # for aggressive attraction. Best config: +0.449 score / 91.2 % mean cov,
    # essentially matching Track 2's ACO (+0.470) and Track 3's Consensus.
    attract_gain: float = 1.0
    drone_repel_gain: float = 2.0
    drone_repel_range: float = 4.0
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


class VoronoiPartitionController:
    """Static Voronoi partition + nearest-uncovered-in-region."""

    def __init__(
        self,
        cfg: Optional[VoronoiPartitionConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or VoronoiPartitionConfig()
        self.hover_drone_idx = hover_drone_idx
        # `_partition[y, x]` = drone index that owns cell (y, x), -1 for walls.
        self._partition: Optional[np.ndarray] = None
        self._last_targets: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._partition = None
        self._last_targets = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        return {
            "regions": self._partition,
            "targets": self._last_targets,
            "title_extra": "Voronoi Partition",
        }

    def _compute_partition(self, env: CoverageEnv) -> np.ndarray:
        """Voronoi assignment of every free cell to nearest drone start."""
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        positions = np.array([d.pos for d in env.drones])    # (n, 2)

        ys = np.arange(h)[:, None] + 0.5
        xs = np.arange(w)[None, :] + 0.5
        # Distance² from every cell to every drone — shape (n, h, w).
        d2 = ((positions[:, 0:1, None] - xs[None]) ** 2
              + (positions[:, 1:2, None] - ys[None]) ** 2)
        owner = d2.argmin(axis=0).astype(np.int32)
        owner[~free_mask] = -1
        return owner

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        if self._partition is None:
            self._partition = self._compute_partition(env)

        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered
        if not uncov_mask.any():
            return actions

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

            # Find this drone's partition's uncovered cells.
            mine = (self._partition == i) & uncov_mask
            if mine.any():
                my_y, my_x = np.where(mine)
                my_pos = np.column_stack([my_x + 0.5, my_y + 0.5])
                d2 = ((my_pos - drone.pos) ** 2).sum(axis=1)
                target = my_pos[int(d2.argmin())]
            elif cfg.fallback_to_global:
                uy, ux = np.where(uncov_mask)
                uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])
                d2 = ((uncov_pos - drone.pos) ** 2).sum(axis=1)
                target = uncov_pos[int(d2.argmin())]
            else:
                continue
            targets_log[i] = target.copy()

            # ---- attract toward target ----
            offset = target - drone.pos
            dist = float(np.linalg.norm(offset))
            if dist > 1e-9:
                a = cfg.attract_gain * (offset / dist)
            else:
                a = np.zeros(2)

            # ---- drone repel ----
            for j in range(n):
                if j == i:
                    continue
                off = drone.pos - positions[j]
                d = float(np.linalg.norm(off))
                if 1e-9 < d < cfg.drone_repel_range:
                    a += cfg.drone_repel_gain * off / max(d ** 2, 0.05)

            # ---- wall repel ----
            if has_walls:
                d_wall = drone.pos - wall_pos
                wd = np.linalg.norm(d_wall, axis=1)
                in_w = (wd > 1e-9) & (wd < cfg.wall_repel_range)
                if in_w.any():
                    weights = 1.0 / np.maximum(wd[in_w] ** 2, 0.05)
                    a += cfg.wall_repel_gain * (d_wall[in_w].T * weights).sum(axis=1)

            # ---- saturate ----
            a_norm = float(np.linalg.norm(a))
            if a_norm > max_accel:
                a *= max_accel / a_norm
            actions[i, :2] = a

            # ---- yaw track velocity ----
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
