"""
Spanning Tree Coverage controller (~Gabriely & Rimon, 2001).

Each drone is assigned a Voronoi-partition region of the free cells (same
as `VoronoiPartitionController`), then follows a **precomputed traversal
order** through every cell in its region. The order comes from a BFS
spanning tree rooted at the drone's start — cells are visited in
breadth-first order, which naturally produces a "spiral-out from start"
walk on uniform terrain and an obstacle-aware shortest-path-from-start
walk in mazes.

Difference from `VoronoiPartitionController`:
    - Voronoi: at each step, head to the **nearest currently-uncovered**
      cell in the partition (greedy local).
    - STC: at each step, head to the **next cell in the precomputed
      traversal order** that is still uncovered. This avoids the
      backtracking that pure-greedy can cause when the nearest uncovered
      cell is far from the drone's local region.

Stateful — instantiate fresh per `env.reset()`.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from environment import CoverageEnv
from maze import FREE, WALL


@dataclass
class STCConfig:
    """Tunable knobs for `STCController`."""
    # Movement gains — defaults from `tools/grid_search_stc.py` 27-config
    # sweep (best: +0.433 score / 84.3 % mean cov). Top-9 of 10 had
    # attract=4.0 — STC's planned walk benefits from a strong pull toward
    # the next cell so the drone commits to following the plan instead of
    # being deflected by repulsions.
    attract_gain: float = 4.0
    drone_repel_gain: float = 10.0
    drone_repel_range: float = 4.0
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


class STCController:
    """Spanning-Tree-Coverage controller (BFS-ordered walk per partition)."""

    def __init__(
        self,
        cfg: Optional[STCConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or STCConfig()
        self.hover_drone_idx = hover_drone_idx
        # Per-drone walk: list of (y, x) tuples in BFS order from drone start.
        self._walks: Optional[list[list[tuple[int, int]]]] = None
        self._walk_idx: Optional[list[int]] = None

    def reset(self) -> None:
        self._walks = None
        self._walk_idx = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        n = env.n_drones
        h, w = env.grid.shape
        targets = [None] * n
        paths = [None] * n
        regions = None
        if self._walks is not None:
            regions = -np.ones((h, w), dtype=np.int32)
            for i in range(min(n, len(self._walks))):
                walk = self._walks[i]
                if not walk:
                    continue
                # Region overlay = every cell in this drone's walk.
                for (yy, xx) in walk:
                    if 0 <= yy < h and 0 <= xx < w:
                        regions[yy, xx] = i
                # Path from current walk index forward (planned upcoming walk).
                k = self._walk_idx[i] if self._walk_idx else 0
                remaining = walk[k:]
                if remaining:
                    paths[i] = np.array(
                        [(c[1] + 0.5, c[0] + 0.5) for c in remaining],
                        dtype=float,
                    )
                    yy, xx = remaining[0]
                    targets[i] = np.array([xx + 0.5, yy + 0.5], dtype=float)
        return {
            "regions": regions,
            "paths": paths,
            "targets": targets,
            "title_extra": "STC",
        }

    def _bfs_order(
        self, env: CoverageEnv, start_yx: tuple[int, int],
        owned_mask: np.ndarray,
    ) -> list[tuple[int, int]]:
        """Return cells in `owned_mask` ordered by BFS from `start_yx`."""
        h, w = env.grid.shape
        visited: set[tuple[int, int]] = {start_yx}
        order: list[tuple[int, int]] = [start_yx]
        queue = deque([start_yx])
        while queue:
            y, x = queue.popleft()
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and (ny, nx) not in visited \
                        and owned_mask[ny, nx]:
                    visited.add((ny, nx))
                    order.append((ny, nx))
                    queue.append((ny, nx))
        return order

    def _compute_plans(self, env: CoverageEnv) -> list[list[tuple[int, int]]]:
        """Voronoi-assign free cells, then BFS-order each drone's region."""
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        positions = np.array([d.pos for d in env.drones])
        n = env.n_drones

        ys = np.arange(h)[:, None] + 0.5
        xs = np.arange(w)[None, :] + 0.5
        d2 = ((positions[:, 0:1, None] - xs[None]) ** 2
              + (positions[:, 1:2, None] - ys[None]) ** 2)
        owner = d2.argmin(axis=0).astype(np.int32)
        owner[~free_mask] = -1

        walks = []
        for i in range(n):
            owned = (owner == i) & free_mask
            sx, sy = int(positions[i][0]), int(positions[i][1])
            # Snap start to nearest owned cell if (sy, sx) isn't owned.
            if not (0 <= sy < h and 0 <= sx < w and owned[sy, sx]):
                ys_o, xs_o = np.where(owned)
                if len(ys_o) == 0:
                    walks.append([])
                    continue
                d2_o = (xs_o + 0.5 - positions[i][0]) ** 2 \
                       + (ys_o + 0.5 - positions[i][1]) ** 2
                k = int(d2_o.argmin())
                sy, sx = int(ys_o[k]), int(xs_o[k])
            walks.append(self._bfs_order(env, (sy, sx), owned))
        return walks

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        if self._walks is None or len(self._walks) != n:
            self._walks = self._compute_plans(env)
            self._walk_idx = [0] * n

        free_mask = env.grid == FREE
        wy, wx = np.where(env.grid == WALL)
        has_walls = len(wy) > 0
        wall_pos = (np.column_stack([wx + 0.5, wy + 0.5])
                    if has_walls else None)

        positions = np.array([d.pos for d in env.drones])
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        cfg = self.cfg

        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            walk = self._walks[i]
            # Skip cells in the walk that are already covered.
            while self._walk_idx[i] < len(walk):
                y, x = walk[self._walk_idx[i]]
                if env.covered[y, x]:
                    self._walk_idx[i] += 1
                else:
                    break

            if self._walk_idx[i] >= len(walk):
                # Walk done — fall back to nearest uncov globally.
                uncov_mask = free_mask & ~env.covered
                if not uncov_mask.any():
                    continue
                uy, ux = np.where(uncov_mask)
                cand_pos = np.column_stack([ux + 0.5, uy + 0.5])
                d2 = ((cand_pos - drone.pos) ** 2).sum(axis=1)
                target = cand_pos[int(d2.argmin())]
            else:
                y, x = walk[self._walk_idx[i]]
                target = np.array([x + 0.5, y + 0.5], dtype=np.float64)

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

        return actions
