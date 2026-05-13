"""
Grid (cellular) decomposition coverage controller.

Classical task-allocation approach:
    1. Divide the map into a regular grid of `block_size × block_size` blocks.
    2. Each block's representative is the centroid of its free cells.
    3. Assign each block to the drone closest to its centroid (Voronoi-style).
    4. Each drone has an ordered list of its blocks (nearest-from-start first).
    5. Per step, the drone heads to the **nearest uncovered free cell within
       its current block**; when no free cells remain in the block, advance
       to the next block in its list.

Movement: same attract + drone-repel + wall-repel + yaw-tracks-velocity
stack as PF.

Stateful — instantiate fresh per `env.reset()` (the block list is built
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
class GridDecompositionConfig:
    """Tunable knobs for `GridDecompositionController`."""
    block_size: int = 5   # cells per side of each rectangular block  (BO; was 8)

    # Movement gains — defaults from `tools/grid_search_grid_decomposition.py`
    # 27-config sweep (best: +0.382 score / 85.2 % mean cov). block_size=8
    # dominated the top 10; smaller blocks fragment the work too much for
    # small swarms.
    attract_gain: float = 3.0263   # BO-tuned (was 1.0)
    attract_damp_gain: float = 0.0851   # see PFConfig.attract_damp_gain (lower  (BO; was 0.1)
                                     # than the 0.2 default — 0.2 stalls this
                                     # controller's per-block targeting; 0.1
                                     # found by sweep, gives 285s vs 290s base)
    drone_repel_gain: float = 3.7054   # BO-tuned (was 5.0)
    drone_repel_range: float = 2.8548   # BO-tuned (was 2.5)
    wall_repel_gain: float = 2.2397   # BO-tuned (was 2.0)
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    yaw_damp_gain: float = 4.9   # critical damping K_d = 2·√K_p; see PFConfig
    velocity_align_threshold: float = 0.05

    # Stuck-detector + per-drone blacklist. See STCConfig.stuck_* for full
    # rationale.
    stuck_timeout_s: float = 5.0
    stuck_min_progress: float = 0.3
    blacklist_decay_distance: float = 5.0


class GridDecompositionController:
    """Block-partitioned coverage controller."""

    def __init__(
        self,
        cfg: Optional[GridDecompositionConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or GridDecompositionConfig()
        self.hover_drone_idx = hover_drone_idx
        # `_blocks_per_drone[i]` = list of (y0, y1, x0, x1) tuples ordered by
        # distance from drone i's initial position (nearest first).
        self._blocks_per_drone: Optional[list[list[tuple]]] = None
        self._block_idx: Optional[list[int]] = None
        self._last_targets: Optional[list[Optional[np.ndarray]]] = None
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        self._blacklist: Optional[list[set[tuple[int, int]]]] = None
        self._blacklist_anchor_pos: Optional[list[np.ndarray]] = None

    def reset(self) -> None:
        self._blocks_per_drone = None
        self._block_idx = None
        self._last_targets = None
        self._stuck_best_dist = None
        self._stuck_anchor_t = None
        self._blacklist = None
        self._blacklist_anchor_pos = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        n = env.n_drones
        h, w = env.grid.shape
        regions = None
        targets = getattr(self, "_last_targets", None)
        if self._blocks_per_drone is not None:
            regions = -np.ones((h, w), dtype=np.int32)
            for i in range(min(n, len(self._blocks_per_drone))):
                for (y0, y1, x0, x1) in self._blocks_per_drone[i]:
                    regions[y0:y1, x0:x1] = i
            # Mark walls as unowned for clarity.
            regions[env.grid == WALL] = -1
        return {
            "regions": regions,
            "targets": targets,
            "title_extra": "Grid Decomposition",
        }

    def _compute_plan(self, env: CoverageEnv) -> list[list[tuple]]:
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        bs = self.cfg.block_size
        positions = np.array([d.pos for d in env.drones])
        n = env.n_drones

        # Enumerate blocks (y0, y1, x0, x1) and their free-centroid (or center).
        blocks = []
        centroids = []
        for y0 in range(0, h, bs):
            for x0 in range(0, w, bs):
                y1 = min(y0 + bs, h)
                x1 = min(x0 + bs, w)
                free_in_block = free_mask[y0:y1, x0:x1]
                if not free_in_block.any():
                    continue   # all-wall block; skip
                # Centroid in the block's local coords, then offset to global.
                ys, xs = np.where(free_in_block)
                cy = float(ys.mean()) + y0 + 0.5
                cx = float(xs.mean()) + x0 + 0.5
                blocks.append((y0, y1, x0, x1))
                centroids.append(np.array([cx, cy]))

        if not blocks:
            return [[] for _ in range(n)]

        centroids_arr = np.array(centroids)
        # Voronoi-style assignment: each block → nearest drone start.
        d2 = ((positions[:, None, :] - centroids_arr[None, :, :]) ** 2).sum(axis=2)
        owner = d2.argmin(axis=0)

        # Per-drone ordered block list (nearest-first from drone start).
        per_drone: list[list[tuple]] = [[] for _ in range(n)]
        for b_idx, b_owner in enumerate(owner):
            per_drone[int(b_owner)].append((b_idx, blocks[b_idx]))
        for i in range(n):
            per_drone[i].sort(
                key=lambda b: float(
                    np.linalg.norm(centroids_arr[b[0]] - positions[i])
                )
            )
        # Strip the b_idx now that ordering is done.
        return [[b[1] for b in lst] for lst in per_drone]

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        if self._blocks_per_drone is None or len(self._blocks_per_drone) != n:
            self._blocks_per_drone = self._compute_plan(env)
            self._block_idx = [0] * n
            self._last_targets = [None] * n
            self._stuck_best_dist = [float('inf')] * n
            self._stuck_anchor_t = [env.time_seconds] * n
            self._blacklist = [set() for _ in range(n)]
            self._blacklist_anchor_pos = [d.pos.copy() for d in env.drones]

        h, w = env.grid.shape
        free_mask = env.grid == FREE
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

            # Find the next block with uncovered free cells (excluding blacklist).
            blocks = self._blocks_per_drone[i]
            target = None
            while self._block_idx[i] < len(blocks):
                y0, y1, x0, x1 = blocks[self._block_idx[i]]
                block_uncov = free_mask[y0:y1, x0:x1] & ~env.covered[y0:y1, x0:x1]
                if self._blacklist[i]:
                    block_uncov = block_uncov.copy()
                    for (by, bx) in self._blacklist[i]:
                        if y0 <= by < y1 and x0 <= bx < x1:
                            block_uncov[by - y0, bx - x0] = False
                if not block_uncov.any():
                    self._block_idx[i] += 1
                    continue
                ys, xs = np.where(block_uncov)
                # Convert to global cell-center coords.
                cand_pos = np.column_stack([xs + x0 + 0.5, ys + y0 + 0.5])
                d2 = ((cand_pos - drone.pos) ** 2).sum(axis=1)
                target = cand_pos[int(d2.argmin())]
                break

            if target is None:
                # All assigned blocks done — fall back to nearest uncov globally.
                uncov_mask = free_mask & ~env.covered
                if self._blacklist[i]:
                    uncov_mask = uncov_mask.copy()
                    for (by, bx) in self._blacklist[i]:
                        if 0 <= by < h and 0 <= bx < w:
                            uncov_mask[by, bx] = False
                    if not uncov_mask.any():
                        self._blacklist[i].clear()
                        uncov_mask = free_mask & ~env.covered
                if not uncov_mask.any():
                    continue
                uy, ux = np.where(uncov_mask)
                cand_pos = np.column_stack([ux + 0.5, uy + 0.5])
                d2 = ((cand_pos - drone.pos) ** 2).sum(axis=1)
                target = cand_pos[int(d2.argmin())]
            # Reset progress tracker when the target switches.
            if prev is None or not np.allclose(target, prev, atol=0.1):
                self._stuck_best_dist[i] = float('inf')
                self._stuck_anchor_t[i] = env.time_seconds
            targets_log[i] = target.copy()

            # ---- attract ----
            offset = target - drone.pos
            dist = float(np.linalg.norm(offset))
            if dist > 1e-9:
                a = cfg.attract_gain * (offset / dist) - cfg.attract_damp_gain * drone.vel
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
                    np.clip(cfg.yaw_align_gain * err - cfg.yaw_damp_gain * drone.yaw_rate,
                            -max_yaw_accel, max_yaw_accel)
                )

        self._last_targets = targets_log
        return actions
