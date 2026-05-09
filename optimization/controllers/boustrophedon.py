"""
Boustrophedon (lawnmower) controller for swarm coverage.

Classical coverage path planning, ~Choset & Lynch 1998 style:
    1. Partition the map into `n_drones` vertical strips of equal width.
    2. Each drone gets one strip and runs a back-and-forth sweep over it,
       with parallel rows spaced by `lane_spacing` cells.
    3. Plan is computed once at first call (`_compute_plan`); subsequent
       calls just track the current waypoint.

Movement: heads toward the current waypoint with a PF-like attract +
drone-repel + wall-repel + yaw-tracks-velocity stack. When close enough
to the waypoint (`arrival_radius`), advance to the next.

Walls inside a drone's strip are dealt with by **snap-to-free**: a
waypoint that lands on a wall cell is replaced with the nearest free cell
(within `wall_snap_radius`); if no free cell exists in range, the
waypoint is skipped entirely. This keeps the drone moving on a
best-effort lawnmower and avoids deadlocking against walls.

**Stateless across episodes** — instantiate fresh per `env.reset()`.
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
class BoustrophedonConfig:
    """Tunable knobs for `BoustrophedonController`."""
    # Path geometry — defaults from `tools/grid_search_boustrophedon.py` 27-config
    # sweep. Wide lane spacing (2.5) wins overall because at large swarm sizes
    # each strip is narrow enough that lane_spacing is ~irrelevant, while at
    # small swarms tight spacing causes the drone to run out of battery before
    # finishing its strip. Mean composite score +0.157 / mean cov 69.5 %.
    lane_spacing: float = 2.5
    arrival_radius: float = 0.6    # cells; advance to next waypoint when within this
    wall_snap_radius: float = 3.0  # cells; max search radius when a waypoint hits a wall

    # Movement gains (lower attract & repel work better here than the PF
    # defaults — the precomputed plan already does most of the spreading).
    attract_gain: float = 1.0
    drone_repel_gain: float = 2.0
    drone_repel_range: float = 2.0
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


class BoustrophedonController:
    """Strip-partitioned lawnmower coverage controller."""

    def __init__(
        self,
        cfg: Optional[BoustrophedonConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or BoustrophedonConfig()
        self.hover_drone_idx = hover_drone_idx
        self._plans: Optional[list[list[np.ndarray]]] = None
        self._idx: Optional[list[int]] = None

    def reset(self) -> None:
        self._plans = None
        self._idx = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        n = env.n_drones
        targets = [None] * n
        paths = [None] * n
        if self._plans is not None:
            for i in range(min(n, len(self._plans))):
                wps = self._plans[i]
                if not wps:
                    continue
                paths[i] = np.array([np.asarray(w, dtype=float) for w in wps])
                k = self._idx[i] if self._idx else 0
                if 0 <= k < len(wps):
                    targets[i] = np.asarray(wps[k], dtype=float)
        return {
            "paths": paths,
            "targets": targets,
            "title_extra": "Boustrophedon",
        }

    def _compute_plan(self, env: CoverageEnv) -> list[list[np.ndarray]]:
        """Per-drone list of waypoints tracing a vertical-strip lawnmower."""
        n = env.n_drones
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        cfg = self.cfg

        # Strip boundaries: split the interior x-range into n equal bands.
        x_lo_int, x_hi_int = 1.0, float(w - 1)  # interior: [1, w-1)
        strip_w = (x_hi_int - x_lo_int) / n
        margin = 0.5     # keep strip endpoints inside cells

        plans: list[list[np.ndarray]] = []
        for i in range(n):
            x_left = x_lo_int + i * strip_w + margin
            x_right = x_lo_int + (i + 1) * strip_w - margin

            waypoints: list[np.ndarray] = []
            y = 1.5
            direction = +1   # +1 = sweep left→right, then advance row
            while y < h - 1:
                if direction > 0:
                    waypoints.append(np.array([x_left, y]))
                    waypoints.append(np.array([x_right, y]))
                else:
                    waypoints.append(np.array([x_right, y]))
                    waypoints.append(np.array([x_left, y]))
                y += cfg.lane_spacing
                direction *= -1

            # Snap any wall-bound waypoint to the nearest free cell (within
            # `wall_snap_radius`); drop if no free cell in range.
            snapped: list[np.ndarray] = []
            for wp in waypoints:
                wx, wy = int(wp[0]), int(wp[1])
                if 0 <= wy < h and 0 <= wx < w and free_mask[wy, wx]:
                    snapped.append(wp)
                    continue
                # Search outward for the nearest free cell.
                replacement = None
                r2_max = cfg.wall_snap_radius ** 2
                for dr in range(1, int(cfg.wall_snap_radius) + 1):
                    found = False
                    for dy in range(-dr, dr + 1):
                        for dx in range(-dr, dr + 1):
                            if dy * dy + dx * dx > r2_max:
                                continue
                            ny, nx = wy + dy, wx + dx
                            if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx]:
                                replacement = np.array(
                                    [nx + 0.5, ny + 0.5], dtype=np.float64
                                )
                                found = True
                                break
                        if found:
                            break
                    if replacement is not None:
                        break
                if replacement is not None:
                    snapped.append(replacement)
                # else: skip this waypoint entirely

            plans.append(snapped)
        return plans

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        if self._plans is None or len(self._plans) != n:
            self._plans = self._compute_plan(env)
            self._idx = [0] * n

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

            wps = self._plans[i]
            # Advance through any waypoints we've already reached.
            while self._idx[i] < len(wps):
                target = wps[self._idx[i]]
                if float(np.linalg.norm(target - drone.pos)) <= cfg.arrival_radius:
                    self._idx[i] += 1
                else:
                    break

            if self._idx[i] >= len(wps):
                # Plan exhausted — head to the closest uncovered cell as a
                # fallback so the drone keeps contributing instead of idling.
                free_uncov = free_mask & ~env.covered
                if free_uncov.any():
                    uy, ux = np.where(free_uncov)
                    uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])
                    d2 = ((uncov_pos - drone.pos) ** 2).sum(axis=1)
                    target = uncov_pos[int(d2.argmin())]
                else:
                    continue
            else:
                target = wps[self._idx[i]]

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

        return actions
