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
    lane_spacing: float = 3.448   # BO-tuned (was 2.5)
    arrival_radius: float = 0.6    # cells; advance to next waypoint when within this
    wall_snap_radius: float = 3.0  # cells; max search radius when a waypoint hits a wall

    # Movement gains (lower attract & repel work better here than the PF
    # defaults — the precomputed plan already does most of the spreading).
    attract_gain: float = 1.2834   # BO-tuned (was 1.0)
    attract_damp_gain: float = 0.7051   # see PFConfig.attract_damp_gain  (BO; was 0.2)
    drone_repel_gain: float = 1.1089   # BO-tuned (was 2.0)
    drone_repel_range: float = 4.3793   # BO-tuned (was 2.0)
    wall_repel_gain: float = 0.6703   # BO-tuned (was 2.0)
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    yaw_damp_gain: float = 4.9   # critical damping K_d = 2·√K_p; see PFConfig
    velocity_align_threshold: float = 0.05

    # Stuck-detector. The controller has no path planner — it sums attract +
    # wall_repel + drone_repel and follows the gradient. When a waypoint sits
    # on the far side of a wall the gradient balances out and the drone idles
    # against the wall (often **oscillating** along it — net motion ≠ progress
    # toward the target). The detector measures *distance-to-target progress*:
    # the drone's best (minimum) distance to the current target so far must
    # shrink by at least `stuck_min_progress` cells within `stuck_timeout_s`
    # seconds, else the waypoint is declared unreachable. On stuck we jump
    # ahead by `stuck_skip_n` waypoints, not one — adjacent waypoints in a
    # boustrophedon row are often on the same side of the wall, so advancing
    # by one usually hits another unreachable waypoint and wastes another
    # timeout window. Cheap and generic; not a substitute for proper path
    # planning, just stops permanent idling.
    stuck_timeout_s: float = 5.0
    stuck_min_progress: float = 0.8   # cells (closer to target). Bumped from
                                      # 0.3 so the detector trips when the
                                      # drone is *sliding along a wall*
                                      # making 0.4-cell-per-5s incremental
                                      # progress (the attract+wall_repel sum
                                      # produces tangential motion that
                                      # technically reduces dist-to-target
                                      # but never actually goes around the
                                      # wall). 0.8 cells/5s ≈ 0.8 m/s of
                                      # *radial* progress — well below the
                                      # 1.8 cells/s max_speed of an
                                      # unobstructed drone, so this doesn't
                                      # false-positive on legitimately-slow
                                      # approaches in tight corridors.
    stuck_skip_n: int = 3             # waypoints to advance per stuck event


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
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        # The waypoint the drone is *actually* pursuing this step — either a
        # plan waypoint or, after plan exhaustion, the nearest-uncov fallback.
        # Surfaced via viz_overlay so the X stays visible in mop-up phase.
        self._last_targets: Optional[list[Optional[np.ndarray]]] = None

    def reset(self) -> None:
        self._plans = None
        self._idx = None
        self._stuck_best_dist = None
        self._stuck_anchor_t = None
        self._last_targets = None

    def viz_overlay(self, env: CoverageEnv) -> dict:
        n = env.n_drones
        targets = [None] * n
        paths = [None] * n
        if self._plans is not None:
            for i in range(min(n, len(self._plans))):
                wps = self._plans[i]
                if wps:
                    paths[i] = np.array([np.asarray(w, dtype=float) for w in wps])
        # Pull the actually-pursued target so the X stays visible during
        # mop-up (after the precomputed plan is exhausted, the drone heads
        # to the nearest uncovered cell each step).
        if self._last_targets is not None:
            for i in range(min(n, len(self._last_targets))):
                if self._last_targets[i] is not None:
                    targets[i] = np.asarray(self._last_targets[i], dtype=float)
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
            self._stuck_best_dist = [float('inf')] * n
            self._stuck_anchor_t = [env.time_seconds] * n
            self._last_targets = [None] * n

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
            # Stuck-detection by distance-to-target progress (NOT net motion).
            # Track the smallest distance the drone has achieved to the current
            # target; if it doesn't shrink by `stuck_min_progress` within
            # `stuck_timeout_s`, the waypoint is unreachable. Oscillating along
            # a wall produces motion but no closing distance, so it triggers.
            if self._idx[i] < len(wps):
                target = wps[self._idx[i]]
                dist = float(np.linalg.norm(drone.pos - target))
                if dist < self._stuck_best_dist[i] - cfg.stuck_min_progress:
                    self._stuck_best_dist[i] = dist
                    self._stuck_anchor_t[i] = env.time_seconds
                    stuck = False
                else:
                    stuck = (env.time_seconds - self._stuck_anchor_t[i]
                             > cfg.stuck_timeout_s)
            else:
                stuck = False

            # Advance past waypoints we've already reached, whose cells are
            # already covered, or that we've been stuck on. On stuck we jump
            # by `stuck_skip_n` (not 1) — adjacent waypoints often share the
            # same unreachable side of a wall.
            h, w = env.grid.shape
            while self._idx[i] < len(wps):
                target = wps[self._idx[i]]
                reached = (
                    float(np.linalg.norm(target - drone.pos)) <= cfg.arrival_radius
                )
                tx, ty = int(target[0]), int(target[1])
                already_covered = (
                    0 <= tx < w and 0 <= ty < h and bool(env.covered[ty, tx])
                )
                if stuck:
                    self._idx[i] = min(self._idx[i] + cfg.stuck_skip_n, len(wps))
                    self._stuck_best_dist[i] = float('inf')
                    self._stuck_anchor_t[i] = env.time_seconds
                    stuck = False
                elif reached or already_covered:
                    self._idx[i] += 1
                    self._stuck_best_dist[i] = float('inf')
                    self._stuck_anchor_t[i] = env.time_seconds
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
                    self._last_targets[i] = None
                    continue
            else:
                target = wps[self._idx[i]]
            self._last_targets[i] = np.asarray(target, dtype=float).copy()

            # ---- attract toward target ----
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
                    np.clip(cfg.yaw_align_gain * err - cfg.yaw_damp_gain * drone.yaw_rate,
                            -max_yaw_accel, max_yaw_accel)
                )

        return actions
