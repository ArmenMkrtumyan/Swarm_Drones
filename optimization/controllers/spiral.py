"""
Spiral-coverage controller.

Each drone independently follows an outward Archimedean spiral
(`r = pitch · θ / (2π)`) centered at its starting position. Waypoints
are sampled at a fixed angular step, so spacing along the spiral is
roughly constant. The drone follows the waypoints with the same
attract + drone-repel + wall-repel + yaw-tracks-velocity stack as
PF / Boustrophedon.

**Why spirals**: a classical pattern for area coverage from a known
starting position with no map information — each drone covers ever-
larger circles without backtracking. Combined with random initial
positions, the swarm naturally splits the map into spiraling
neighborhoods. Inter-drone repulsion handles overlap when spirals
collide.

**Walls** are handled by the same snap-to-free machinery as
`BoustrophedonController` — waypoints inside a wall are replaced with
the nearest free cell, or skipped if no free cell is reachable
within `wall_snap_radius`.

Stateful — instantiate fresh per `env.reset()`.
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
class SpiralConfig:
    """Tunable knobs for `SpiralController`."""
    # Spiral geometry: r = pitch · θ / (2π). Defaults from
    # `tools/grid_search_spiral.py` — wide pitch (4.0) wins because
    # uncoordinated spirals overlap heavily; spreading to larger radii
    # per revolution covers more new ground per drone-second.
    pitch: float = 2.5446   # BO-tuned (was 4.0)
    angle_step_deg: float = 18.0   # waypoint sampling resolution
    max_radius: float = 25.0       # cells; cap so spirals don't run forever

    arrival_radius: float = 0.6
    wall_snap_radius: float = 3.0

    # Movement gains — low attract + high drone repel won the grid (drones
    # need strong personal-space enforcement to avoid stomping each other's
    # spirals).
    attract_gain: float = 2.43   # BO-tuned (was 1.0)
    attract_damp_gain: float = 0.0956   # see PFConfig.attract_damp_gain  (BO; was 0.2)
    drone_repel_gain: float = 7.9536   # BO-tuned (was 10.0)
    drone_repel_range: float = 3.1575   # BO-tuned (was 2.5)
    wall_repel_gain: float = 1.4287   # BO-tuned (was 2.0)
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    yaw_damp_gain: float = 4.9   # critical damping K_d = 2·√K_p; see PFConfig
    velocity_align_threshold: float = 0.05

    # Stuck-detector — see BoustrophedonConfig for the rationale + the
    # reason `stuck_min_progress = 0.8` (wall-slide false-negatives at 0.3).
    stuck_timeout_s: float = 5.0
    stuck_min_progress: float = 0.8   # cells (closer to target)
    stuck_skip_n: int = 3             # waypoints to advance per stuck event


class SpiralController:
    """Per-drone Archimedean-spiral coverage."""

    def __init__(
        self,
        cfg: Optional[SpiralConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or SpiralConfig()
        self.hover_drone_idx = hover_drone_idx
        self._plans: Optional[list[list[np.ndarray]]] = None
        self._idx: Optional[list[int]] = None
        self._stuck_best_dist: Optional[list[float]] = None
        self._stuck_anchor_t: Optional[list[float]] = None
        # Actually-pursued target each step (plan waypoint or fallback);
        # see boustrophedon.py for rationale.
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
        if self._last_targets is not None:
            for i in range(min(n, len(self._last_targets))):
                if self._last_targets[i] is not None:
                    targets[i] = np.asarray(self._last_targets[i], dtype=float)
        return {
            "paths": paths,
            "targets": targets,
            "title_extra": "Spiral",
        }

    def _compute_plan(self, env: CoverageEnv) -> list[list[np.ndarray]]:
        """Build per-drone spiral waypoint lists, snapping wall cells to free."""
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        cfg = self.cfg
        step_rad = math.radians(cfg.angle_step_deg)

        plans: list[list[np.ndarray]] = []
        for drone in env.drones:
            cx, cy = float(drone.pos[0]), float(drone.pos[1])
            waypoints: list[np.ndarray] = []
            theta = 0.0
            while True:
                r = cfg.pitch * theta / (2.0 * math.pi)
                if r > cfg.max_radius:
                    break
                x = cx + r * math.cos(theta)
                y = cy + r * math.sin(theta)
                wp = np.array([x, y], dtype=np.float64)
                # Skip if outside grid.
                if not (0.5 <= x < w - 0.5 and 0.5 <= y < h - 0.5):
                    theta += step_rad
                    continue
                # Snap to nearest free cell if wp lands on a wall.
                ix, iy = int(x), int(y)
                if not free_mask[iy, ix]:
                    replacement = self._snap_to_free(env, wp)
                    if replacement is None:
                        theta += step_rad
                        continue
                    wp = replacement
                waypoints.append(wp)
                theta += step_rad
            plans.append(waypoints)
        return plans

    def _snap_to_free(
        self, env: CoverageEnv, wp: np.ndarray
    ) -> Optional[np.ndarray]:
        cfg = self.cfg
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        wx, wy = int(wp[0]), int(wp[1])
        for dr in range(1, int(cfg.wall_snap_radius) + 1):
            for dy in range(-dr, dr + 1):
                for dx in range(-dr, dr + 1):
                    if dy * dy + dx * dx > cfg.wall_snap_radius ** 2:
                        continue
                    ny, nx = wy + dy, wx + dx
                    if 0 <= ny < h and 0 <= nx < w and free_mask[ny, nx]:
                        return np.array([nx + 0.5, ny + 0.5], dtype=np.float64)
        return None

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
            # Stuck-detection — see boustrophedon.py for rationale.
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
                # Plan exhausted — fall back to nearest uncovered cell.
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

        return actions
