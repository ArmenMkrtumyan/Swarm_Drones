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
    pitch: float = 4.0
    angle_step_deg: float = 18.0   # waypoint sampling resolution
    max_radius: float = 25.0       # cells; cap so spirals don't run forever

    arrival_radius: float = 0.6
    wall_snap_radius: float = 3.0

    # Movement gains — low attract + high drone repel won the grid (drones
    # need strong personal-space enforcement to avoid stomping each other's
    # spirals).
    attract_gain: float = 1.0
    drone_repel_gain: float = 10.0
    drone_repel_range: float = 2.5
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05


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
            while self._idx[i] < len(wps):
                target = wps[self._idx[i]]
                if float(np.linalg.norm(target - drone.pos)) <= cfg.arrival_radius:
                    self._idx[i] += 1
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
                    continue
            else:
                target = wps[self._idx[i]]

            # ---- attract ----
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
