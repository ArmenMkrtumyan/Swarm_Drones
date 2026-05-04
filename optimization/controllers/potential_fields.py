"""
Potential Fields controller for swarm coverage.

Classical Khatib-style superposition:
    F_total = F_attract  +  F_drone_repel  +  F_wall_repel

  - F_attract       pulls each drone toward the nearest uncovered free cell.
  - F_drone_repel   pushes drones away from neighbors within `drone_repel_range`
                    so they spread instead of stacking on the same target.
  - F_wall_repel    pushes drones away from nearby wall cells; supplements the
                    env's hard wall collision (which is correct but abrupt) with
                    a smooth gradient so the drone slides along corridors
                    instead of slamming into them.

Yaw is independent: heading tracks the drone's current velocity direction
(P-controller on heading error → yaw acceleration), so the forward wedge
sweeps the path the drone is actually walking. This is the simplest yaw
strategy that keeps coverage aligned with motion.

Stateless across calls — every step recomputes from `env.covered`. Local-
minimum risk is mitigated by the fact that as cells get covered, the
nearest-uncovered target shifts.

Pattern matches `tools/demo.py:random_policy_with_hover`: returns shape
(n_drones, 3) = [ax, ay, alpha_yaw]. Optional `hover_drone_idx` forces
that drone to zero acceleration so the demo's hover-verify sanity check
keeps working.
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
class PFConfig:
    """
    Tunable knobs for `PotentialFieldsController`.

    All gains are in world-acceleration units (cells/s²) BEFORE the
    `max_accel` magnitude clip — they're the "raw" forces, the env
    handles saturation. Raising a gain makes that term dominate.
    """
    # Attractive pull toward the nearest uncovered cell. 1.0 means the unit
    # vector toward target is added to F_total at full strength.
    attract_gain: float = 1.5

    # Repulsion from other drones — kept slightly above attract_gain so two
    # drones can't sit on the same target. Falls off as 1/r².
    drone_repel_gain: float = 5.0
    drone_repel_range: float = 2.5      # cells; outside this, no repulsion

    # Wall repulsion — softer than drone repulsion (walls are static and the
    # env's hard collision handles the absolute boundary). Smooths corner
    # transitions.
    wall_repel_gain: float = 2.0
    wall_repel_range: float = 1.5       # cells

    # Yaw P-controller: alpha_yaw = yaw_align_gain · heading_error.
    # Headings within `velocity_align_threshold` cells/s of zero velocity
    # don't yaw (heading_error is undefined for stationary drones).
    yaw_align_gain: float = 6.0
    velocity_align_threshold: float = 0.05    # cells/s


class PotentialFieldsController:
    """
    Stateless potential-fields policy for swarm coverage.

    Usage:
        controller = PotentialFieldsController()
        actions = controller(env)
        env.step(actions)

    To preserve the demo's hover-verify sanity check, leave
    `hover_drone_idx=0` (default); set to None to let every drone move.
    """

    def __init__(
        self,
        cfg: Optional[PFConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or PFConfig()
        self.hover_drone_idx = hover_drone_idx

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncovered_mask = free_mask & ~env.covered

        if not uncovered_mask.any():
            # 100 % covered — env.step becomes a no-op anyway, but return
            # zeros explicitly so the controller's intent is clear.
            return actions

        # Cell-center coordinates of every uncovered free cell, shape (M, 2).
        uncov_y, uncov_x = np.where(uncovered_mask)
        uncov_pos = np.column_stack([uncov_x + 0.5, uncov_y + 0.5])

        # Cell-center coordinates of every wall cell, shape (W, 2). Empty grids
        # (no walls) skip wall repulsion entirely.
        wall_y, wall_x = np.where(env.grid == WALL)
        has_walls = len(wall_y) > 0
        wall_pos = (
            np.column_stack([wall_x + 0.5, wall_y + 0.5]) if has_walls else None
        )

        positions = np.array([d.pos for d in env.drones])  # (n, 2)
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel

        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            f_total = np.zeros(2)

            # ---- Attract: pull toward the nearest uncovered cell ----
            d_uncov = uncov_pos - drone.pos
            dist_sq = (d_uncov ** 2).sum(axis=1)
            nearest = int(dist_sq.argmin())
            target_dist = math.sqrt(max(dist_sq[nearest], 1e-12))
            attract_dir = d_uncov[nearest] / max(target_dist, 1e-9)
            f_total += self.cfg.attract_gain * attract_dir

            # ---- Drone repulsion: push away from close neighbors ----
            for j in range(n):
                if j == i:
                    continue
                offset = drone.pos - positions[j]
                dist = float(np.linalg.norm(offset))
                if 1e-9 < dist < self.cfg.drone_repel_range:
                    # Inverse-square; floor the denominator so very close drones
                    # don't blow up to infinity (env.step's max_accel clip would
                    # catch it but the magnitudes hide the attract direction).
                    f_total += (
                        self.cfg.drone_repel_gain
                        * offset
                        / max(dist ** 2, 0.05)
                    )

            # ---- Wall repulsion: gradient from nearby walls ----
            if has_walls:
                d_wall = drone.pos - wall_pos
                wall_dists = np.linalg.norm(d_wall, axis=1)
                in_range = (wall_dists > 1e-9) & (
                    wall_dists < self.cfg.wall_repel_range
                )
                if in_range.any():
                    d_in = d_wall[in_range]
                    dists_in = wall_dists[in_range]
                    # Each wall cell contributes a 1/r² push outward; we sum
                    # them (not average) so a corner exerts the combined
                    # gradient of both walls.
                    weights = 1.0 / np.maximum(dists_in ** 2, 0.05)
                    f_total += (
                        self.cfg.wall_repel_gain
                        * (d_in.T * weights).sum(axis=1)
                    )

            # ---- Saturate to max_accel (preserves direction) ----
            f_norm = float(np.linalg.norm(f_total))
            if f_norm > max_accel:
                f_total *= max_accel / f_norm

            actions[i, 0] = f_total[0]
            actions[i, 1] = f_total[1]

            # ---- Yaw: P-control toward velocity direction ----
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > self.cfg.velocity_align_threshold:
                target_heading = math.atan2(v[1], v[0])
                err = (target_heading - drone.heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                alpha = self.cfg.yaw_align_gain * err
                actions[i, 2] = float(
                    np.clip(alpha, -max_yaw_accel, max_yaw_accel)
                )

        return actions
