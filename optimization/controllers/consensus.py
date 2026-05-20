"""
Consensus-Based Coordination controller.

Decentralized coverage via local Voronoi partitioning (Lloyd-style):

    1. Each drone i polls neighbors within `DroneConfig.comm_range`
       (`env.neighbors(i)`). The "communication group" is
       {i} ∪ neighbors(i).
    2. The group implicitly partitions the *uncovered free cells* by
       Euclidean distance — each cell is "owned" by whichever drone in the
       group is closest. (Cells outside any group's view fall to the global
       Voronoi default; with the default `comm_range = None` the partition
       becomes a global Voronoi over all drones — the textbook centralized
       baseline.)
    3. Drone i computes the centroid of its owned-and-uncovered cells and
       heads toward it (the Lloyd-flow attractor).
    4. Repulsion from neighbors and walls is identical to Potential Fields,
       so close-quarters dynamics are familiar.
    5. Yaw tracks velocity (same as PF) so the wedge sweeps the path.

Why this is the natural step up from PF: PF has every drone chase the
*same* nearest uncovered cell, then rely on inter-drone repulsion to
spread them out. Consensus replaces "all chase the nearest cell" with
"each drone is responsible for its own region" — the swarm assigns
territory via local agreement instead of competing for the same target.
The expected gain shows up in `wasted_visits_total` (less re-coverage)
and `overlap_cells_m2` (less double-coverage).

Stateless across calls. Returns shape (n_drones, 3) like PF.
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
class ConsensusConfig:
    """
    Tunable knobs. Repulsion gains intentionally match PF defaults so the
    only difference between the two controllers is the attractor (centroid
    vs. nearest uncovered cell).

    Same `sparse()` / `dense()` preset story as PF — see those classmethods
    below. Sparse swarms gain less from tuning here than for PF (the
    Voronoi attractor already enforces territory partition).
    """
    attract_gain: float = 4.7894   # BO-tuned (was 1.5)
    attract_damp_gain: float = 0.6809   # see PFConfig.attract_damp_gain  (BO; was 0.2)

    drone_repel_gain: float = 1.7859   # BO-tuned (was 5.0)
    drone_repel_range: float = 1.3672   # BO-tuned (was 2.5)
    wall_repel_gain: float = 3.0575   # BO-tuned (was 2.0)
    wall_repel_range: float = 1.5

    yaw_align_gain: float = 6.0
    yaw_damp_gain: float = 4.9   # critical damping K_d = 2·√K_p; see PFConfig
    velocity_align_threshold: float = 0.05

    # When a drone owns zero uncovered cells in its Voronoi region (common
    # late in the run when its area is fully covered), fall back to the
    # nearest uncovered cell anywhere on the map. Without this, the drone
    # would have no attractor and just drift on repulsion alone.
    fallback_to_nearest_global: bool = True

    # Attractor target within the Voronoi region:
    #   "centroid" — Lloyd-style: pull toward the mean of owned uncovered
    #     cells. Smooth flow but slow on maze topologies because the centroid
    #     can be inside a wall or unreachable through corridors.
    #   "nearest" — pull toward the nearest owned uncovered cell. Faster and
    #     more robust on mazes; matches PF's behavior restricted to the
    #     drone's Voronoi share. This is the default — it preserves the
    #     consensus property (each drone owns its territory) while being
    #     aggressive enough to actually clear that territory.
    target_strategy: str = "nearest"   # "nearest" | "centroid"

    @classmethod
    def sparse(cls) -> "ConsensusConfig":
        """
        Preset tuned for sparse swarms (n ≤ 2). Lighter repulsion and lower
        attract gain reduce oscillation when drones rarely interact.
        """
        return cls(
            attract_gain=1.5,
            drone_repel_gain=2.0,
            drone_repel_range=2.5,
        )

    @classmethod
    def dense(cls) -> "ConsensusConfig":
        """
        Same 7-knob config as `PFConfig.dense()` — both controllers share
        the dense-swarm optimum, which suggests the gain comes from the
        underlying force dynamics rather than the attractor type.
        """
        return cls(
            attract_gain=2.22,
            drone_repel_gain=4.13,
            drone_repel_range=3.93,
            wall_repel_gain=3.48,
            wall_repel_range=1.43,
            yaw_align_gain=2.94,
            velocity_align_threshold=0.152,
        )


class ConsensusController:
    """
    Voronoi-flavored consensus controller for swarm coverage.

    Usage matches `PotentialFieldsController`:
        controller = ConsensusController()
        actions = controller(env)
        env.step(actions)
    """

    def __init__(
        self,
        cfg: Optional[ConsensusConfig] = None,
        hover_drone_idx: Optional[int] = 0,
    ) -> None:
        self.cfg = cfg or ConsensusConfig()
        self.hover_drone_idx = hover_drone_idx

    def viz_overlay(self, env: CoverageEnv) -> dict:
        # Compute the (group-local Voronoi) partition over uncovered cells +
        # the comm graph edges, fresh from current state.
        h, w = env.grid.shape
        free_mask = env.grid == FREE
        uncov_mask = free_mask & ~env.covered
        n = env.n_drones
        positions = np.array([d.pos for d in env.drones])

        # Partition over ALL free cells (so the regions show even before any
        # cell is covered — purely educational, no group-local restriction).
        ys = np.arange(h)[:, None] + 0.5
        xs = np.arange(w)[None, :] + 0.5
        d2 = ((positions[:, 0:1, None] - xs[None]) ** 2
              + (positions[:, 1:2, None] - ys[None]) ** 2)
        regions = d2.argmin(axis=0).astype(np.int32)
        regions[~free_mask] = -1

        # Communication-graph edges from env.neighbors().
        edges = []
        for i in range(n):
            for j in env.neighbors(i):
                if i < j:
                    edges.append((i, j))

        # Per-drone target = chosen attractor from current uncov cells.
        targets = [None] * n
        if uncov_mask.any():
            uy, ux = np.where(uncov_mask)
            uncov_pos = np.column_stack([ux + 0.5, uy + 0.5])
            for i, drone in enumerate(env.drones):
                if i == self.hover_drone_idx:
                    continue
                group = np.concatenate([[i], env.neighbors(i)])
                group_pos = positions[group]
                diffs = uncov_pos[:, None, :] - group_pos[None, :, :]
                dists_sq = (diffs ** 2).sum(axis=2)
                owner_in_group = dists_sq.argmin(axis=1)
                mine = owner_in_group == 0
                if mine.any():
                    owned = uncov_pos[mine]
                    if self.cfg.target_strategy == "centroid":
                        targets[i] = owned.mean(axis=0)
                    else:
                        d_owned = ((owned - drone.pos) ** 2).sum(axis=1)
                        targets[i] = owned[int(d_owned.argmin())].copy()

        return {
            "regions": regions,
            "edges": edges,
            "targets": targets,
            "title_extra": "Consensus",
        }

    def __call__(self, env: CoverageEnv) -> np.ndarray:
        n = env.n_drones
        actions = np.zeros((n, 3), dtype=np.float64)

        free_mask = env.grid == FREE
        uncovered_mask = free_mask & ~env.covered
        if not uncovered_mask.any():
            return actions

        uncov_y, uncov_x = np.where(uncovered_mask)
        uncov_pos = np.column_stack([uncov_x + 0.5, uncov_y + 0.5])  # (M, 2)

        wall_y, wall_x = np.where(env.grid == WALL)
        has_walls = len(wall_y) > 0
        wall_pos = (
            np.column_stack([wall_x + 0.5, wall_y + 0.5]) if has_walls else None
        )

        positions = np.array([d.pos for d in env.drones])  # (n, 2)
        max_accel = env.drone_cfg.max_accel
        max_yaw_accel = env.drone_cfg.max_yaw_accel
        cfg = self.cfg

        for i, drone in enumerate(env.drones):
            if i == self.hover_drone_idx:
                continue

            # ---- Voronoi attractor: centroid of "my" uncovered cells ----
            # Communication group = {i} ∪ neighbors(i). Cells where drone i
            # is the closest member of this group are i's owned region.
            group = np.concatenate([[i], env.neighbors(i)])
            group_pos = positions[group]                          # (G, 2)
            # Pairwise distances cell ↔ each group member, shape (M, G).
            diffs = uncov_pos[:, None, :] - group_pos[None, :, :]
            dists_sq = (diffs ** 2).sum(axis=2)
            owner_in_group = dists_sq.argmin(axis=1)
            mine = owner_in_group == 0   # 0 == "drone i" since we put i first
            owned_cells = uncov_pos[mine]

            if len(owned_cells) > 0:
                if cfg.target_strategy == "centroid":
                    target = owned_cells.mean(axis=0)
                else:  # "nearest"
                    d_owned = owned_cells - drone.pos
                    target = owned_cells[int((d_owned ** 2).sum(axis=1).argmin())]
            elif cfg.fallback_to_nearest_global:
                # No uncovered cells in my Voronoi region — chase the nearest
                # uncovered cell anywhere (degenerate to PF for this drone).
                d_uncov = uncov_pos - drone.pos
                dist_sq = (d_uncov ** 2).sum(axis=1)
                target = uncov_pos[int(dist_sq.argmin())]
            else:
                target = drone.pos.copy()  # no attractor

            offset_to_target = target - drone.pos
            target_dist = float(np.linalg.norm(offset_to_target))
            if target_dist > 1e-9:
                attract_dir = offset_to_target / target_dist
                f_total = cfg.attract_gain * attract_dir - cfg.attract_damp_gain * drone.vel
            else:
                f_total = np.zeros(2)

            # ---- Drone repulsion (same form as PF) ----
            for j in range(n):
                if j == i:
                    continue
                offset = drone.pos - positions[j]
                dist = float(np.linalg.norm(offset))
                if 1e-9 < dist < cfg.drone_repel_range:
                    f_total += (
                        cfg.drone_repel_gain * offset
                        / max(dist ** 2, 0.05)
                    )

            # ---- Wall repulsion (same form as PF) ----
            if has_walls:
                d_wall = drone.pos - wall_pos
                wall_dists = np.linalg.norm(d_wall, axis=1)
                in_range = (wall_dists > 1e-9) & (
                    wall_dists < cfg.wall_repel_range
                )
                if in_range.any():
                    d_in = d_wall[in_range]
                    dists_in = wall_dists[in_range]
                    weights = 1.0 / np.maximum(dists_in ** 2, 0.05)
                    f_total += (
                        cfg.wall_repel_gain * (d_in.T * weights).sum(axis=1)
                    )

            # ---- Saturate ----
            f_norm = float(np.linalg.norm(f_total))
            if f_norm > max_accel:
                f_total *= max_accel / f_norm

            actions[i, 0] = f_total[0]
            actions[i, 1] = f_total[1]

            # ---- Yaw: align with velocity ----
            v = drone.vel
            v_norm = float(np.linalg.norm(v))
            if v_norm > cfg.velocity_align_threshold:
                target_heading = math.atan2(v[1], v[0])
                err = (target_heading - drone.heading + math.pi) % (
                    2 * math.pi
                ) - math.pi
                alpha = cfg.yaw_align_gain * err - cfg.yaw_damp_gain * drone.yaw_rate
                actions[i, 2] = float(
                    np.clip(alpha, -max_yaw_accel, max_yaw_accel)
                )

        return actions
