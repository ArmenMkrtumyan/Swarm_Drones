"""
Composite multi-criteria scoring for swarm-coverage runs.

The energy-aware objective in `README.md` lists multiple components
(coverage, overlap, wasted visits, energy, time, scalability). For
optimizer-driven hyperparameter search (e.g. grid search), we need a
**single scalar** to declare a "winner". This module is that scalar.

Default weighting (`DEFAULT_WEIGHTS`) is a starting point — every term is
normalized into [0, 1] so the weights are directly comparable. Higher
score = better. Coverage is the dominant positive term; overlap, waste,
and energy enter as penalties.

    score = w_cov · coverage_fraction
          − w_overlap · (overlap_m² / total_free_area_m²)
          − w_wasted  · (wasted_visits / max(1, total_entries))
          − w_energy  · (energy_used_j / energy_budget_j)

`total_entries = wasted_visits + unique_cells_visited`, so
`wasted/total_entries` ∈ [0, 1) is "the fraction of every cell-entry that
was redundant". Normalizing this way keeps the score comparable across
swarm sizes and run lengths (a longer run that revisits a lot is
penalized in the same way as a short run that revisits a lot).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ScoreWeights:
    """Multi-criteria weighting. All in `score` units (dimensionless)."""
    coverage: float = 1.0     # primary positive term; 100% coverage = +1.0
    overlap:  float = 0.30    # max penalty for fully-overlapped map = -0.30
    wasted:   float = 0.30    # max penalty when ~all entries are wasted = -0.30
    energy:   float = 0.20    # max penalty for spending the full battery budget = -0.20


DEFAULT_WEIGHTS = ScoreWeights()


def composite_score(
    *,
    coverage_fraction: float,
    overlap_m2: float,
    free_area_m2: float,
    wasted_visits: int,
    unique_cells_visited: int,
    energy_used_j: float,
    energy_budget_j: float,
    weights: Optional[ScoreWeights] = None,
) -> float:
    """
    Compute the composite score for a single run.

    Args:
        coverage_fraction: in [0, 1].
        overlap_m2:        env.overlap_cells_m2() — area touched by ≥ 2 drones.
        free_area_m2:      total free (coverable) area in m².
        wasted_visits:     env.wasted_visits_total() — entry events past the
                           first for any cell, summed across all drones.
        unique_cells_visited: distinct free cells any drone has entered
                              (= int((env.covered & free_mask).sum())).
        energy_used_j:     total joules consumed across the swarm.
        energy_budget_j:   total joules in the swarm at full charge
                           (= n_drones · BatteryConfig.initial_energy_j).
        weights:           optional `ScoreWeights` override; defaults to
                           `DEFAULT_WEIGHTS`.

    Returns:
        Scalar score; higher is better. Range is roughly [-0.8, +1.0].
    """
    w = weights or DEFAULT_WEIGHTS

    overlap_frac = overlap_m2 / free_area_m2 if free_area_m2 > 0 else 0.0
    total_entries = max(1, wasted_visits + unique_cells_visited)
    wasted_frac = wasted_visits / total_entries
    energy_frac = energy_used_j / energy_budget_j if energy_budget_j > 0 else 0.0

    return (
        w.coverage * coverage_fraction
        - w.overlap * overlap_frac
        - w.wasted * wasted_frac
        - w.energy * energy_frac
    )
