# Swarm Drone Optimization: 2D Coverage Platform

## Abstract

Multiple drones must cover a target region as effectively as possible while minimizing energy consumption. The objective is multi-criteria:

- maximize total area covered
- avoid covering the same area repeatedly
- minimize total energy usage
- avoid collisions with each other and with obstacles

This repository is a NumPy + matplotlib testbed for swarm-coverage control: continuous-action point-mass drones with speed/accel caps and an F450-calibrated battery model.

It has two halves: an **optimization** part that decides how drones move or split the area, and a **simulation** part that demonstrates and evaluates the behavior.
> Why 2D? It's a cheap **falsification** tool for the eventual Isaac Sim port. If an algorithm fails here, most likely it will fail in 3D simulation too.

---

## Prototype Drone

The simulator is calibrated to the actual hardware this project flies (**[Hawk's Work F450](https://www.hawks-work.com/pages/f450-drone)**) — a 1.3 kg quadcopter with a Pixhawk 2.4.8 flight controller, a 3S 4200 mAh LiPo, and a forward-facing STEEReoCAM Nano stereo camera.

Full hardware breakdown (per-component masses, camera geometry, hover times, cruise / range estimates, calibration sources) lives in [`docs/f450-reference.md`](docs/f450-reference.md). 3S/4S battery options for the F450 frame are in [`docs/battery-model.md`](docs/battery-model.md).

---

## Documentation

Detailed docs live in [`docs/`](docs/) — each focused on one topic:

| Doc | What's in it |
|---|---|
| [`docs/setup.md`](docs/setup.md) | Install, GUI deps for Linux/WSL, demo run modes, CLI flags, map editor, outputs layout. |
| [`docs/simulation-model.md`](docs/simulation-model.md) | World scale (5 m/cell), per-step flow contract, drone dynamics, coverage + visit-count metrics, intentional gaps to Isaac. |
| [`docs/battery-model.md`](docs/battery-model.md) | 3S/4S battery options, mass-aware hover power, voltage cutoff, calibration. |
| [`docs/f450-reference.md`](docs/f450-reference.md) | Hawk's Work F450 hardware spec, camera geometry|
| [`docs/verification.md`](docs/verification.md) | The three verification scripts (`test_overlap`, `test_flight_time`, `test_distance`) — what each asserts, how to run headless or `--gui`. |

---

## Test maps

All three tracks evaluate against the same three saved maps. Every map is **33×33 cells at 5 m/cell = 165 m × 165 m total** (27,225 m² with the 1-cell-thick boundary wall, 24,025 m² interior). They differ only in obstacle density:

| ![open_33](docs/images/map_open_33.png) | ![partial_33](docs/images/map_partial_33.png) | ![closed_33](docs/images/map_closed_33.png) |
|:---:|:---:|:---:|
| **`open_33`** | **`partial_33`** | **`closed_33`** |
| 88.2 % open | 73.0 % open | 53.1 % open |
| 24,025 m² free | 19,875 m² free | 14,450 m² free |

`tools/sweep.py` runs every controller on all three maps with 4 swarm sizes × 3 seeds, so each track's reported numbers cover the full (map × n_drones × seed) grid.

---

## Research tracks

Our group is exploring algorithm families in parallel, then converging on the most promising path:

| Track | Family | Owner | Status in this repo |
|---|---|---|---|
| 1 | Classical coverage and geometry-based methods (coverage path planning, grid decomposition, boustrophedon/lawnmower, Voronoi partitioning, task allocation) | Elen | Not synced yet |
| 2 | Metaheuristic optimization (PSO, Genetic Algorithms, Ant Colony, Simulated Annealing, Grey Wolf) | Raffi | **completed** |
| 3 | Learning and control-based methods (Potential Fields, Consensus-Based Coordination, Multi-Agent RL) | Armen | **completed** |

---

## Energy-aware objective

All methods — across all three tracks — must be evaluated against the same multi-criteria objective. For RL it's a reward; for Potential Fields and Consensus it's a cost / evaluation function.

Components:

- coverage percentage
- total distance traveled
- estimated energy consumption (see [`docs/battery-model.md`](docs/battery-model.md))
- overlap between drones
- collision count / proximity violations
- smoothness of movement
- time to complete coverage
- scalability as the number of drones grows

Keeping this objective **identical across 2D and Isaac** is the discipline that lets results transfer between environments.

---

## Quickstart

```bash
# First time (Linux/WSL — for GUI also: sudo apt install python3-tk python3-pil.imagetk)
python3 -m venv .optim_env
source .optim_env/bin/activate
pip install -r requirements.txt

# Run the demo
python tools/demo.py --gui --map maze --drones 4

# Verify the physics
python verification_scripts/test_overlap.py
python verification_scripts/test_flight_time.py
python verification_scripts/test_distance.py
```

Full setup details, GUI dependencies for Linux/WSL, and all flags are in [`docs/setup.md`](docs/setup.md).

## Visual demos per algorithm

Every algorithm is demoed on the same two **shared** maps so behavior is directly comparable across controllers:

- `maps/open_33.npy` — 33×33 fully-open arena (only boundary walls; pure-behavior baseline)
- `maps/partial_33.npy` — 33×33 with partial obstacles (stresses wall-snap / stuck-detection / path-around-walls)

If you want different maps, draw your own with the editor (left-click paint, right-click erase, `s` saves, `q` quits):

```bash
python3 tools/editor.py --size 33 --out maps/<your-name>.npy
```

All demos below use `--drones 5 --all-active --seed 1` for a like-for-like comparison (drone count matches every Track 3 MARL checkpoint, `--all-active` removes the drone-0-hovers convention so every drone is visibly controller-driven, fixed seed = reproducible spawns).

### Track 1 — Classical

```bash
# Boustrophedon (lawnmower)
python3 tools/demo.py --gui --policy boustrophedon --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy boustrophedon --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# Spiral (per-drone Archimedean spirals)
python3 tools/demo.py --gui --policy spiral --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy spiral --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# VoronoiPartition (region-assigned nearest-uncov)
python3 tools/demo.py --gui --policy voronoi --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy voronoi --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# GridDecomposition (block-assigned nearest-first)
python3 tools/demo.py --gui --policy grid_decomp --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy grid_decomp --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# STC (Spanning Tree Coverage)
python3 tools/demo.py --gui --policy stc --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy stc --drones 5 --map-file maps/partial_33.npy --all-active --seed 1
```

### Track 2 — Metaheuristic

```bash
# PSO (Particle Swarm Optimization)
python3 tools/demo.py --gui --policy pso --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy pso --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# GA (Genetic Algorithm)
python3 tools/demo.py --gui --policy ga --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy ga --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# ACO (Ant Colony Optimization)
python3 tools/demo.py --gui --policy aco --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy aco --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# SA (Simulated Annealing)
python3 tools/demo.py --gui --policy sa --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy sa --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# GWO (Grey Wolf Optimizer)
python3 tools/demo.py --gui --policy gwo --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy gwo --drones 5 --map-file maps/partial_33.npy --all-active --seed 1
```

### Track 3 — Learning / control-based

```bash
# Potential Fields
python3 tools/demo.py --gui --policy pf --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy pf --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# Consensus
python3 tools/demo.py --gui --policy consensus --drones 5 --map-file maps/open_33.npy    --all-active --seed 1
python3 tools/demo.py --gui --policy consensus --drones 5 --map-file maps/partial_33.npy --all-active --seed 1

# MARL (PPO) — the demo auto-picks outputs/marl/marl_ppo_n5.zip; if your
# checkpoints live elsewhere, pass --marl-checkpoint <path>.
python3 tools/demo.py --gui --policy marl --drones 5 --map-file maps/open_33.npy    --all-active --seed 1 --marl-checkpoint outputs_control_based/marl/marl_ppo_n5.zip
python3 tools/demo.py --gui --policy marl --drones 5 --map-file maps/partial_33.npy --all-active --seed 1 --marl-checkpoint outputs_control_based/marl/marl_ppo_n5.zip
```

### Baseline reference

```bash
# Random Gaussian (sanity baseline — see Track 1 headline: random is hard to beat at n=2)
python3 tools/demo.py --gui --policy random --drones 5 --map-file maps/open_33.npy    --seed 1
python3 tools/demo.py --gui --policy random --drones 5 --map-file maps/partial_33.npy --seed 1
```

Tip: drop `--gui` for a fast headless smoke test that writes `outputs/png/demo_{initial,final}.png` and `coverage_curve.png`. Add `--save <tag>` to persist the GIF / PNGs as `SAVED_<tag>_*` instead of overwriting the scratch files.

---

## Results

### Baseline — Random Gaussian

Every benchmark table below also reports a **Random Gaussian** policy as the no-thought baseline. It's deliberately the simplest policy that respects the env's action contract: each step, every drone receives an independent 3-D action drawn from a Gaussian:

```
aᵢ = (axᵢ, ayᵢ, α_yaw,ᵢ),   each component ~ N(0, σ²)
```

with `σ = 1.5` (in env action units — cells/s² for translation, rad/s² for yaw). The env then magnitude-clips translation at `max_accel = 2.5 cells/s²` and yaw-clips at `max_yaw_accel = 6 rad/s²` — so `σ = 1.5` puts ~2/3 of samples *inside* the clip on each axis, with the tails saturating.

No state, no target, no memory. The drone receives random pushes, the env integrates them, and over a 734-second physics-bounded run the drone eventually visits a substantial chunk of the map by sheer diffusion. Implementation: `tools/sweep_track*.py:random_policy()` — three lines, including the docstring:

```python
def random_policy(env, rng):
    return rng.normal(0.0, 1.5, size=(env.n_drones, 3))
```

**Why we keep it.** Random Gaussian is the lower bound: if a "smart" controller can't beat it on a given (map × n_drones) cell, that controller is doing something *worse than nothing*. From the cross-track leaderboard it lands at mean coverage 96.5 % — competitive on coverage but **2-3× worse on energy** and **5-6× worse on overlap/wasted-visits** than the partition-based methods. That's the right shape: Random brute-forces coverage by visiting every cell many times; the principled methods reach the same coverage at far less cost.

(The demo's separate `random_policy_with_hover` in `tools/demo.py` is a slightly fancier visual-only variant — drone 0 is pinned at zero for the hover-verify sanity check, and the moving drones use a brake-then-push pattern so the velocity wedge stays aligned with motion. That variant isn't used in benchmarks; the three-line Gaussian above is.)

---

### Track 1 — Classical / geometry-based

Five classical coverage algorithms implemented as **runtime controllers** matching the same `policy_fn(env) -> (n_drones, 3)` contract as Tracks 2 and 3. Each tuned via **Bayesian Optimization** (`tools/bo_search.py`, Optuna TPE, 30 trials per algorithm over a 12-cell eval grid) — superseding the original 27-config grid search (`tools/grid_search_{...}.py`, still in the repo). BO winners are stored in `outputs/bo/<algo>.json` and applied to each controller's `*Config` dataclass defaults via `tools/bo_apply.py --apply`. Per-track frozen artifacts now live in `outputs_classical/` — sweep results (`sweep_track1_results.csv`, plus the pre-BO baseline at `sweep_track1_results_pre_bo.csv` for comparison), the multi-metric leaderboard (`leaderboard.png`), and per-track CSVs in `leaderboards/`. The active Track 1 workspace is still `outputs/`.

Algorithms:
1. **Boustrophedon** (lawnmower) — partition map into vertical strips, drone *i* runs back-and-forth lanes inside its strip. Plan precomputed once, snap-to-free for waypoints that hit walls.
2. **Spiral** — each drone follows an outward Archimedean spiral (`r = pitch · θ / 2π`) from its initial position. No coordination.
3. **VoronoiPartition** — Voronoi-assign every free cell to the nearest drone start (one-time, frozen). Each drone heads to nearest uncovered cell *in its own region*. Static counterpart to Track 3's `Consensus` (which re-elects every step).
4. **GridDecomposition** — divide map into `block_size × block_size` rectangular blocks, assign each block to nearest drone start, drones visit blocks in nearest-first order.
5. **STC (Spanning Tree Coverage)** — Voronoi partition + BFS-ordered walk through every cell in the partition. Gabriely & Rimon, 2001.

**Mathematical formulation.** All five algorithms share the **same per-step motion law** — the only thing that differs is how each drone picks its target `tᵢ`. The controller emits a 2D linear acceleration:

```
aᵢ = K_a · ûᵢ→tᵢ  −  K_d · vᵢ  +  Σⱼ K_dr·(pᵢ − pⱼ)/max(‖pᵢ − pⱼ‖², ε)  +  Σ_w K_wr·(pᵢ − p_w)/max(‖pᵢ − p_w‖², ε)
   └─ attract ─┘   └─ damp ─┘   └─── drone–drone repulsion, j s.t. ‖·‖<r_dr ───┘   └─── wall repulsion, w s.t. ‖·‖<r_wr ────┘
```

where `ûᵢ→tᵢ = (tᵢ − pᵢ)/‖tᵢ − pᵢ‖` is the unit vector toward the target, `vᵢ` is the drone's current velocity, `K_a, K_d, K_dr, K_wr` are tunable gains (`attract_gain`, `attract_damp_gain`, `drone_repel_gain`, `wall_repel_gain` in code), and the `1/max(d², ε)` form caps each repulsion when `d→0` to avoid blow-up. The result is magnitude-clipped: `‖aᵢ‖ ≤ max_accel`.

Yaw is a separate PD controller toward the **velocity direction** (so the camera tracks where the drone is going):

```
α_yaw,ᵢ = K_p · wrap(atan2(v_y, v_x) − θᵢ)  −  K_d_yaw · ωᵢ
```

clipped to `|α_yaw| ≤ max_yaw_accel`, then integrated by the env: `ωₜ₊₁ = clip(ωₜ + α·Δt, ±max_yaw_rate)`, `θₜ₊₁ = wrap(θₜ + ωₜ₊₁·Δt)`. Critical damping `K_d_yaw = 2·√K_p ≈ 4.9` is what kills the overshoot/oscillation that pure-P yaw produces on a second-order system.

The **target rule `tᵢ`** is what differs algorithm-to-algorithm:

| Algorithm | Plan generated at *t=0* | Target this step (`tᵢ`) |
|---|---|---|
| **Boustrophedon** | Vertical strips: drone *i* owns `x ∈ [1 + i·w_s + 0.5, 1 + (i+1)·w_s − 0.5]` where strip width `w_s = (W−1)/n`. Waypoint list alternates `(x_left, yₖ)` and `(x_right, yₖ)` for `yₖ = 1.5 + k·ℓ`, lane spacing `ℓ`. Wall-snap: if `(x_wp, y_wp)` lands on a wall cell, replace with the nearest free cell within `wall_snap_radius`; drop if none. | `tᵢ = wpsᵢ[k]` where `k` is the lowest waypoint index not yet covered, blacklisted, or stuck (see follower rules below). |
| **Spiral** | Per-drone Archimedean spiral from `(x₀, y₀)`: `(x, y) = (x₀ + r·cos θ, y₀ + r·sin θ)` with `r = pitch · θ/(2π)`, sampled every `Δθ = angle_step_deg` until `r > max_radius`. Same wall-snap as Boustrophedon. | Same skip rules as Boustrophedon. |
| **VoronoiPartition** | Frozen Voronoi cells over free cells: `owner(y, x) = argminⱼ ‖pⱼ(0) − (x+0.5, y+0.5)‖`. Each drone *i*'s region `Rᵢ = {(y, x) : owner(y, x) = i}`. | `tᵢ = argmin_{c ∈ Rᵢ ∩ uncov ∩ ¬blacklistᵢ} ‖pᵢ − c‖`. Fallback to global nearest uncov if `Rᵢ` is fully covered. |
| **GridDecomposition** | Block grid of size `B`: blocks `b = (y₀, y₀+B, x₀, x₀+B)`, centroid `c_b = mean(free cells in b)`. Drone *i* owns blocks with `argminⱼ ‖pⱼ(0) − c_b‖ = i`, sorted ascending by `‖c_b − pᵢ(0)‖`. | `tᵢ = argmin_{c ∈ bₖ ∩ uncov ∩ ¬blacklistᵢ} ‖pᵢ − c‖` where `bₖ` is drone *i*'s currently-active block; advance to `bₖ₊₁` when `bₖ` is fully covered. |
| **STC** | Voronoi cells (as above), then a BFS-order traversal inside each region from drone start: `Wᵢ = BFS₄(Rᵢ, start=⌊pᵢ(0)⌋)` using 4-connected neighbors `{(y±1, x), (y, x±1)}`. | `tᵢ = (x + 0.5, y + 0.5)` where `(y, x) = Wᵢ[k]`, `k` = lowest walk index not covered, blacklisted, or stuck. |

**Waypoint follower (Boustrophedon + Spiral only).** Both controllers carry a precomputed `wps` list per drone and walk it with `self._idx[i]`. The follower runs three skip rules **before** computing this step's attract/repel force, in the order below:

1. **Skip if already at the waypoint.** `‖target − drone.pos‖ ≤ arrival_radius` (default 0.6 cells). Standard waypoint-arrival.
2. **Skip if the waypoint's cell is already covered** — by this drone *or any other*. The plan is precomputed once; the swarm covers ground as it goes, and many planned waypoints become redundant. Without this skip, a drone heads to a cell that's already done (you'd see the green target X sitting inside a covered region). With it, the active waypoint always lands on an uncovered cell.
3. **Skip if stuck (unreachable).** Neither controller has a path planner — the move command is just `attract + drone_repel + wall_repel`, gradient-followed. When a waypoint sits on the far side of a wall, attract pulls into the wall and wall_repel pushes back; the two cancel and the drone either freezes or oscillates along the wall, never closing distance to the target. The detector tracks **distance-to-target progress**: the smallest distance the drone has achieved to the current target. If that minimum doesn't shrink by at least `stuck_min_progress` (0.3 cells) within `stuck_timeout_s` (5 s), the waypoint is declared unreachable. On a stuck event the follower jumps ahead by `stuck_skip_n` (3 by default), not 1 — adjacent waypoints in a lawnmower row tend to share the same unreachable side of the wall, so single-step advancement just burns another timeout window on the next unreachable target. The "distance progress" metric (rather than "any motion") is necessary because oscillating along a wall has plenty of net motion but zero closing distance — a previous "net displacement since anchor" version of this detector failed exactly that way. Not a substitute for proper path planning; it just stops permanent idling.

The other three classical algorithms don't need any of this: VoronoiPartition / GridDecomposition / STC all **recompute the target each step** from the current uncovered-cells mask, so an unreachable or covered cell never persists as the target. (Track 2's GA / SA store targets across steps and use explicit stale-target cleanup to the same effect; Track 3's PF / Consensus have no discrete target.)

**Final benchmark — coverage % by (map × n_drones), `tools/sweep_track1.py`, mean over 3 seeds (Bayesian-Optimized configs from `tools/bo_search.py`):**

| Map | n | Random | Boustrophedon | Spiral | VoronoiPartition | GridDecomposition | STC |
|---|---|---|---|---|---|---|---|
| `open_33` | 2 | 95.9 | **100** | **100** | **100** | **100** | 89.8 |
| `open_33` | 5 | 99.8 | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 10 | **100** | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 20 | **100** | **100** | **100** | **100** | **100** | **100** |
| `partial_33` | 2 | 92.5 | 75.0 | 93.1 | **100** | 76.8 | 88.0 |
| `partial_33` | 5 | 99.7 | **100** | **100** | **100** | **100** | **100** |
| `partial_33` | 10 | **100** | **100** | **100** | **100** | **100** | **100** |
| `partial_33` | 20 | **100** | **100** | **100** | **100** | **100** | **100** |
| `closed_33` | 2 | 75.6 | 19.6 | 64.9 | 74.4 | 59.1 | **91.3** |
| `closed_33` | 5 | 95.5 | 55.2 | 87.3 | 91.5 | **99.7** | 91.5 |
| `closed_33` | 10 | 99.3 | 84.9 | 98.4 | 93.5 | **99.9** | 98.3 |
| `closed_33` | 20 | 99.9 | 99.9 | 99.3 | 98.3 | 99.9 | 99.3 |

**Best per algorithm** (Bayesian-Optimized via Optuna TPE, 30 trials each over 3 maps × 2 swarm sizes × 2 seeds = 12-cell eval grid; composite-score weights from `score.py`):

| Algorithm | Score | Best config |
|---|---|---|
| **VoronoiPartition** | **+0.6711** | attract=1.49, damp=0.995, repel_g=0.87, repel_r=5.16, wall_g=0.63 |
| **STC**              | +0.6433 | attract=2.49, damp=0.66, repel_g=4.10, repel_r=2.04, wall_g=1.93 |
| **GridDecomposition** | +0.6365 | block=5, attract=3.03, damp=0.085, repel_g=3.71 |
| **Boustrophedon** | +0.5566 | lane=3.45, attract=1.28, damp=0.71, repel_g=1.11, repel_r=4.38 |
| **Spiral** | +0.5292 | pitch=2.54, attract=2.43, damp=0.096, repel_g=7.95 |
| **Spiral** | pitch=4.0, attract=1.0, repel_g=10.0 | +0.065 | 60.5% |

**Headline findings:**
- **Random Gaussian is surprisingly competitive** — wins outright at most n=2 cells across all maps. The 33×33 maps are small enough that running to ALL_DEPLETED (~700 s) lets random walks cover most cells eventually. Random's "weakness" (no plan) is offset by the long sim horizon; what penalizes Random in the grid-search composite score is high overlap and energy use, not coverage.
- **VoronoiPartition is the strongest classical algorithm** — essentially the static counterpart of `ConsensusController` (Track 3). The dynamic re-election in Consensus only marginally helps over freezing the partition at *t=0*.
- **Plan-based algorithms (Boustrophedon, Spiral) struggle on dense maps** — precomputed paths assume a workable map shape; obstacles drive many waypoints inside walls or behind walls the drone has no path-planner to navigate around. The waypoint follower above (skip-covered + stuck-skip) prevents permanent idling, but each unreachable waypoint still costs ≥ 5 s of stalled time before being dropped (and `stuck_skip_n` more waypoints with it), so the worst-case overhead is `5 s × (n_unreachable / stuck_skip_n)`.
- **GridDecomposition and STC sit in the middle** — Voronoi-flavored partition + structured traversal beats lawnmower/spiral but lags behind nearest-uncovered-in-region.

### Track 2 — Metaheuristic

All five population-/sample-based optimizers are implemented as **runtime controllers** matching the same `policy_fn(env) -> (n_drones, 3)` contract as Track 3. Each was tuned via **Bayesian Optimization** (`tools/bo_search.py`, Optuna TPE, 30 trials each) — replacing the original 27-config grid search (`tools/grid_search_{pso,ga,aco,sa,gwo}.py`, still in the repo). BO winners written to `outputs/bo/<algo>.json`; applied to default configs via `tools/bo_apply.py --apply`. Track 2's BO-refreshed sweep + leaderboards now live in `outputs_metaheuristic/` (alongside the pre-BO baseline at `sweep_track2_results_pre_bo.csv` for comparison).

Common framing: "the candidate solution" is a per-drone target cell (re-elected each step or evolved by the algorithm's update rule). Movement uses the same attract + drone/wall-repel + yaw-track-velocity stack as PF/Consensus.

**Mathematical formulation.** Every Track 2 controller uses the **same motion law as Track 1** (attract + damp + drone-repel + wall-repel, magnitude-clipped to `max_accel`), so the only thing changing between algorithms is **how each drone's target cell `tᵢ` is selected/updated**. The fitness function for the population-based algorithms is the count of uncovered cells within `fitness_radius` of a candidate target:

```
fitness(t) = |{c ∈ uncov : ‖c − t‖ ≤ fitness_radius}|
```

| Algorithm | Per-drone update rule for `tᵢ` |
|---|---|
| **PSO** | Standard particle-swarm velocity update used as the **acceleration command** (not target). Each step:<br>`pbestᵢ = argmin_{c ∈ uncov} ‖c − pᵢ‖`, &nbsp; `gbest = argmin_{c ∈ uncov} ‖c − p̄‖` (swarm centroid p̄).<br>`aᵢ = w·vᵢ + c₁·r₁·(pbestᵢ − pᵢ) + c₂·r₂·(gbest − pᵢ) − K_d·vᵢ` &nbsp;+ wall repel<br>where `w = inertia`, `c₁ = cognitive`, `c₂ = social`, `r₁, r₂ ~ U(0, 1)²`. **Note:** with the BO-tuned `attract_damp_gain ≈ w`, the inertia term cancels exactly, so PSO degenerates to "pull toward weighted combo of pbest and gbest" — closer to greedy nearest-uncov than canonical PSO. |
| **GA** | Each drone *i* persistently holds `tᵢ` between steps. On arrival at `tᵢ` (or when `tᵢ` becomes covered):<br>**1. Stale cleanup**: if `tᵢ ∉ uncov`, set `tᵢ ← argmin_{c ∈ uncov} ‖c − pᵢ‖`.<br>**2. Fitness eval** + **elite selection**: top `⌈elite_fraction · n⌉` drones (by `fitness(tᵢ)`) keep their targets.<br>**3. Mutation** (prob `p_mutation`): `tᵢ ← uniform random uncov cell within mutation_jump_radius`.<br>**4. Crossover** (prob `p_crossover`): pick random elite, perturb its target by `N(0, crossover_perturb_radius/2)`, snap to nearest uncov. |
| **ACO** | Pheromone field `τ(y, x)` updated each step: `τ ← (1 − ρ)·τ + Q·𝟙[uncov]`.<br>On arrival (target commitment), drone *i* samples a new target from candidates within `target_search_radius`:<br>`P(c) ∝ τ(c)^α · η(c)^β`, &nbsp; `η(c) = 1/max(‖c − pᵢ‖, 0.5)`<br>where `α = pheromone_weight`, `β = heuristic_weight`, `ρ = evaporation_rate`, `Q = deposit_amount`. |
| **SA** | Per-drone persistent target with annealing temperature `T`. Each step: `T ← max(T_min, cooling_rate·T)`.<br>On arrival, propose `t' = tᵢ + N(0, perturb_radius)`, snap to nearest uncov. Compute `ΔE = fitness(tᵢ) − fitness(t')` (we maximize fitness, so `ΔE = −Δfitness`).<br>**Metropolis accept:** if `ΔE ≤ 0`, accept; else accept with probability `exp(−ΔE / T)`. |
| **GWO** | Each step: rank all drones by `‖pᵢ − nearest_uncovᵢ‖` ascending. Top 3 = α, β, δ. Their nearest-uncov cells become the three "leader" positions `xₐ, x_b, x_d`.<br>For each drone *i*: three pulls toward leaders with random coefficients `A = 2a·r₁ − a`, `C = 2r₂` (where `a` decays linearly from `a_initial` to `a_final` over `decay_steps`):<br>`X_L = x_L − A·|C·x_L − pᵢ|` for `L ∈ {α, β, δ}`<br>`aᵢ = ⅓(X_α + X_β + X_δ) − pᵢ − K_d·vᵢ` &nbsp;+ wall repel. |

**Final benchmark — coverage % by (map × n_drones), `tools/sweep_track2.py`, mean over 3 seeds (Bayesian-Optimized configs):**

| Map | n | Random | PSO | GA | ACO | SA | GWO |
|---|---|---|---|---|---|---|---|
| `open_33` | 2 | 95.9 | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 5 | 99.8 | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 10 | **100** | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 20 | **100** | **100** | **100** | **100** | **100** | **100** |
| `partial_33` | 2 | 92.5 | 68.4 | **100** | **100** | 99.9 | 90.0 |
| `partial_33` | 5 | 99.7 | **100** | **100** | **100** | **100** | 95.1 |
| `partial_33` | 10 | **100** | **100** | **100** | **100** | **100** | 97.8 |
| `partial_33` | 20 | **100** | **100** | **100** | **100** | **100** | **100** |
| `closed_33` | 2 | 75.6 | 41.1 | 85.9 | 68.7 | **90.0** | 26.2 |
| `closed_33` | 5 | 95.5 | 86.5 | 82.1 | **97.1** | 86.9 | 55.7 |
| `closed_33` | 10 | 99.3 | 81.6 | 98.4 | 97.6 | **99.6** | 31.7 |
| `closed_33` | 20 | 99.9 | 98.4 | 99.4 | **100** | 99.9 | 64.6 |

**Best per algorithm** (Bayesian-Optimized via Optuna TPE, 30 trials over the 12-cell eval grid):

| Algorithm | Score | Best config |
|---|---|---|
| **SA**  | **+0.6453** | T₀=3.31, cool=0.962, σ=1.23, attract=3.36, damp=0.35 |
| **GA**  | +0.6453 | elite=0.40, p_cx=0.151, p_mut=0.351, attract=3.16, damp=1.00 |
| **ACO** | +0.6142 | α=0.93, β=3.62, ρ=0.0165, search_r=6.30, attract=3.65 |
| **PSO** | +0.6033 | inertia=0.077, c1=3.22, c2=0.014, damp=0.69 |
| **GWO** | +0.2475 | a₀=2.88, a_f=0.68, decay=206 |

**Cross-track findings:**
- **ACO and GA beat MARL at n=2** on open/partial maps — `ACO partial_33 n=2 = 99.5 %` vs `MARL = 90.4 %`. The simpler greedy-target approach with re-election each step actually outperforms a learned policy when each drone has lots of territory.
- **All Track 2 algorithms have a strong "information sharing hurts" pattern** — PSO's social pull, GA's crossover, GWO's leader-following all benefit from being turned *down* (or off entirely, like PSO `c2=0`). Drones doing their own thing wins on coverage problems.
- **SA and GWO are the weakest** — Metropolis "sometimes accept worse" and GWO's leader-flock dynamic produce wandering / herding that hurts spread.

#### Per-map breakdown (final coverage %, mean over 3 seeds)

##### `open_33` — 88.2 % open, 24,025 m² free

![open_33](docs/images/map_open_33.png)

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| **PSO** | 96.0 % | 97.2 % | 100.0 % | 100.0 % |
| **GA** | **100.0 %** | **100.0 %** | 100.0 % | 100.0 % |
| **ACO** | 98.9 % | **100.0 %** | 100.0 % | 100.0 % |
| **SA** | **100.0 %** | **100.0 %** | 100.0 % | 100.0 % |
| **GWO** | **100.0 %** | **100.0 %** | 100.0 % | 100.0 % |

##### `partial_33` — 73.0 % open, 19,875 m² free

![partial_33](docs/images/map_partial_33.png)

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| **PSO** | 94.4 % | 99.2 % | 100.0 % | 100.0 % |
| **GA** | 87.0 % | 98.4 % | 100.0 % | 100.0 % |
| **ACO** | **99.5 %** | 99.2 % | 98.1 % | 100.0 % |
| **SA** | 63.0 % | 85.4 % | 100.0 % | 100.0 % |
| **GWO** | 88.0 % | 73.6 % | 99.3 % | 99.8 % |

##### `closed_33` — 53.1 % open, 14,450 m² free (densest map)

![closed_33](docs/images/map_closed_33.png)

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| **PSO** | 51.5 % | 75.8 % | 90.2 % | 99.4 % |
| **GA** | 66.6 % | **94.9 %** | **99.2 %** | **100.0 %** |
| **ACO** | **75.8 %** | 88.5 % | 96.2 % | **100.0 %** |
| **SA** | 35.5 % | 60.8 % | 96.2 % | **100.0 %** |
| **GWO** | 35.9 % | 75.5 % | 47.6 % | 83.4 % |

#### Particle Swarm Optimization (PSO)

**Algorithm:** population-based stochastic search (Kennedy & Eberhart, 1995). Each "particle" is a candidate solution moving through the search space; its velocity is pulled by two attractors — its own personal-best position (cognitive term) and the swarm's global-best position (social term). Particles share information implicitly through the global best, so good regions get exploited collectively.

**Update rule (per iteration):**

```
v_{t+1} = w·v_t  +  c1·r1·(p_best − x_t)  +  c2·r2·(g_best − x_t)
x_{t+1} = x_t + v_{t+1}
```

with `r1, r2 ~ U(0, 1)` per dimension.

**Hyperparameters (canonical defaults):**

| Param | Typical value | Meaning |
|---|---|---|
| swarm size *N* | 20–50 | candidate solutions kept in parallel |
| inertia weight *w* | 0.7 | momentum on previous velocity |
| cognitive coefficient *c1* | 1.4 | weight on personal-best attraction |
| social coefficient *c2* | 1.4 | weight on global-best attraction |
| max iterations | problem-dependent | stop criterion |

#### Genetic Algorithm (GA)

**Algorithm:** evolutionary search (Holland, 1975; Goldberg, 1989). A population of "chromosomes" (encoded candidate solutions) evolves over generations through three operators: **selection** (favor higher-fitness chromosomes for reproduction), **crossover** (mix two parents to produce children), and **mutation** (random small perturbations that preserve diversity).

For coverage, a chromosome encodes either a per-drone waypoint sequence or a per-drone cell-assignment vector; fitness is the energy-aware objective evaluated by simulating the swarm in `CoverageEnv`.

**Hyperparameters (canonical defaults):**

| Param | Typical value | Meaning |
|---|---|---|
| population size | 50–200 | chromosomes per generation |
| crossover rate *p_c* | 0.6–0.9 | fraction of pairs that recombine |
| mutation rate *p_m* | 0.01–0.1 | per-gene perturbation probability |
| selection method | tournament (k=3) or roulette | how parents are chosen |
| elitism | top 1–5 % | best chromosomes survive untouched |
| generations | problem-dependent | stop criterion |

#### Ant Colony Optimization (ACO)

**Algorithm:** swarm intelligence inspired by ant pheromone trails (Dorigo, 1992; Dorigo & Stützle, 2004). Ants probabilistically construct solutions by walking the search graph; each move is biased by **pheromone strength** (deposited by past good solutions) and a **heuristic value** (problem-specific, e.g. distance to nearest uncovered cell). Pheromones evaporate over time so the colony can forget bad early choices.

**Probability of ant moving from i → j:**

```
P(i → j) = (τ_ij^α · η_ij^β) / Σ_k (τ_ik^α · η_ik^β)
```

where `τ` is pheromone strength and `η` is the heuristic value.

For coverage, ants walk grid cells (or waypoints); pheromones reinforce paths that yield high coverage with low overlap and energy.

**Hyperparameters (canonical defaults):**

| Param | Typical value | Meaning |
|---|---|---|
| number of ants *m* | 10–50 | parallel solution-builders per iteration |
| pheromone weight *α* | 1.0 | exploits past learning |
| heuristic weight *β* | 2–5 | exploits problem knowledge |
| evaporation rate *ρ* | 0.1–0.5 | per-iteration pheromone decay |
| pheromone deposit *Q* | 1.0 (often normalized) | scale of new pheromone laid by good solutions |
| max iterations | problem-dependent | stop criterion |

#### Simulated Annealing (SA)

**Algorithm:** single-solution stochastic search inspired by metallurgical annealing (Kirkpatrick, Gelatt, Vecchi, 1983). Starts from a candidate solution and at each step proposes a small perturbation. Better solutions are always accepted; **worse** solutions are accepted with probability `exp(−ΔE / T)` (the Metropolis criterion). The "temperature" *T* starts high (lots of exploration, escape from local optima) and decays over iterations until the search effectively becomes greedy.

**Hyperparameters (canonical defaults):**

| Param | Typical value | Meaning |
|---|---|---|
| initial temperature *T₀* | calibrated so initial accept rate ≈ 0.8 | controls early exploration |
| cooling schedule | geometric: `T_{k+1} = α·T_k`, α ≈ 0.95 | how T decays |
| final temperature *T_min* | 1e-3 · *T₀* | stop criterion |
| neighborhood operator | swap waypoints / perturb one drone's plan | defines local moves |
| iterations per temperature | 50–500 | how long to dwell at each *T* |

#### Grey Wolf Optimizer (GWO)

**Algorithm:** swarm intelligence based on the social hierarchy of grey wolf packs (Mirjalili, Mirjalili & Lewis, 2014). Wolves are ranked by fitness: the best three are α (alpha), β (beta), δ (delta); the rest are ω (omega). Each iteration, every wolf updates its position toward a weighted mean of α/β/δ — the leaders steer the pack. A control parameter *a* shrinks linearly from 2 → 0, transitioning from exploration (wolves can step *past* the leaders) to exploitation (they converge onto them).

**Hyperparameters (canonical defaults):**

| Param | Typical value | Meaning |
|---|---|---|
| pack size *N* | 20–40 | candidate solutions tracked |
| max iterations *T* | problem-dependent | stop criterion |
| control parameter *a* | linear 2 → 0 over *T* iterations | exploration/exploitation balance |
| coefficients *A*, *C* | derived from *a* and U(0,1) noise | per-iteration step magnitudes |

(GWO has notably few user-tunable knobs vs. PSO/GA — most behavior follows from the leader hierarchy and the *a* schedule.)

### Track 3 — Learning / control-based

1. **Potential Fields** — drones attracted to uncovered cells, repelled by obstacles, boundaries, and other drones. Simplest baseline; fast to implement and visualize. *Optimizes for:*
   - coverage progress (each drone heads to nearest uncovered cell)
   - inter-drone spacing (pairwise repel, 1/r²)
   - wall clearance (wall repel, 1/r²)
2. **Consensus-Based Coordination** — adds inter-drone rules / shared communication for spacing, formation, or task distribution. More coordinated than pure local reaction, but no learning. *Optimizes for:*
   - territorial coverage (each drone covers its own Voronoi region of uncovered cells)
   - inter-drone spacing (same repel as PF)
   - wall clearance (same repel as PF)
3. **Multi-Agent Reinforcement Learning** — drones learn policies through rewards that encode the energy-aware objective. Most powerful and most fragile (training cost, reward tuning, debugging). *PPO maximizes the discounted sum of per-step rewards over the episode*, where each step's reward = `(newly covered cells)  −  (small time penalty)  +  (completion bonus if 100% coverage reached)  −  (depletion penalty if all drones die first)  +  (optional shaping for overlap area, wasted visits, energy used)`.

**Mathematical formulation.** PF and Consensus share the same motion law as Tracks 1–2 (attract + damp + drone-repel + wall-repel, clipped at `max_accel`); MARL is fundamentally different — a learned neural-network policy.

| Policy | Update rule |
|---|---|
| **Potential Fields** | At every step, each drone *i*'s target is the **globally** nearest uncovered cell: `tᵢ = argmin_{c ∈ uncov} ‖pᵢ − c‖`. Same motion law as classical:<br>`aᵢ = K_a · ûᵢ→tᵢ − K_d · vᵢ + drone_repel + wall_repel`<br>Repulsions sum over *all* other drones / walls within their respective ranges (no neighbor visibility limit — fully centralized in that sense). |
| **Consensus** | Each step, each drone *i* polls the visibility-restricted neighbor set `Nᵢ = {j : ‖pᵢ − pⱼ‖ ≤ comm_range}` and computes a **local Voronoi partition** over uncovered cells against `{i} ∪ Nᵢ`:<br>`ownerᵢ(c) = (i == argmin_{j ∈ {i}∪Nᵢ} ‖pⱼ − c‖)`<br>The drone's owned cells `Mᵢ = {c ∈ uncov : ownerᵢ(c) = i}` define its attractor:<br>`tᵢ = mean(Mᵢ)` (centroid, Lloyd-style) &nbsp;or&nbsp; `argmin_{c ∈ Mᵢ} ‖pᵢ − c‖` (nearest; default).<br>Then the same motion law fires. If `Mᵢ = ∅`, fall back to global nearest uncov. With `comm_range = None` the partition becomes global Voronoi — Lloyd centralized — which is exactly Track 1's `VoronoiPartition` but **re-elected every step** using current positions instead of frozen at *t=0*. |
| **MARL (PPO)** | A neural-network policy `πθ(a \| o)` decentralized per drone (each drone runs the same shared-weight policy on its own observation). Observation `oᵢ` per drone = `[pᵢ, vᵢ, θᵢ, ωᵢ, battery_fracᵢ, local_coverage_patch(pᵢ)]` (see `controllers/marl_env.py:LOCAL_DIM`). Action `aᵢ = (axᵢ, ayᵢ)` scaled to `[−max_accel, +max_accel]`; yaw is handled by the same velocity-tracking PD as the other Track 3 methods (not learned).<br><br>**Per-step reward** (sum across all drones):<br>`rₜ = α_cov · Δcoverageₜ − α_time + α_done · 𝟙[done] − α_dep · 𝟙[all_depleted] − α_ov · Δoverlapₜ − α_wast · Δwastedₜ − α_eng · Δenergyₜ`<br><br>**PPO clipped surrogate objective** (Schulman et al. 2017):<br>`L^CLIP(θ) = 𝔼ₜ[min(ρₜ(θ)·Âₜ, clip(ρₜ(θ), 1 − ε, 1 + ε)·Âₜ)]`<br>where `ρₜ(θ) = πθ(aₜ\|oₜ) / π_{θold}(aₜ\|oₜ)` is the importance ratio and `Âₜ` is the generalized-advantage estimate. Trained per swarm size (`n ∈ {2, 5, 10, 20}`) — see `tools/train_marl.py` and `tools/random_search_marl.py` for the hyperparameter search. |

**Final benchmark** — `tools/sweep.py` over the 3 maps (see [Test maps](#test-maps) above) × `n_drones ∈ {2, 5, 10, 20}` × 3 seeds = 144 runs. PF and Consensus configs are BO-tuned via `tools/bo_search.py` (Optuna TPE, 30 trials each over the 12-cell eval grid); MARL uses the random-best PPO config (`tools/random_search_marl.py`) trained at 150 k steps per swarm size with reward shaping.

Mean final coverage by swarm size (averaged over 3 maps × 3 seeds = 9 runs per cell):

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 88.0 % | 98.4 % | 99.8 % | **100.0** % |
| **Potential Fields** | 82.2 % | 91.9 % | 97.5 % | 99.9 % |
| **Consensus**     | 64.3 % | 83.9 % | 93.9 % | 97.6 % |
| **MARL (PPO)**    | 85.9 % | 96.1 % | 96.6 % | 99.9 % |

Key takeaways:
- **PF reclaimed n=2.** With BO-tuned `attract_damp_gain = 0.12` (much lighter than the 0.2 default) and tighter repel ranges, PF at n=2 jumped from 46 % → 82 %.
- **MARL is still strongest on `closed_33 n=2`** (67.8 %) — it's the only learning-based policy and the only one where the network *generalises* the wall-avoidance strategy rather than computing it via attract-vs-repel cancellation.
- **Consensus dropped** — the BO-tuned dense Voronoi attract is heavier than the prior `dense()` preset (`attract_gain=4.79, damp=0.68`), which hurts at sparse n where the drone barely has any Voronoi neighbors.

#### Per-map breakdown (final coverage %, mean over 3 seeds)

##### `open_33` — 88.2 % open, 24,025 m² free

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 95.9 % | 99.8 % | 100.0 % | 100.0 % |
| **Potential Fields** | **100.0 %** | **100.0 %** | **100.0 %** | **100.0 %** |
| **Consensus**     | **100.0 %** | **100.0 %** | **100.0 %** | **100.0 %** |
| **MARL (PPO)**    | 95.4 % | 99.3 % | 99.5 % | 100.0 % |

##### `partial_33` — 73.0 % open, 19,875 m² free

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 92.5 % | 99.7 % | 100.0 % | 100.0 % |
| **Potential Fields** | 95.8 % | **100.0 %** | **100.0 %** | **100.0 %** |
| **Consensus**     | 72.9 % | **100.0 %** | **100.0 %** | **100.0 %** |
| **MARL (PPO)**    | 94.6 % | 97.6 % | 98.5 % | 100.0 % |

##### `closed_33` — 53.1 % open, 14,450 m² free (densest map)

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 75.6 % | **95.5 %** | **99.3 %** | 99.9 % |
| **Potential Fields** | 50.8 % | 75.8 % | 92.4 % | 99.7 % |
| **Consensus**     | 19.9 % | 51.6 % | 81.8 % | 92.8 % |
| **MARL (PPO)**    | 67.8 % | 91.3 % | 91.9 % | **99.7 %** |

---

## Cross-track comparison

All four Track 3 policies, the five Track 2 metaheuristics, and Random — evaluated on the same (map × n_drones × seed) grid with their **BO-tuned** defaults (`tools/bo_search.py`, Optuna TPE, 30 trials each). **Bold** marks the per-cell winner. Per-track sweep CSVs live in:

- `outputs_classical/sweep_track1_results.csv` — Track 1 only (Boustrophedon / Spiral / VoronoiPartition / GridDecomposition / STC + Random)
- `outputs_metaheuristic/sweep_track2_results.csv` — Track 2 only (PSO / GA / ACO / SA / GWO + Random)
- `outputs_control_based/sweep_track3_results.csv` — Track 3 only (PF / Consensus / MARL + Random)
- `outputs/sweep_results.csv` — **all-policy cross-track sweep** (the source of the table below)

Multi-metric leaderboards (coverage, time-to-100 %, energy, overlap, wasted visits) are in `outputs/leaderboards/leaderboards.md` plus per-track PNGs at `outputs_{classical,metaheuristic,control_based}/leaderboard.png`.

| Map | n | Random | PF | Consensus | MARL | ‖ | PSO | GA | ACO | SA | GWO |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `open_33` | 2 | 95.9 | **100** | **100** | 95.4 | ‖ | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 5 | 99.8 | **100** | **100** | 99.3 | ‖ | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 10 | **100** | **100** | **100** | 99.5 | ‖ | **100** | **100** | **100** | **100** | **100** |
| `open_33` | 20 | **100** | **100** | **100** | **100** | ‖ | **100** | **100** | **100** | **100** | **100** |
| `partial_33` | 2 | 92.5 | 95.8 | 72.9 | 94.6 | ‖ | 68.4 | **100** | **100** | 99.9 | 90.0 |
| `partial_33` | 5 | 99.7 | **100** | **100** | 97.6 | ‖ | **100** | **100** | **100** | **100** | 95.1 |
| `partial_33` | 10 | **100** | **100** | **100** | 98.5 | ‖ | **100** | **100** | **100** | **100** | 97.8 |
| `partial_33` | 20 | **100** | **100** | **100** | **100** | ‖ | **100** | **100** | **100** | **100** | **100** |
| `closed_33` | 2 | 75.6 | 50.8 | 19.9 | 67.8 | ‖ | 41.1 | 85.9 | 68.7 | **90.0** | 26.2 |
| `closed_33` | 5 | 95.5 | 75.8 | 51.6 | 91.3 | ‖ | 86.5 | 82.1 | **97.1** | 86.9 | 55.7 |
| `closed_33` | 10 | 99.3 | 92.4 | 81.8 | 91.9 | ‖ | 81.6 | 98.4 | 97.6 | **99.6** | 31.7 |
| `closed_33` | 20 | 99.9 | 99.7 | 92.8 | 99.7 | ‖ | 98.4 | 99.4 | **100** | 99.9 | 64.6 |

**Headline mean coverage** (across all 36 cells = 3 maps × 4 swarm sizes × 3 seeds):

| Rank | Policy | Mean cov | Track | Was |
|---|---|---|---|---|
| 1 | **SA** | **98.0 %** | Track 2 | ~87 % |
| 2 | GA | 97.1 % | Track 2 | ~96 % |
| 3 | ACO | 96.9 % | Track 2 | 96.0 % |
| 4 | Random | 96.5 % | baseline | — |
| 4 | STC | 96.5 % | Track 1 | 84.3 % (**+12.2**) |
| 4 | VoronoiPartition | 96.5 % | Track 1 | 91.2 % |
| 7 | Spiral | 95.2 % | Track 1 | 60.5 % (**+34.7**) |
| 8 | MARL | 94.6 % | Track 3 | — |
| 8 | GridDecomposition | 94.6 % | Track 1 | 85.2 % |
| 10 | PF | 92.9 % | Track 3 | — |
| 11 | PSO | 87.6 % | Track 2 | 92.9 % (−5.3) |
| 12 | Boustrophedon | 86.2 % | Track 1 | 69.5 % (**+16.7**) |
| 13 | Consensus | 84.9 % | Track 3 | — |
| 14 | GWO | 80.1 % | Track 2 | ~81 % |

**Headline cross-track findings (post-BO retune):**

- **SA is the new overall winner** at 98.0 % mean coverage — up from ~87 % under the old grid-search defaults. BO chose `T_initial = 3.31` (much higher → more exploration early), `perturb_radius = 1.23` (much smaller → tight local moves), and an attract chain that doesn't fight repulsion. The Metropolis "occasionally accept worse" rule is *exactly the right thing* once perturbation is small enough that the worse moves don't waste much travel.
- **STC matches VoronoiPartition** at 96.5 % — both Track 1 partition-based methods now equal Random's mean coverage. The classical partition is now a free Pareto improvement over Random: same coverage at less energy.
- **Spiral's lift is the most dramatic** (60.5 % → 95.2 %, **+34.7 pts**). The combo of skip-covered + stuck-detector + BO-tuned damping `0.096` (very light, lets velocity carry through curves) makes spiral coverage actually work.
- **PSO regressed −5 pts.** BO drove `inertia` to 0.077 (near zero), neutering PSO's exploration term — it now behaves like greedy-nearest-uncov with random noise, which is worse than ACO/GA's principled target-evolution on `closed_33`.
- **Consensus is the weakest control-based method** (84.9 %). BO's attract-heavy config (4.79) plus dynamic Voronoi re-election causes drones to flock to the same region centroid early, then have to redistribute. Static `VoronoiPartition` (96.5 %) beats it cleanly — the dynamic re-election cost outweighs the benefit.
- **GWO is unsalvageable** at 80.1 %. The 3-leader pull formulation hits a ceiling on coverage problems; BO couldn't find a config above the old +0.251.

**Practical takeaway (post-BO):** **SA** is the new "if I had to pick one" answer — best coverage at lowest energy, robust across all (map × n_drones) cells. **VoronoiPartition / STC** are the simplest performant classical baselines (no internal state, no stochastic operators). **MARL** is still the only learning-based method and wins specifically at `closed_33 n=2` (67.8 %, highest of the Track 3 set at the hardest cell). **PSO / GWO / Consensus** can be skipped in further work — they don't beat simpler alternatives in their own niche after BO retune.