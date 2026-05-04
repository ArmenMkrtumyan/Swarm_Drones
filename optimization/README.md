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
| [`docs/battery-model.md`](docs/battery-model.md) | 3S/4S battery options, energy bookkeeping (`P = P_hover + k·v²`), mass-aware hover power, voltage cutoff, calibration. |
| [`docs/f450-reference.md`](docs/f450-reference.md) | Hawk's Work F450 hardware spec, camera geometry, hover / cruise / range direct calculations, community-data sanity checks. |
| [`docs/verification.md`](docs/verification.md) | The three verification scripts (`test_overlap`, `test_flight_time`, `test_distance`) — what each asserts, how to run headless or `--gui`. |

---

## Research tracks

Our group is exploring algorithm families in parallel, then converging on the most promising path:

| Track | Family | Owner | Status in this repo |
|---|---|---|---|
| 1 | Classical coverage and geometry-based methods (coverage path planning, grid decomposition, boustrophedon/lawnmower, Voronoi partitioning, task allocation) | Elen | Not synced yet |
| 2 | Metaheuristic optimization (PSO, Genetic Algorithms, Ant Colony, Simulated Annealing, Grey Wolf / Firefly / Bee) | Raffi | Not synced yet |
| **3** | **Learning and control-based methods (Potential Fields, Consensus-Based Coordination, Multi-Agent RL)** | **Armen** | **completed** |

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

## Implementation (Armen)

Simplest method first, riskiest last. So there are checkpoints as the project goes on.

1. **Potential Fields** — drones attracted to uncovered cells, repelled by obstacles, boundaries, and other drones. Simplest baseline; fast to implement and visualize.
2. **Consensus-Based Coordination** — adds inter-drone rules / shared communication for spacing, formation, or task distribution. More coordinated than pure local reaction, but no learning.
3. **Multi-Agent Reinforcement Learning** — drones learn policies through rewards that encode the energy-aware objective. Most powerful and most fragile (training cost, reward tuning, debugging).

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

---

## Implementation status

| Component | State |
|---|---|
| Map generators (`open_arena`, `random_obstacles`, `recursive_backtracker`) | ✅ done |
| Map validators (`is_fully_connected`, `find_components`, `disconnected_cells`, `boundary_breaches`) | ✅ done |
| Interactive map editor (`tools/editor.py`) | ✅ done |
| Environment (`CoverageEnv` — continuous actions, speed/accel caps, wall collision, coverage tracking) | ✅ done |
| Headless + GUI visualization (`render_frame`, `animate`) | ✅ done |
| Random-action baseline (`random_policy` in `tools/demo.py`) | ✅ done — plateaus ~25 % |
| Battery model (F450 11.1V 4200 mAh, hover + speed²) | ✅ done — exposed via `CoverageEnv.battery_state()` |
| Voltage cutoff / drone shutdown (`min_voltage_v`, linear `V(E)` discharge approximation) | ✅ done — depleted drones freeze, render gray, tagged `[DEAD]` |
| Visit-count overlap metrics (`entry_count`, `total_visits`, `self_revisits`, `cross_overlap_visits`, `wasted_visits_total`) | ✅ done — entry-event semantics, hovering doesn't inflate |
| Per-drone HSV palette + first-visitor map paint (`init_drone_palette`) | ✅ done — random rotation per run, deterministic seed for tests |
| Verification scripts (`test_overlap.py`, `test_flight_time.py`, `test_distance.py`) | ✅ done — headless asserts + `--gui` live observation |
| **Potential Fields controller** (`controllers/potential_fields.py`) | ✅ done — attract-to-uncovered + drone/wall repulsion + yaw-tracks-velocity |
| Per-drone `comm_range` on `DroneConfig` (limited-view neighbor observations via `env.neighbors(i)`) | ✅ done |
| **Consensus-Based Coordination controller** (`controllers/consensus.py`) | ✅ done — Voronoi-flavored, nearest-uncovered-in-region |
| **Multi-Agent RL** (`controllers/marl.py`, `tools/train_marl.py`) | ✅ done — shared-policy PPO via Stable-Baselines3, Gymnasium wrapper, trained checkpoint at `outputs/marl_ppo.zip` |
| Benchmark harness (`tools/benchmark.py`) | ✅ done — runs all 4 policies on a fixed seed×map grid, emits CSV + comparison plots |
| Energy-aware objective implementation (multi-criteria scoring on top of coverage) | ⬜ pending — current benchmark reports the components separately (coverage / time / energy / overlap / wasted visits); compose into a single score later |
| Isaac Sim port of the chosen method(s) | ⬜ pending |

---

## Results

Placeholder sections — fill in as each track produces benchmarks against the energy-aware objective.

### Track 1 — Classical / geometry-based (Teammate 1)

_To be added: chosen method(s), benchmark conditions, scores on the energy-aware objective, link to writeup._

### Track 2 — Metaheuristic (Teammate 2)

_To be added: chosen method(s), benchmark conditions, scores on the energy-aware objective, link to writeup._

### Track 3 — Learning / control-based (this repo)

All four policies share a common contract: `policy_fn(env) -> np.ndarray` of shape `(n_drones, 3)` = `[ax, ay, alpha_yaw]`. The benchmark harness (`tools/benchmark.py`) runs each on the same `(map, seed)` pairs and writes `benchmark_results.csv` plus four comparison PNGs into `outputs/images/`.

Headline results — 5 seeds × 2 map kinds (`random`, `maze`) × 4 drones, 15×15 grid, run-to-terminal:

| Policy | Final coverage | t→100 % (random) | t→100 % (maze) | Overlap area | Wasted visits |
|---|---|---|---|---|---|
| Random Gaussian | 96.5 % | 405 s | rarely reached | 2055 m² | 1619 |
| **Potential Fields** | 86.7 % | **83 s** | gets stuck ~75 % | **625 m²** | 1311 |
| **Consensus** | 87.8 % | **76 s** | gets stuck ~76 % | 660 m² | **1229** |
| **MARL (PPO)** | 93.0 % | 193 s | reaches ~90 % | 1625 m² | 1842 |

#### Random baseline

Pure Gaussian acceleration. Plateaus ~24 % on a 300-step budget but eventually reaches near-100 % when run to depletion (~405 s on random maps, ~700 s+ on mazes). Lower bound for the energy-aware metrics — any real controller should beat this on at least one axis.

#### Potential Fields  ([`controllers/potential_fields.py`](controllers/potential_fields.py))

Khatib-style superposition: each drone is attracted to its nearest uncovered cell, repelled from neighbor drones (1/r²) and walls. Yaw tracks velocity so the wedge sweeps the path. Stateless. **Strength**: fastest to 100 % on random maps. **Weakness**: gets stuck on maze topology because Euclidean attraction doesn't respect corridors.

#### Consensus-Based Coordination  ([`controllers/consensus.py`](controllers/consensus.py))

Voronoi-flavored coordination. Each drone polls neighbors within `DroneConfig.comm_range` (`env.neighbors(i)`); the communication group implicitly partitions uncovered cells by proximity, and each drone heads to the nearest uncovered cell *in its own region*. Repulsion identical to PF. **Strength**: cleanest territory partition (lower wasted visits and slightly faster to 80 % than PF). **Weakness**: same maze-topology stall as PF.

The **prerequisite — `DroneConfig.comm_range`** — is now a real field on `DroneConfig` (default `None` = global mesh; set to a finite radius to scope each drone's observations to local neighbors only). It models the routing horizon a real swarm has on its companion computer (e.g., [BATMAN-adv](https://www.open-mesh.org/projects/batman-adv/wiki/BATMAN_IV) on the Jetson Nano's Wi-Fi link) without running a real mesh stack.

#### Multi-Agent Reinforcement Learning  ([`controllers/marl.py`](controllers/marl.py), [`tools/train_marl.py`](tools/train_marl.py))

Independent PPO with shared parameters via Stable-Baselines3 + Gymnasium. The joint-policy network sees per-drone local state (position, velocity, heading, battery, 5×5 local coverage mask, global coverage fraction) flattened across the swarm; emits joint 2D acceleration. Yaw is set deterministically to track velocity (the policy doesn't have to learn it). Reward = `+coverage_delta_cells - 0.01 (time penalty) +100 (full coverage) -20·(1-cov) (depletion penalty)`. Training: 500k env steps × 4 parallel envs (~2.5 min on this Mac, no GPU). **Strength**: more robust on maze topology (~90 % vs PF/Consensus's ~75 %). **Weakness**: slower than PF/Consensus on simple random maps; still has variance across eval seeds.

#### Comparison plots

Generated by `tools/benchmark.py`:
- `benchmark_coverage_curves.png` — mean ± std coverage curves across all runs
- `benchmark_coverage_curves_{random,maze}.png` — split by map type
- `benchmark_summary_bars.png` — final coverage / t→80 % / energy / wasted-visits-per-%-coverage
- `benchmark_per_map.png` — final coverage by map type, all policies

Run with `python tools/benchmark.py --seeds 7 11 17 23 29 --maps both --grid 15 --drones 4` (after `python tools/train_marl.py` to populate `outputs/marl_ppo.zip`).
