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

---

## Results

### Track 1 — Classical / geometry-based

_To be added: chosen method(s), benchmark conditions, scores on the energy-aware objective, link to writeup._

### Track 2 — Metaheuristic

Five candidate population-/sample-based optimizers under evaluation. _Not yet implemented in this repo — descriptions below set the conventions for the eventual port._ When wired up, each will share the same `policy_fn(env) -> (n_drones, 3)` contract as Track 3 so they're benchmarked under identical conditions on the same `(map, seed)` pairs via `tools/benchmark.py` and `tools/sweep.py`.

Common framing: "the candidate solution" is a per-drone trajectory plan (sequence of waypoints / cell allocations), evaluated by simulating the swarm in `CoverageEnv` and scoring the resulting run against the energy-aware objective (coverage, time, energy, overlap, wasted visits — the exact components benchmarked for Track 3).

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

**Final benchmark** — `tools/sweep.py` over the 3 maps (see [Test maps](#test-maps) above) × `n_drones ∈ {2, 5, 10, 20}` × 3 seeds = 144 runs. PF and Consensus use the `dense()` preset (random-search winner across all 7 knobs at n=5 `partial_33`, robustness confirmed by a refined random search). MARL uses the random-best PPO config (also from `tools/random_search_marl.py`) trained at 150k steps per swarm size with reward shaping.

Mean final coverage by swarm size (averaged over 3 maps × 3 seeds = 9 runs per cell):

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 83.6 % | 97.5 % | 99.8 % | 99.9 % |
| **Potential Fields** | 46.1 % | 84.9 % | **99.3 %** | **100.0 %** |
| **Consensus**     | 48.5 % | 83.3 % | **99.7 %** | **100.0 %** |
| **MARL (PPO)**    | **85.8 %** | **94.5 %** | 98.3 % | **100.0 %** |

Key takeaways:
- **MARL dominates at sparse swarms (n = 2)** — `PFConfig.dense()` / `ConsensusConfig.dense()` use a wide repel range tuned for crowded swarms, which hurts when drones rarely meet (the trade-off documented in the dense-preset docstring).
- **All four policies converge to ≥ 99 % by n = 20** — at saturating swarm densities, the choice of policy matters less than the swarm size.

#### Per-map breakdown (final coverage %, mean over 3 seeds)

##### `open_33` — 88.2 % open, 24,025 m² free

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 94.8 % | 99.8 % | 100.0 % | 100.0 % |
| **Potential Fields** | 11.8 % | 57.4 % | 100.0 % | 100.0 % |
| **Consensus**     | 11.8 % | 58.5 % | 100.0 % | 100.0 % |
| **MARL (PPO)**    | **93.4 %** | **99.0 %** | 99.3 % | 100.0 % |

##### `partial_33` — 73.0 % open, 19,875 m² free

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 82.4 % | 98.2 % | 100.0 % | 100.0 % |
| **Potential Fields** | 78.9 % | **99.9 %** | 98.0 % | 100.0 % |
| **Consensus**     | 86.2 % | 98.8 % | 100.0 % | 100.0 % |
| **MARL (PPO)**    | **90.4 %** | 97.1 % | 98.2 % | 100.0 % |

##### `closed_33` — 53.1 % open, 14,450 m² free (densest map)

| Policy | n = 2 | n = 5 | n = 10 | n = 20 |
|---|---|---|---|---|
| Random Gaussian   | 73.7 % | 94.6 % | 99.3 % | 99.9 % |
| **Potential Fields** | 47.5 % | **97.3 %** | **99.8 %** | 100.0 % |
| **Consensus**     | 47.5 % | 92.6 % | 99.0 % | 100.0 % |
| **MARL (PPO)**    | **73.6 %** | 87.5 % | 97.3 % | 100.0 % |