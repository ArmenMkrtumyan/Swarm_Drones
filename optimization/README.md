# Swarm Drone Optimization: 2D Coverage Platform

A fast, NumPy + matplotlib platform for evaluating swarm-coverage control methods before porting them to NVIDIA Isaac Sim. Continuous-action point-mass drones with speed/acceleration caps that mirror real quadcopter limits.

> Why 2D first? It's a cheap **falsification** tool — a method that fails here almost certainly fails in Isaac, so dead ends get caught quickly.

---

## Project drone

The simulator is calibrated to the actual hardware this project flies — the **[Hawk's Work F450](https://www.hawks-work.com/pages/f450-drone)** build:

| Component | Spec |
|---|---|
| Frame | Hawk's Work F450, 450 mm wheelbase, 280 g (clone of DJI Flame Wheel F450 — same airframe class) |
| Motors | 4× A2212 920 KV brushless |
| ESCs | 4× 20 A brushless with 5 V / 1 A BEC |
| Battery | 11.1 V 3S LiPo, 4200 mAh, 25 C, XT60 |
| Props | 9450 self-tightening (CW + CCW) |
| Flight controller | Pixhawk 2.4.8, PX4 autopilot |
| Camera | e-con Systems **STEEReoCAM Nano**, **forward-facing fixed-mount** (no gimbal — drone yaws to redirect). 2× OV2311 stereo, 4.3 mm f/2.8 M12 lens, HFOV 54° / VFOV 49.5°, 0.95–8 m stereo depth range, on-board 6-axis IMU. |
| Companion | NVIDIA Jetson Nano |
| Total mass | ~1.9 kg |

Full reference numbers (hover times, cruise / range estimates, calibration sources) live in [`docs/f450-reference.md`](docs/f450-reference.md).

---

## Documentation

Detailed docs live in [`docs/`](docs/) — each focused on one topic:

| Doc | What's in it |
|---|---|
| [`docs/setup.md`](docs/setup.md) | Install, system deps for `--gui` on Linux/WSL, demo run modes, all CLI flags, map editor, outputs layout. |
| [`docs/simulation-model.md`](docs/simulation-model.md) | Coordinate frame, world scale (5 m/cell), per-step flow, drone dynamics, wall collision, coverage and visit-count metrics, intentional gaps to Isaac. |
| [`docs/battery-model.md`](docs/battery-model.md) | Energy bookkeeping (`P = P_hover + k·v²`), unit conversions and equation sources, voltage cutoff, calibration, worked example. |
| [`docs/f450-reference.md`](docs/f450-reference.md) | Real-world F450 references: 3S/4S battery options, published hover times, inferred cruise endurance and range, max forward speed. |
| [`docs/verification.md`](docs/verification.md) | The three verification scripts (`test_overlap`, `test_flight_time`, `test_distance`) — what each asserts, how to run headless or `--gui`. |

---

## Problem

Multiple drones must cover a target region as effectively as possible while minimizing energy consumption. The objective is multi-criteria:

- maximize total area covered
- avoid covering the same area repeatedly
- minimize total energy usage
- avoid collisions with each other and with obstacles

The work has two halves: an **optimization** part that decides how drones move or split the area, and a **simulation** part that demonstrates and evaluates the behavior.

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
| **Potential Fields controller** | ⬜ next |
| Per-drone `comm_range` on `CoverageEnv` (limited-view neighbor observations) | ⬜ prerequisite for Consensus |
| **Consensus-Based Coordination controller** | ⬜ after PF |
| **Multi-Agent RL** | ⬜ stretch |
| Energy-aware objective implementation (multi-criteria scoring on top of coverage) | ⬜ pending |
| Isaac Sim port of the chosen method(s) | ⬜ pending |

---

## Results

Placeholder sections — fill in as each track produces benchmarks against the energy-aware objective.

### Track 1 — Classical / geometry-based (Teammate 1)

_To be added: chosen method(s), benchmark conditions, scores on the energy-aware objective, link to writeup._

### Track 2 — Metaheuristic (Teammate 2)

_To be added: chosen method(s), benchmark conditions, scores on the energy-aware objective, link to writeup._

### Track 3 — Learning / control-based (this repo)

#### Random baseline

- Map: recursive-backtracker, 21×21, 4 drones, 300 steps.
- Coverage: plateaus ~24 % under random Gaussian acceleration commands.
- Purpose: lower bound — any real controller must beat this.

#### Potential Fields

_Pending implementation._

#### Consensus-Based Coordination

_Pending implementation._

**Prerequisite — comm-range modeling.** Consensus depends on drones exchanging state with their neighbors. In real flight that's done by a mesh routing protocol on each drone's companion computer (e.g., [BATMAN-adv](https://www.open-mesh.org/projects/batman-adv/wiki/BATMAN_IV) on a Jetson Nano's Wi-Fi link). In this 2D testbed there's no network, so before implementing Consensus we need to add a `comm_range` parameter to `CoverageEnv` (likely on `DroneConfig`) so each drone's observation is restricted to peers within range. That *models* the mesh's effective behavior — who can hear whom — without running a real mesh stack, which only earns its keep when packets and antennas are physical.

#### Multi-Agent Reinforcement Learning

_Pending implementation._
