# Swarm Drone Optimization: 2D Coverage Platform

A fast, NumPy + matplotlib platform for evaluating swarm-coverage control methods before porting them to NVIDIA Isaac Sim. Continuous-action point-mass drones with speed/acceleration caps that mirror real quadcopter limits.

> Why 2D first? It's a cheap **falsification** tool — a method that fails here almost certainly fails in Isaac, so dead ends get caught quickly.

---

## Problem

Multiple drones must cover a target region as effectively as possible while minimizing energy consumption. The objective is multi-criteria, not just coverage:

- maximize total area covered
- avoid covering the same area repeatedly
- minimize total energy usage
- avoid collisions with each other and with obstacles
- keep behavior realistic enough to demonstrate in NVIDIA Isaac Sim

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
- estimated energy consumption (see battery model below)
- overlap between drones
- collision count / proximity violations
- smoothness of movement
- time to complete coverage
- scalability as the number of drones grows

Keeping this objective **identical across 2D and Isaac** is the discipline that lets results transfer between environments. The energy term in particular is computed from the battery model described under [Simulation model → Battery](#battery).

---

## Implementation (Armen)

Simplest method first, riskiest last. So there are checkpoints as the project goes on.

1. **Potential Fields** — drones attracted to uncovered cells, repelled by obstacles, boundaries, and other drones. Simplest baseline; fast to implement and visualize.
2. **Consensus-Based Coordination** — adds inter-drone rules / shared communication for spacing, formation, or task distribution. More coordinated than pure local reaction, but no learning.
3. **Multi-Agent Reinforcement Learning** — drones learn policies through rewards that encode the energy-aware objective. Most powerful and most fragile (training cost, reward tuning, debugging).

---

## Simulation model

The 2D platform has just enough physics to expose the algorithmic question (does this controller cover ground efficiently?), no more.

### Coordinate frame and units

- The grid is an `int8` numpy array indexed `[y, x]`, shape `(H, W)`. Origin is top-left; `y` increases downward (matches numpy row-major and matplotlib's `origin="upper"`).
- Positions are floats in **cell units**. The cell at integer index `(cx, cy)` has its center at `(cx + 0.5, cy + 0.5)` and spans `[cx, cx+1) × [cy, cy+1)`.
- Time advances in fixed steps of `step_seconds = 0.1 s` by default (the simulation timestep, often written `dt` in physics formulas).

### World scale

`SimConfig.meters_per_cell` (default **5.0**) is the single knob that maps the cell-unit physics to the real world. The default 5 m/cell is derived from the F450's camera footprint at typical altitude (~14 m diameter ≈ 7 m radius), so:

| Param | In cells | Real units (at 5 m/cell) | Real-world reference |
|---|---|---|---|
| `sensor_radius` | 1.5 | 7.5 m | F450 camera at ~10 m altitude |
| `max_speed` (demo) | 1.5 | 7.5 m/s | realistic F450 cruise |
| `max_accel` (demo) | 2.5 | 12.5 m/s² ≈ 1.3 g | typical multirotor punch |
| `drone_radius` | 0.05 | 0.25 m | actual F450 half-width |
| 21×21 map | 21 | 105 × 105 m | small park / parking lot |

**Why this scale (and not "1 cell = 1 m")?** Coverage problems are defined by how much area the sensor can resolve per cell. Picking `meters_per_cell` from the sensor footprint keeps the per-cell discretization meaningful. Picking it from drone body size instead would make the map only 21×21 m and the drone slow at 1.35 m/s. Picking from max horizontal speed (15 m/s) gives a 210×210 m map with sensor implying ~20 m altitude. The middle option — sensor footprint — is what the default targets.

**Drone-radius caveat.** At the default scale, `drone_radius = 0.05` cells = 0.25 m matches a real F450 half-width and is small relative to the 5 m corridor, so collision logic essentially treats drones as point masses (correct for coverage missions; would need bumping for tight-tunnel scenarios).

**Mass.** `DroneConfig.mass_kg = 1.9` (F450 + Jetson Nano + camera + 3S LiPo) is **informational only** — the power model uses back-calculated `hover_power_w = 165 W` rather than deriving it from `m·g` and rotor area. Adding aerodynamic derivation would require rotor area and air density.

### Per-step flow

Each call to `env.step(actions)` advances the world by `step_seconds`. The ordering is fixed — controllers can rely on this contract:

1. **Receive actions.** The caller passes a NumPy array of shape `(n_drones, 2)` containing per-drone acceleration commands in cell-units / s² (any direction, any magnitude). Each drone's vector is rescaled so `|a| ≤ max_accel` while preserving direction. No clipping per-axis.

2. **For each drone, in index order:**

   a. **Integrate velocity.** `v_new = v_old + a · step_seconds`. If `|v_new| > max_speed`, rescale to the speed cap (direction preserved). [Explicit Euler integration][src-euler-wiki] — see also [Gaffer On Games' "Integration Basics"][src-euler-game] for the same `position += velocity · dt` / `velocity += acceleration · dt` form used in game physics.

   b. **Propose a position.** `p_new = p_old + v_new · step_seconds`. Uses the *new* velocity, not a midpoint.

   c. **Resolve walls** axis-by-axis (X first, then Y with the accepted X). Either axis may have its velocity component zeroed if the move would put the drone-radius footprint into a wall or off-grid (see *Wall collision*).

   d. **Drain battery.** `P = P_hover + k · |v_new|²` watts; subtract `P · step_seconds` joules from `battery_j`, clamped at zero. The drain uses the *post-collision* velocity so a fully blocked drone pays only `P_hover · step_seconds`.

3. **Update coverage.** After all drones have moved, every free cell within `sensor_radius` of any drone center is marked covered. The mask is cumulative — once covered, always covered.

4. **Advance clock.** `time_seconds += step_seconds`; `step_count += 1`.

What the caller can rely on:
- All drones are stepped against the *same* `actions` snapshot — no within-step communication. A controller that needs to react to drone *i*'s post-step state must wait for the next step.
- Coverage reflects the *post-step* world, so `env.coverage_fraction()` after `step()` includes the new positions.
- Battery is drained per drone independently and cannot go negative.

### Drone dynamics

Each drone is a 2D point mass with state `(pos, vel)`. The full integration sequence is described in *Per-step flow* above; this subsection just notes the design choices and defaults.

Configuration is split across two dataclasses so the names match what the values mean:

| Param | Lives in | Default | Demo override | Notes |
|---|---|---|---|---|
| `step_seconds` (`dt`) | `SimConfig` | 0.1 s | (kept) | Sim timestep — environment-level. |
| `meters_per_cell` | `SimConfig` | 5.0 m | (kept) | World-scale factor; see *World scale* above. |
| `sensor_radius` | `DroneConfig` | 1.5 cells (= 7.5 m) | (kept) | Disc sensor radius. |
| `max_speed` | `DroneConfig` | 1.0 cells/s (= 5 m/s) | 1.5 (= 7.5 m/s) | Magnitude cap, not per-axis. |
| `max_accel` | `DroneConfig` | 2.0 cells/s² (= 10 m/s²) | 2.5 (= 12.5 m/s²) | Magnitude cap, not per-axis. |
| `drone_radius` | `DroneConfig` | 0.05 cells (= 0.25 m) | (kept) | Used by wall collision only. |
| `mass_kg` | `DroneConfig` | 1.9 kg | (kept) | F450 + payload; informational, see *World scale*. |

`DroneConfig` is currently shared across every drone (homogeneous fleet). To support heterogeneous swarms later, pass a list of `DroneConfig` to `CoverageEnv` instead of one shared instance.

Two design choices worth justifying:

- **Explicit Euler.** Crude but adequate at `step_seconds = 0.1` and the speeds we exercise (worst-case displacement per step ≈ 0.15 cells, well under `drone_radius`). Higher-order integrators (RK4) would buy nothing here.
- **Magnitude clip, not per-axis.** Clipping each axis independently would change the *direction* of a commanded action whenever it lay outside the speed/accel box — fatal for any controller that produces direction vectors (Potential Fields, Consensus). Clipping by magnitude preserves direction and only attenuates magnitude.

### Wall collision

Drones have a `drone_radius` of 0.05 cells (≈ 0.25 m at the default scale, matching real F450 half-width). Collision is checked **axis-separately** so a blocked X move doesn't cancel a free Y move:

1. Try the candidate `(p_new.x, p.y)`. If the bounding box `[x − r, x + r] × [y − r, y + r]` lies inside the grid and overlaps no wall cell, accept the X move. Otherwise, zero `vel.x`.
2. Try `(accepted_x, p_new.y)` with the same test. Accept Y or zero `vel.y`.

This avoids the common point-mass gotcha where a drone moving diagonally into a corner sticks instead of sliding along one wall.

### Coverage and sensing

Each drone has an omnidirectional disc sensor of `sensor_radius` cells (default 1.5). After every step the coverage mask updates as:

```
for each drone d:
    for each cell (cx, cy) with center (cx + 0.5, cy + 0.5):
        if (cx + 0.5 − d.pos.x)² + (cy + 0.5 − d.pos.y)² ≤ sensor_radius²:
            covered[cy, cx] = True
```

Wall cells are pre-marked covered at `reset()`, so the headline metric divides only by coverable cells:

```
coverage_fraction = (covered ∧ free).sum() / free.sum()
```

Coverage is monotone in this model — once a cell is seen, it stays covered. Methods that try to re-cover decayed regions can be added later by adding a decay step in `_update_coverage`.

**Per-drone coverage and overlap metrics.** Every drone keeps its own boolean mask `env.drone_covered[i]` of free cells it has personally swept through. From these we derive three monotone metrics. Coverage stays as a fraction (it's a ratio of "this map" to "this map"); overlap is in **square meters** since the world is physically scaled.

- `env.coverage_fraction()` — **global coverage** (fraction of free cells covered by *any* drone). Bounded `[0, 1]`. Monotone.
- `env.drone_coverage_fraction(i)` — **per-drone coverage** fraction. Bounded `[0, 1]`. Monotone.
- `env.overlap_cells_m2()` — **overlap area** (m²): free territory touched by ≥ 2 drones at some point. Bounded by the total free area. Monotone. Reads as "X m² of the map is being double-covered." This is the headline coordination-quality metric — a working consensus controller should drive it *down* relative to a non-coordinating baseline.

### Battery

Per-drone, independent, modeled on the **Hawks F450** 11.1V 3S LiPo, 4200 mAh (≈46.62 Wh, 167,832 J). Constants live in `BatteryConfig` in `environment.py`; pass `battery=BatteryConfig(...)` to `CoverageEnv` to override.

#### Energy bookkeeping

Each drone holds one scalar, `battery_j` (joules remaining). The energy model is:

```
initial_energy_J  = V · (capacity_mAh / 1000) · 3600        # full-charge contents
P_inst            = P_hover + k · |v_new|²                  # instantaneous draw, watts
ΔE_step           = P_inst · step_seconds                   # joules consumed this step
battery_j        ← max(0, battery_j − ΔE_step)
mAh_used          = used_J / (V · 3.6)                      # for display
```

`v_new` is the velocity *after* the speed cap and wall collision in step 2d above — so blocked drones drain only `P_hover · step_seconds`. Drain is also gated by the voltage cutoff described next: once a drone reaches `min_voltage_v` it freezes in place and stops draining further.

#### Voltage cutoff (drone shutdown)

A real flight controller stops feeding the motors when battery voltage drops below a per-cell safety threshold (typically 3.0–3.5 V) to protect the LiPo from over-discharge damage. We model this with one extra `BatteryConfig` parameter:

```
min_voltage_v: float = 10.0       # ≈ 3.33 V/cell on a 3S pack
```

When `current_voltage_v(battery_j) ≤ min_voltage_v`, the drone is **depleted**: its velocity is zeroed, any incoming action is ignored, and battery drain stops (no point counting joules below the cutoff).

To map voltage to remaining energy we use a **linear `V(E)` approximation** between the per-cell full-charge and dead-cell anchors:

```
n_cells = round(voltage_v / 3.7)              # 11.1 V → 3 cells, 14.8 V → 4 cells
V_full  = 4.2 · n_cells                       # fully charged
V_dead  = 3.0 · n_cells                       # complete cell damage
V(E)    = V_dead + (V_full − V_dead) · (E / E_full)
```

For the F450 defaults this gives:
- `V(E_full) = 12.6 V`, `V(0) = 9.0 V`
- `min_voltage_v = 10.0 V` corresponds to ~27.8% remaining energy → cutoff at ~46,620 J

This is **deliberately conservative**. Real LiPo discharge has a steep drop near full, a long plateau around 3.7 V/cell, and a sharp knee near empty — not the straight line we use. A real 3S pack hits 10 V at ~5–10% remaining; our linear model trips the cutoff much earlier (~28% remaining). That's the safer side to err on for swarm planning. To delay the cutoff (riskier, more usable capacity), lower `min_voltage_v` (e.g., 9.5 V); to be more cautious, raise it (e.g., 10.5 V).

**Visual indicator.** Depleted drones render in **gray** with a `☠` next to their label, and their row in the battery panel is tagged `[DEAD]`. The simulation continues so other drones can keep operating.

#### Where the unit conversions come from

Both formulas above are exact dimensional identities, not approximations.

- **`1 mAh @ V volts = V · 3.6 J`.** A milliamp-hour is a charge unit: `1 mA · 1 h = 0.001 A · 3600 s = 3.6 C`. Multiplied by voltage you get energy in joules: `V · 3.6 J/mAh`. Hence `4200 mAh × 11.1 V = 4200 · 11.1 · 3.6 = 167,832 J = 46.62 Wh` — and the reverse, `mAh = J / (V · 3.6)`, is what `battery_state()` reports. See [Rebel-Cell's battery terminology guide][src-battery-units] or [RELiON's Wh-vs-Ah explainer][src-battery-wh-ah] for the underlying `Wh = V · Ah` identity.
- **`1 Wh = 3600 J`.** So `P_hover = 165 W` against a 167,832 J pack gives `167,832 / 165 ≈ 1017 s ≈ 17 min` of pure-hover endurance — consistent with reported F450 flight times of ~18 min on a 5000 mAh 3S pack and ~10 min on a 2200 mAh 3S pack ([source][src-f450-times]).

#### Where the constants come from

| Symbol | Value | Justification |
|---|---|---|
| `V` (`voltage_v`) | 11.1 V | 3S LiPo nominal (3 cells × 3.7 V); standard battery for the F450 ([source][src-3s-lipo]). |
| `capacity_mah` | 4200 mAh | User-specified Hawks F450 reference pack. |
| `P_hover` (`hover_power_w`) | 165 W | Back-calculated from reported F450 flight times: a 5000 mAh 3S (55.5 Wh) pack lasting ~18 min implies ~185 W, while a 2200 mAh 3S (24.4 Wh) pack lasting ~10 min implies ~146 W ([source][src-f450-times]). 165 W sits in the middle and gives the canonical ~17 min hover endurance. |
| `k` (`motion_coeff_w_per_v2`) | 13 W per (cell/s)² | Calibrated to real F450: at `v = 15 m/s = 3 cells/s`, model power = 165 + 13·9 ≈ 282 W, matching the ~280 W reported for max-speed forward flight (≈ +70% over hover). Derivation: `k_m·15² = 115 W` → `k_m ≈ 0.51 W·s²/m²`, then `k_cells = k_m × meters_per_cell² = 0.51 × 25 ≈ 13`. |
| `min_voltage_v` | 10.0 V | ≈ 3.33 V/cell on 3S — at the edge of LiPo cell-damage territory. Drone freezes when reached. See *Voltage cutoff* above. |
| `cell_full_voltage_v`, `cell_dead_voltage_v` | 4.2 V, 3.0 V | Per-cell anchors for the linear `V(E)` discharge approximation used by the cutoff. |

**Why `v²` and not `v³`?** The full multirotor power model splits propulsion power into three terms ([Multirotor Power Consumption (Liu et al., 2022)][src-multirotor-power], [Bauersfeld & Scaramuzza, 2021][src-bauersfeld]):

```
P_total = P_induced(v)  +  P_profile(v)  +  P_parasitic(v)
            ~T^1.5           ~v²              ~v³
```

- `P_induced` actually *decreases* slightly at moderate forward speed (translational lift), then rises again. Net effect: a small dip in power around 5–7 m/s for typical quadcopters ([Quadcopter Flight School][src-translational-lift], [Stanford quadrotor aerodynamics][src-stanford-quad]).
- `P_profile` (blade drag) scales **quadratically** with speed and dominates at moderate cruise.
- `P_parasitic` (airframe drag) scales **cubically** with speed and dominates at high cruise (~15+ m/s).

Our model captures only the `v²` (profile) term, which is dominant in the speed regime we exercise (`max_speed = 7.5 m/s`). The `v³` term would matter more if `max_speed` were raised toward the F450's true ceiling. The translational-lift dip is **deliberately ignored** so the cost is monotone — useful for optimization, slightly pessimistic at moderate cruise.

**Calibration check.** With `k = 13 W·(cells/s)⁻²` and `meters_per_cell = 5`:

| Speed | Cells/s | P (W) | vs hover |
|---|---|---|---|
| 0 (hover) | 0 | 165 | baseline |
| 7.5 m/s (cruise / our cap) | 1.5 | 165 + 13·2.25 ≈ **194** | +18% |
| 15 m/s (real F450 max) | 3.0 | 165 + 13·9 ≈ **282** | +71% ✓ matches reports |

If a future controller starts gaming the missing translational-lift dip (sitting at 5–7 m/s to "save" power that real drones do save), swap `k · v²` for the full Liu/Bauersfeld closed form.

#### Worked example: one step at full cruise

```
v_new = max_speed = 1.5 cells/s = 7.5 m/s
P_inst = 165 + 13 · 1.5² = 194.25 W
ΔE_step = 194.25 · 0.1 = 19.425 J
       = 19.425 / (11.1 · 3.6) ≈ 0.486 mAh
```

At 10 steps per second of sim time, that's ≈4.86 mAh/s. A full 4200 mAh pack would last `4200 / 4.86 ≈ 864 s ≈ 14.4 min` if the drone could maintain `max_speed` continuously — vs ~17 min at pure hover, a 15% endurance hit for sustained cruise. That difference is the signal an energy-aware controller should optimize against.

#### Where you see it

- `env.battery_state(idx)` returns a dict with raw joules, mAh used / remaining, and percent.
- The demo prints the starting battery (same for every drone) and a per-drone final breakdown.
- Every rendered frame overlays a panel listing every drone's used / remaining mAh.

#### Sources

External references for the numbers and equations above:

- [DJI F450 specs and reported flight times][src-f450-times] — used to back-calculate the 144–185 W hover-power range from real flight-time data
- [3S LiPo voltage specification (Roger's Hobby Center)][src-3s-lipo] — confirms 11.1 V nominal = 3 cells × 3.7 V
- [Rebel-Cell battery terminology guide][src-battery-units] and [RELiON Wh vs Ah explainer][src-battery-wh-ah] — `Wh = V · Ah` and the mAh/Joules unit conversion
- [Quadcopter Flight School: hover vs. forward flight power efficiency][src-translational-lift] — explains the translational-lift dip
- [Stanford quadrotor aerodynamics & control (Hoffmann et al., 2007)][src-stanford-quad] — induced-power-vs-speed for small quadrotors
- [Modelling Power Consumptions for Multi-rotor UAVs (Liu et al., 2022)][src-multirotor-power] — closed-form decomposition into induced + profile + parasitic terms; supports our `v²` choice for the dominant term in our speed regime
- [Range, Endurance, and Optimal Speed Estimates for Multicopters (Bauersfeld & Scaramuzza, 2021)][src-bauersfeld] — F450-class measured power-vs-speed used to back-calibrate `k = 13 W·(cells/s)⁻²`
- [Wikipedia: Euler method][src-euler-wiki] and [Gaffer On Games: Integration Basics][src-euler-game] — explicit Euler integration for the position/velocity update

[src-f450-times]: https://www.amainhobbies.com/dji-flame-wheel-f450-quadcopter-drone-combo-kit-dji-nzm450c1/p297771
[src-3s-lipo]: https://www.hawks-work.com/pages/rc-battery-4200-xt60
[src-battery-units]: https://www.rebel-cell.com/knowledge-base/battery-terminology/
[src-battery-wh-ah]: https://www.relionbattery.com/blog/whats-the-difference-in-amp-hours-and-watt-hours
[src-translational-lift]: http://quadcopter101.blogspot.com/2014/02/flight-school-5-power-efficiency-hover.html
[src-stanford-quad]: https://ai.stanford.edu/~gabeh/papers/Quadrotor_Dynamics_GNC07.pdf
[src-multirotor-power]: https://arxiv.org/pdf/2209.04128
[src-bauersfeld]: https://rpg.ifi.uzh.ch/docs/Arxiv21_Bauersfeld.pdf
[src-euler-wiki]: https://en.wikipedia.org/wiki/Euler_method
[src-euler-game]: https://gafferongames.com/post/integration_basics/

### Intentionally omitted (gap to Isaac)

The 2D model leaves out everything that doesn't change algorithmic feasibility:

- attitude / tilt dynamics (a real quad must tilt to translate; here velocity is direct)
- aerodynamic drag and rotor wake interactions
- actuator delay and motor lag
- IMU / GPS noise, EKF lag
- wind disturbance
- battery sag (transient voltage drop under load) and the actual non-linear LiPo discharge curve — we use a linear `V(E)` approximation only for the voltage cutoff threshold
- altitude — everything is at one height; no 3D sensor footprint

These are the gaps that make 2D **falsification but not validation**: a controller that fails here will fail in Isaac; a controller that works here may still need re-tuning once these effects appear.

---

## Setup

```bash
cd ~/Desktop/Swarm_Drones/optimization

# First time only:
python3 -m venv .optim_env
source .optim_env/bin/activate
pip install -r requirements.txt

# Every session:
source .optim_env/bin/activate
```

Direct deps: `numpy`, `matplotlib`. The native `MacOSX` backend is preferred on macOS to avoid a known TkAgg / system-Tcl version crash.

---

## Run modes

The demo runner (`tools/demo.py`) is the main entrypoint. Two run modes, two persistence modes, freely combined.

### Headless (default)

Renders frames to PNGs without opening a window. Output files **overwrite** on every run — they are scratch artifacts.

```bash
python tools/demo.py                                  # default
python tools/demo.py --headless                       # explicit (same thing)
python tools/demo.py --map maze --drones 4
```

Writes to `outputs/images/`:
- `demo_initial.png` — first frame
- `demo_final.png` — last frame
- `coverage_curve.png` — coverage % over time

### GUI (live animation)

Opens an interactive window. Drones move in real time; the coverage trail fills in as a green overlay.

```bash
python tools/demo.py --gui
python tools/demo.py --gui --map maze --drones 4
```

### Persisting outputs with `--save`

By default, output files are scratch (overwritten next run). Add `--save` to keep them: outputs are renamed with a `SAVED_<tag>_` prefix so multiple runs accumulate side-by-side.

```bash
python tools/demo.py --save                           # SAVED_<timestamp>_*.png
python tools/demo.py --save baseline_random           # SAVED_baseline_random_*.png
python tools/demo.py --gui --save run3                # also writes SAVED_run3_animation.gif
```

Tag rules:
- `--save` (no value) → `SAVED_20260426_180012_demo_initial.png`
- `--save NAME` → `SAVED_NAME_demo_initial.png`

`--gui` and `--headless` are mutually exclusive; argparse rejects the combination.

### All flags

| Flag | Default | Notes |
|---|---|---|
| `--headless` / `--gui` | `--headless` | mutually exclusive |
| `--save [TAG]` | off (overwrite) | omit TAG for timestamp |
| `--map {random,maze}` | `random` | auto-generators |
| `--map-file PATH` | none | load `.npy` / `.txt` map |
| `--size N` | 55 | grid side length (in cells) |
| `--drones N` | 3 | swarm size |
| `--seed N` | 17 | reproducibility |

**There is intentionally no `--steps` flag and no step-count cap.** The simulation runs until one of two natural terminal conditions is reached:
- `env.is_done()` — 100% coverage (mission success)
- `env.all_depleted()` — every drone hit its voltage cutoff (mission failure)

The all-depleted condition is **physics-bounded**: maximum sim time is `(initial_energy − cutoff_energy) / P_hover ≈ 121 kJ / 165 W ≈ 734 s ≈ 7340 steps`. Past that, every drone has hit cutoff and `is_terminal()` fires automatically — no artificial step cap needed.

If you want shorter runs, change the swarm config (more drones, larger sensor radius, smaller map) — that's what controls real coverage missions, not arbitrary clock cycles.

---

## Creating maps

Two ways to feed a map into `tools/demo.py`.

### A — auto-generate (built into `tools/demo.py`)

- `--map random` — boundary walls + scattered interior obstacles, BFS-validated to keep the free space connected, retried on failure
- `--map maze` — recursive-backtracker perfect maze with 1-cell-wide corridors

No file is written; the grid is regenerated each run from `--seed`.

### B — hand-draw with the editor

```bash
python tools/editor.py --size 21
```

Controls:
- **left-drag** — paint walls
- **right-drag** — erase to free
- **s** — save (default path: `outputs/maps/custom_map.npy`)
- **c** — clear interior (keep boundary)
- **r** — reset to fresh boundary-walled grid
- **f** — fill all interior with walls
- **v** — toggle validation overlay
- **g** — toggle grid lines
- **q** — quit

Live validation overlays:
- **red** — free cells in a disconnected pocket (unreachable from main area)
- **orange** — free cells touching the outer border (boundary wall missing)

Edit an existing map and save to a new file:
```bash
python tools/editor.py --load outputs/maps/custom_map.npy --out outputs/maps/v2.npy
```

Then run the demo on it:
```bash
python tools/demo.py --map-file outputs/maps/custom_map.npy
```

---

## Outputs

Everything generated lives under `outputs/` (gitignored):

```
outputs/
├── images/   demo PNGs and animation GIFs
└── maps/     editor-saved .npy / .txt grids
```

Both directories are created on demand. Don't write generated files anywhere else — the project root stays for source only.

---

## Implementation status

| Component | State |
|---|---|
| Map generators (`open_arena`, `random_obstacles`, `recursive_backtracker`) | ✅ done |
| Map validators (`is_fully_connected`, `find_components`, `disconnected_cells`, `boundary_breaches`) | ✅ done |
| Interactive map editor (`tools/editor.py`) | ✅ done |
| Environment (`CoverageEnv` — continuous actions, speed/accel caps, wall collision, coverage tracking) | ✅ done |
| Headless + GUI visualization (`render_frame`, `animate`) | ✅ done |
| Random-action baseline (`random_policy` in `tools/demo.py`) | ✅ done — plateaus ~25% |
| Battery model (`BatteryConfig`, F450 11.1V 4200mAh, hover + speed²) | ✅ done — exposed via `CoverageEnv.battery_state()` |
| Voltage cutoff / drone shutdown (`min_voltage_v`, linear `V(E)` discharge approximation) | ✅ done — depleted drones freeze, render gray, tagged `[DEAD]` |
| **Potential Fields controller** | ⬜ next |
| Per-drone `comm_range` on `CoverageEnv` (limited-view neighbor observations) | ⬜ prerequisite for Consensus — see *Consensus-Based Coordination* below |
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
- Coverage: plateaus ~24% under random Gaussian acceleration commands.
- Purpose: lower bound — any real controller must beat this.

#### Potential Fields
_Pending implementation._

#### Consensus-Based Coordination
_Pending implementation._

**Prerequisite — comm-range modeling.** Consensus depends on drones exchanging state with their neighbors. In real flight that's done by a mesh routing protocol on each drone's companion computer (e.g., [BATMAN-adv](https://www.open-mesh.org/projects/batman-adv/wiki/BATMAN_IV) on a Jetson Nano's Wi-Fi link). In this 2D testbed there's no network, so before implementing Consensus we need to add a `comm_range` parameter to `CoverageEnv` (likely on `DroneConfig`) so each drone's observation is restricted to peers within range. That *models* the mesh's effective behavior — who can hear whom — without running a real mesh stack, which only earns its keep when packets and antennas are physical.

#### Multi-Agent Reinforcement Learning
_Pending implementation._
