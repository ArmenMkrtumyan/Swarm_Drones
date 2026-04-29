# Verification scripts

`verification_scripts/` collects standalone scripts that each prove one specific aspect of the model is correct — hand-computed expectations + assertions, kept as instructor-facing artifacts separate from `tools/` (which holds runnable demos and the map editor).

| Script | Verifies |
|---|---|
| [`test_overlap.py`](#test_overlappy) | Visit-count overlap metrics (`total_visits`, `self_revisits`, `cross_overlap_visits`, `wasted_visits_total`) against a 2-drone scripted scenario. 9 hand-computed assertions. |
| [`test_flight_time.py`](#test_flight_timepy) | Battery / energy model (`P_hover + k·v²`, voltage cutoff). Runs `env.step()` to depletion in hover and cruise regimes; asserts agreement with the analytical formula < 0.5 % and with the published F450 hover range (12–15 min for our 3S 4200 mAh + ~1.9 kg config). |
| [`test_distance.py`](#test_distancepy) | Tile scale (`meters_per_cell = 5.0`) and motion integration. Asserts F450 derived dimensions, that constant-velocity path length matches `v·t·m_per_cell` exactly, that sensor disc area falls within ±30 % of `π·r²`, and that range-to-cutoff equals `cruise_endurance × cruise_speed × m_per_cell`. |

All three scripts have a headless mode (default — runs assertions to completion, exits 0/1) and a `--gui` mode (live animation). The GUI titles refresh as the sim runs so you can watch the metric in question evolve.

## `test_overlap.py`

```bash
python verification_scripts/test_overlap.py            # headless: writes 4 PNGs + asserts 9 metric values
python verification_scripts/test_overlap.py --gui      # live animation: watch self↺ and wasted visits tick up
```

Deterministic 2-drone scenario in a 25×5 corridor with `sensor_range = 0.01` (degenerate wedge — only the drone's own-cell special case fires, so coverage = exactly the drone's current cell regardless of heading; gives clean one-cell-per-position semantics for hand-computable expectations). Three phases:

- **Phase A**: D1 walks east, paints row y=2.
- **Phase B**: D2 enters D1's territory and walks east through it (pure cross-overlap).
- **Phase C**: D2 retraces west (every entry is BOTH cross-overlap AND self-revisit).

Asserts 9 hand-computed values: per-drone `total_visits`, `unique_cells`, `self_revisits`, `cross_overlap_visits`, plus swarm `wasted_visits_total`.

The GUI mode steps through phases A → B → C at 4 fps with the current phase label on the figure suptitle — Phase C is where `self↺` and `wasted visits` start moving and the area-only `overlap area: m²` doesn't (because it's set-membership, not entry-event). See [`simulation-model.md` → Visit-count metrics](simulation-model.md#visit-count-metrics-entry-event) for the metric definitions.

## `test_flight_time.py`

```bash
python verification_scripts/test_flight_time.py            # headless: prints reference table + asserts
python verification_scripts/test_flight_time.py --gui      # live view, 1× / 5× / 10× / 20× speed slider
```

One drone at the F450 reference config (3S 4200 mAh, 1.9 kg). Two regimes:

1. **Hover** (`v = 0`): drone stays put, drain = `P_hover` only. Asserts depletion time = `(E_initial − E_cutoff) / P_hover` within 0.5 %.
2. **Cruise** (`v = max_speed = 7.5 m/s`): drone flies a 2-cell-radius circle (visual) or teleport-pinned (headless), drain = `P_hover + k·v²`. Asserts depletion time = `(E_initial − E_cutoff) / (P_hover + k·v²)` within 0.5 %.

Reality check: the sim's hover endurance (12.24 min) lands inside the published F450 range of 12–15 min for our config ✓. The cruise endurance (10.40 min) sits **below** the inferred real-world range of 12–18 min — pessimistic by design (the `v²` monotone power model omits the translational-lift dip; see [`f450-reference.md` → Cruise (forward flight)](f450-reference.md#cruise-forward-flight)).

`--gui` opens a two-pane window with a `1× / 5× / 10× / 20×` speed slider:

- **Phase 1 (HOVER)**: drone hovers in the arena center. Battery counter ticks down from 100 % to 0 % (where 0 % is the 10 V cutoff, not full discharge).
- **Phase 2 (CRUISE)**: drone flies a 2-cell-radius circle at `max_speed`. Battery drains faster.
- **Phase 3 (DONE)**: title locks at the side-by-side comparison `hover X min Y sec | cruise X min Y sec | F450 reference: hover 12-15 min (published) | cruise 12-18 min (inferred — sim pessimistic by design)`.

At 20× the slider compresses a 12-min sim hover into ~18 s real time.

## `test_distance.py`

```bash
python verification_scripts/test_distance.py            # headless: 4 tests, asserts
python verification_scripts/test_distance.py --gui      # live distance accumulator, 1× / 5× / 10× / 20×
```

Four headless tests:

1. **Tile scale**: `meters_per_cell == 5.0`. Derived F450 dimensions (sensor 7.5 m, max_speed 7.5 m/s, max_accel 12.5 m/s² ≈ 1.3 g, drone radius 0.25 m).
2. **Linear distance**: 100 steps at constant velocity → path length = displacement = `v · t · m_per_cell` exactly (Δ < 0.01 %).
3. **Sensor footprint**: cell-discretized disc area within ±30 % of continuous `π · r²`. At r = 1.5 cells = 7.5 m, 9 cells × 25 m²/cell = 225 m² vs continuous 176.7 m² — discretization-dominated, but the *units* line up.
4. **Range to cutoff**: drone at `max_speed` until battery cutoff. Asserts total distance = `cruise_endurance × max_speed × m_per_cell` (Δ < 0.5 %, accounting for final-step cutoff clamp). Sim predicts **4.68 km** vs inferred real F450 5.4–8.1 km.

`--gui` shows the cruise-to-cutoff distance accumulation in real time: drone flies a 2-cell circle at `max_speed`, the figure suptitle shows running counters for elapsed time, distance covered (m and % of expected total), and battery % remaining. Final state title shows `range traveled: X.XX km` + the inferred F450 reference range.
