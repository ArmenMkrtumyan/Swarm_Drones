# Battery model

Per-drone, independent, modeled on the **Hawk's Work F450** build with **A2212 920 KV motors, 9450 props, and an 11.1 V 3S LiPo, 4200 mAh** ([product page](https://www.hawks-work.com/pages/f450-drone)). Hawk's Work F450 is a clone of the DJI Flame Wheel F450 — same airframe class, so published F450 hover-time data from DJI users (see [`f450-reference.md`](f450-reference.md)) applies directly to our build. Pack energy in physical units (full derivations in *Where the unit conversions come from* below):

- **Watt-hours**: `voltage × capacity = 11.1 V × 4.2 Ah = 46.62 Wh`  *(uses Wh = V · Ah; mAh → Ah by dividing by 1000)*
- **Joules**: `Wh × 3600 s/h = 46.62 × 3600 = 167,832 J`  *(1 Wh = 3600 J because 1 W = 1 J/s)*
- **Equivalent direct form**: `V × mAh × 3.6 = 11.1 × 4200 × 3.6 = 167,832 J`  *(since 1 mAh @ V volts = V · 3.6 J)*

Constants live in `BatteryConfig` in `environment.py`; pass `battery=BatteryConfig(...)` to `CoverageEnv` to override. See [f450-reference.md](f450-reference.md) for what other batteries the F450 supports and what hover times to expect.

## Energy bookkeeping

Each drone holds one scalar, `battery_j` (joules remaining). The energy model is:

```
initial_energy_J  = V · (capacity_mAh / 1000) · 3600        # full-charge contents
P_inst            = P_hover + k · |v_new|²                  # instantaneous draw, watts
ΔE_step           = P_inst · step_seconds                   # joules consumed this step
battery_j        ← max(0, battery_j − ΔE_step)
mAh_used          = used_J / (V · 3.6)                      # for display
```

**Where these equations come from:**

- `initial_energy_J = V · (capacity_mAh / 1000) · 3600` — the dimensional identity `Wh = V · Ah` ([RELiON: Wh vs Ah explainer][src-battery-wh-ah]) plus `1 Wh = 3600 J`. The `/1000` converts mAh to Ah. Re-derived in *Where the unit conversions come from* below.
- `P_inst = P_hover + k · |v_new|²` — quadratic-in-speed profile-power model. The full multirotor decomposition has three terms (`P_induced ~ T^1.5`, `P_profile ~ v²`, `P_parasitic ~ v³`); we keep only the dominant `v²` term for our 0–7.5 m/s regime. Theoretical basis and term-by-term derivation in [Liu et al., 2022 — *Modelling Power Consumptions for Multi-rotor UAVs*][src-multirotor-power]; F450-class measured power-vs-speed (used to back-calibrate `k = 13`) in [Bauersfeld & Scaramuzza, 2021 — *Range, Endurance, and Optimal Speed Estimates for Multicopters*][src-bauersfeld]. Why we drop the `v³` term and the translational-lift dip is discussed under *Why `v²` and not `v³`* below.
- `ΔE_step = P_inst · step_seconds` — the definition of power: `P = dE/dt`, so over a small interval `Δt`, `ΔE ≈ P · Δt` ([Wikipedia: Power (physics)][src-power-definition]). Exact for constant `P_inst` over the step; we treat `P_inst` as constant within `step_seconds` because the drone's velocity is held fixed for the whole step (it's set once at the top of the step and only changes on the *next* step's integration).
- `mAh_used = used_J / (V · 3.6)` — inverse of `1 mAh @ V volts = V · 3.6 J` (see *Where the unit conversions come from* below for the Coulomb derivation).

`v_new` is the velocity *after* the speed cap and wall collision in step 2d of [`per-step flow`](simulation-model.md#per-step-flow) — so blocked drones drain only `P_hover · step_seconds`. Drain is also gated by the voltage cutoff described next: once a drone reaches `min_voltage_v` it freezes in place and stops draining further.

## Voltage cutoff (drone shutdown)

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

## Where the unit conversions come from

Both formulas above are exact dimensional identities, not approximations.

- **`1 mAh @ V volts = V · 3.6 J`.** A milliamp-hour is a charge unit: `1 mA · 1 h = 0.001 A · 3600 s = 3.6 C`. Multiplied by voltage you get energy in joules: `V · 3.6 J/mAh`. Hence `4200 mAh × 11.1 V = 4200 · 11.1 · 3.6 = 167,832 J = 46.62 Wh` — and the reverse, `mAh = J / (V · 3.6)`, is what `battery_state()` reports. See [Rebel-Cell's battery terminology guide][src-battery-units] or [RELiON's Wh-vs-Ah explainer][src-battery-wh-ah] for the underlying `Wh = V · Ah` identity.
- **`1 Wh = 3600 J`.** So `P_hover = 165 W` against a 167,832 J pack gives `167,832 / 165 ≈ 1017 s ≈ 17 min` of pure-hover endurance — consistent with reported F450 flight times (see [f450-reference.md](f450-reference.md)).

## Where the constants come from

| Symbol | Value | Justification |
|---|---|---|
| `V` (`voltage_v`) | 11.1 V | 3S LiPo nominal (3 cells × 3.7 V); standard battery for the F450 ([source][src-3s-lipo]). |
| `capacity_mah` | 4200 mAh | Hawk's Work F450 reference pack ([3S 4200 mAh, 25 C, XT60][src-3s-lipo]). |
| `P_hover` (`hover_power_w`) | 165 W | Back-calculated from reported F450 flight times: a 5000 mAh 3S (55.5 Wh) pack lasting ~18 min implies ~185 W, while a 2200 mAh 3S (24.4 Wh) pack lasting ~10 min implies ~146 W ([source][src-f450-times]). 165 W sits in the middle and gives the canonical ~17 min hover endurance. |
| `k` (`motion_coeff_w_per_v2`) | 13 W per (cell/s)² | Calibrated to real F450: at `v = 15 m/s = 3 cells/s`, model power = 165 + 13·9 ≈ 282 W, matching the ~280 W reported for max-speed forward flight (≈ +70% over hover). Derivation: `k_m·15² = 115 W` → `k_m ≈ 0.51 W·s²/m²`, then `k_cells = k_m × meters_per_cell² = 0.51 × 25 ≈ 13`. |
| `min_voltage_v` | 10.0 V | ≈ 3.33 V/cell on 3S — at the edge of LiPo cell-damage territory. Drone freezes when reached. See *Voltage cutoff* above. |
| `cell_full_voltage_v`, `cell_dead_voltage_v` | 4.2 V, 3.0 V | Per-cell anchors for the linear `V(E)` discharge approximation used by the cutoff. |

## Why `v²` and not `v³`?

The full multirotor power model splits propulsion power into three terms ([Liu et al., 2022][src-multirotor-power], [Bauersfeld & Scaramuzza, 2021][src-bauersfeld]):

```
P_total = P_induced(v)  +  P_profile(v)  +  P_parasitic(v)
            ~T^1.5           ~v²              ~v³
```

- `P_induced` actually *decreases* slightly at moderate forward speed (translational lift), then rises again. Net effect: a small dip in power around 5–7 m/s for typical quadcopters ([Quadcopter Flight School][src-translational-lift], [Stanford quadrotor aerodynamics][src-stanford-quad]).
- `P_profile` (blade drag) scales **quadratically** with speed and dominates at moderate cruise.
- `P_parasitic` (airframe drag) scales **cubically** with speed and dominates at high cruise (~15+ m/s).

Our model captures only the `v²` (profile) term, which is dominant in the speed regime we exercise (`max_speed = 7.5 m/s`). The `v³` term would matter more if `max_speed` were raised toward the F450's true ceiling. The translational-lift dip is **deliberately ignored** so the cost is monotone — useful for optimization, slightly pessimistic at moderate cruise.

If a future controller starts gaming the missing translational-lift dip (sitting at 5–7 m/s to "save" power that real drones do save), swap `k · v²` for the full Liu/Bauersfeld closed form.

### Calibration check

With `k = 13 W·(cells/s)⁻²` and `meters_per_cell = 5`:

| Speed | Cells/s | P (W) | vs hover |
|---|---|---|---|
| 0 (hover) | 0 | 165 | baseline |
| 7.5 m/s (cruise / our cap) | 1.5 | 165 + 13·2.25 ≈ **194** | +18% |
| 15 m/s (real F450 max) | 3.0 | 165 + 13·9 ≈ **282** | +71% ✓ matches reports |

## Worked example: one step at full cruise

```
v_new = max_speed = 1.5 cells/s = 7.5 m/s
P_inst = 165 + 13 · 1.5² = 194.25 W
ΔE_step = 194.25 · 0.1 = 19.425 J
       = 19.425 / (11.1 · 3.6) ≈ 0.486 mAh
```

At 10 steps per second of sim time, that's ≈4.86 mAh/s. A full 4200 mAh pack would last `4200 / 4.86 ≈ 864 s ≈ 14.4 min` if the drone could maintain `max_speed` continuously — vs ~17 min at pure hover, a 15 % endurance hit for sustained cruise. That difference is the signal an energy-aware controller should optimize against.

## Where you see it

- `env.battery_state(idx)` returns a dict with raw joules, mAh used / remaining, and percent.
- The demo prints the starting battery (same for every drone) and a per-drone final breakdown.
- Every rendered frame overlays a panel listing every drone's used / remaining mAh.

The model is verified end-to-end by [`verification_scripts/test_flight_time.py`](verification.md#test_flight_timepy) — it runs `env.step()` to depletion in hover and cruise regimes, asserts agreement with the analytical formula within 0.5 %, and prints comparison with the published F450 hover range.

## Sources

External references for the numbers and equations above:

- [DJI F450 specs and reported flight times][src-f450-times] — used to back-calculate the 144–185 W hover-power range from real flight-time data
- [3S LiPo voltage specification (Roger's Hobby Center)][src-3s-lipo] — confirms 11.1 V nominal = 3 cells × 3.7 V
- [Rebel-Cell battery terminology guide][src-battery-units] and [RELiON Wh vs Ah explainer][src-battery-wh-ah] — `Wh = V · Ah` and the mAh/Joules unit conversion
- [Wikipedia: Power (physics)][src-power-definition] — `P = dE/dt`, basis for the `ΔE = P · Δt` step integration
- [Quadcopter Flight School: hover vs. forward flight power efficiency][src-translational-lift] — explains the translational-lift dip
- [Stanford quadrotor aerodynamics & control (Hoffmann et al., 2007)][src-stanford-quad] — induced-power-vs-speed for small quadrotors
- [Modelling Power Consumptions for Multi-rotor UAVs (Liu et al., 2022)][src-multirotor-power] — closed-form decomposition into induced + profile + parasitic terms; supports our `v²` choice for the dominant term in our speed regime
- [Range, Endurance, and Optimal Speed Estimates for Multicopters (Bauersfeld & Scaramuzza, 2021)][src-bauersfeld] — F450-class measured power-vs-speed used to back-calibrate `k = 13 W·(cells/s)⁻²`

[src-f450-times]: https://www.amainhobbies.com/dji-flame-wheel-f450-quadcopter-drone-combo-kit-dji-nzm450c1/p297771
[src-3s-lipo]: https://www.hawks-work.com/pages/rc-battery-4200-xt60
[src-battery-units]: https://www.rebel-cell.com/knowledge-base/battery-terminology/
[src-battery-wh-ah]: https://www.relionbattery.com/blog/whats-the-difference-in-amp-hours-and-watt-hours
[src-power-definition]: https://en.wikipedia.org/wiki/Power_(physics)
[src-translational-lift]: http://quadcopter101.blogspot.com/2014/02/flight-school-5-power-efficiency-hover.html
[src-stanford-quad]: https://ai.stanford.edu/~gabeh/papers/Quadrotor_Dynamics_GNC07.pdf
[src-multirotor-power]: https://arxiv.org/pdf/2209.04128
[src-bauersfeld]: https://rpg.ifi.uzh.ch/docs/Arxiv21_Bauersfeld.pdf
