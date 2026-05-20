# Battery model

Per-drone, independent, modeled on the **Hawk's Work F450** build with **A2212 920 KV motors, 9450 props, and an 11.1 V 3S LiPo, 4200 mAh** ([product page](https://www.hawks-work.com/pages/f450-drone)). Hawk's Work F450 is a clone of the DJI Flame Wheel F450 — same airframe class, so published F450 hover-time data from DJI users (see [`f450-reference.md`](f450-reference.md)) applies directly to our build. Pack energy in physical units (full derivations in *Where the unit conversions come from* below):

- **Watt-hours**: `voltage × capacity = 11.1 V × 4.2 Ah = 46.62 Wh`  *(uses Wh = V · Ah; mAh → Ah by dividing by 1000)*
- **Joules**: `Wh × 3600 s/h = 46.62 × 3600 = 167,832 J`  *(1 Wh = 3600 J because 1 W = 1 J/s)*
- **Equivalent direct form**: `V × mAh × 3.6 = 11.1 × 4200 × 3.6 = 167,832 J`  *(since 1 mAh @ V volts = V · 3.6 J)*

Constants live in `BatteryConfig` in `environment.py`; pass `battery=BatteryConfig(...)` to `CoverageEnv` to override.

## Battery options the F450 supports

The F450 frame officially accepts **3S or 4S LiPo** packs (per the [Hawk's Work product spec](https://www.hawks-work.com/pages/f450-drone)). Anything outside 3S–4S is out of spec for typical 920 KV motors and 20 A ESCs.

| Pack | Voltage | Common capacities | Prop size | Notes |
|---|---|---|---|---|
| **3S LiPo** (this project) | 11.1 V nominal (12.6 V full → 9.0 V empty) | 2200 / 2700 / 4000 / **4200(chosen)** / 5000 mAh | **9450 (chosen)** or 1045 | Hawk's Work F450 ships with 9450 props on 3S. Hover power ≈ 145–185 W depending on mass and prop choice. |
| **4S LiPo** (alternative) | 14.8 V nominal (16.8 V full → 12.0 V empty) | 3300 / 4000 / 6000 mAh | **8" required** (NOT 9450 or 1045) | Faster forward flight and more thrust headroom; mandatory smaller props because 9450/10" at 4S overspeed and overheat 920 KV motors. |

## Energy bookkeeping

Each drone holds one scalar, `battery_j` (joules remaining). Per-step update:

```
initial_energy_J  = V · (capacity_mAh / 1000) · 3600     # Wh = V·Ah → J
P_inst            = P_hover + k · |v_new|²               # watts (see "Why v²" below)
ΔE_step           = P_inst · step_seconds                # joules per env.step()
battery_j        ← max(0, battery_j − ΔE_step)
mAh_used          = used_J / (V · 3.6)                   # for display
```

`v_new` is the *post-collision* velocity (so wall-blocked drones drain only `P_hover · step_seconds`); `P_inst` is held constant across one `step_seconds`. Drain stops at the voltage cutoff (next section).

## Voltage cutoff (drone shutdown)

A real flight controller stops feeding the motors when battery voltage drops below a per-cell safety threshold (typically 3.0–3.5 V) to protect the LiPo from over-discharge damage. 

```
min_voltage_v: float = 10.0       # ≈ 3.33 V/cell on a 3S pack
```

When `current_voltage_v(battery_j) ≤ min_voltage_v`, the drone is **depleted**: its velocity is zeroed, any incoming action is ignored, and battery drain stops.

To map voltage to remaining energy we use a **linear `V(E)` approximation** between the per-cell full-charge and dead-cell anchors:

```
n_cells = round(voltage_v / 3.7)    # 11.1 V → 3 cells, 14.8 V → 4 cells
V_full  = 4.2 · n_cells             # fully charged
V_dead  = 3.0 · n_cells             # complete cell damage
V(E)    = V_dead + (V_full − V_dead) · (E / E_full)
```

For the F450 defaults this gives:

- `V(E_full) = 12.6 V`, `V(0) = 9.0 V`
- `min_voltage_v = 10.0 V` corresponds to ~27.8% remaining energy → cutoff at ~46,620 J

## Mass-aware hover power

`hover_power_w` is derived from `DroneConfig.mass_kg` at env construction, so changing mass auto-rescales hover power. The formula is momentum theory (actuator-disk model: `P_ideal = T·√(T/(2·ρ·A)) = T^1.5/√(2·ρ·A)`, real rotors `P_real = P_ideal / FoM` after motor/ESC losses + blade non-ideality):

```
P_hover(m) = (m · g)^1.5 / (FoM · √(2 · ρ · A_disk))
where:
    m       = DroneConfig.mass_kg
    g       = 9.81 m/s²
    ρ       = 1.225 kg/m³                  (ISA sea-level; hardcoded)
    A_disk  = n_rotors · π · (D_prop / 2)²
    FoM     = figure_of_merit              (calibration knob)
```

## Why `v²` and not `v³`?

Full multirotor power models include induced, blade-profile, and parasite power. In forward flight, the profile term scales with `v²`, the parasite/fuselage-drag term scales with `v³`, and induced power can decrease as speed increases.

Our simulator intentionally uses only:

```
P = P_hover + k · v²
```

### Calibration check

With `k = 13 W·(cells/s)⁻²`, `meters_per_cell = 5`, and the default `mass_kg = 1.3` (giving `P_hover ≈ 165 W` from the formula in *Mass-aware hover power* above):

| Speed | Cells/s | P (W) | vs hover |
|---|---|---|---|
| 0 (hover) | 0 | 165 | baseline |
| 9.0 m/s (cruise / our cap) | 1.8 | 165 + 13·3.24 ≈ **207** | +25% |
| 15 m/s (real F450 max) | 3.0 | 165 + 13·9 ≈ **282** | +71% ✓ matches reports |

## Sources

External references for the numbers and equations above:

- [3S LiPo voltage specification (Roger's Hobby Center)][src-3s-lipo] — confirms 11.1 V nominal = 3 cells × 3.7 V
- [Modelling Power Consumptions for Multi-rotor UAVs (Liu et al., 2022)][src-multirotor-power] — closed-form decomposition into induced + profile + parasitic terms; supports our `v²` choice for the dominant term in our speed regime

[src-f450-times]: https://www.amainhobbies.com/dji-flame-wheel-f450-quadcopter-drone-combo-kit-dji-nzm450c1/p297771
[src-3s-lipo]: https://www.hawks-work.com/pages/rc-battery-4200-xt60
[src-battery-units]: https://www.rebel-cell.com/knowledge-base/battery-terminology/
[src-battery-wh-ah]: https://www.relionbattery.com/blog/whats-the-difference-in-amp-hours-and-watt-hours
[src-power-definition]: https://en.wikipedia.org/wiki/Power_(physics)
[src-translational-lift]: http://quadcopter101.blogspot.com/2014/02/flight-school-5-power-efficiency-hover.html
[src-stanford-quad]: https://ai.stanford.edu/~gabeh/papers/Quadrotor_Dynamics_GNC07.pdf
[src-multirotor-power]: https://arxiv.org/pdf/2209.04128
[src-bauersfeld]: https://rpg.ifi.uzh.ch/docs/Arxiv21_Bauersfeld.pdf
