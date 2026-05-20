# F450 reference numbers

Real-world F450 specs and community-measured flight times, used to sanity-check the simulator's energy and motion models. The F450 is a DIY frame kit, so most "specs" depend on which battery, motors, and props the builder picks. **Hawk's Work** (the build this project uses) and **DJI Flame Wheel** are functionally the same airframe — DJI released the original ~2012 and Hawk's Work / Readytosky / others ship clones at lower prices with identical wheelbase, frame weight, and motor / battery / prop classes. Most published flight-time data is from DJI users because the platform is older and more popular; that data applies equally to Hawk's Work builds.

## This project's drone (Hawk's Work F450 build)

[Hawk's Work F450 Drone product page](https://www.hawks-work.com/pages/f450-drone):

| Component | Spec | Mass |
|---|---|---|
| Frame | Hawk's Work F450, 450 mm wheelbase, clone of DJI Flame Wheel F450 — same airframe class | 280 g |
| Motors | 4× A2212 920 KV brushless | 52 g each; 4× = 208 g |
| ESCs | 4× 20 A brushless with 5 V / 1 A BEC | 17.9 g each; 4× = 71.6 g |
| Battery | 11.1 V 3S LiPo, 4200 mAh, 25 C, XT60 | 330 g |
| Props | 9450 self-tightening (CW + CCW) | 10 g each; 4× = 40 g |
| Flight controller | Pixhawk 2.4.8, PX4 autopilot | 15.8g |
| Camera | e-con Systems **STEEReoCAM Nano**, **forward-facing fixed-mount** (no gimbal — drone yaws to redirect). 2× OV2311 stereo, 4.3 mm f/2.8 M12 lens, HFOV 54° / VFOV 49.5°, 0.95–8 m stereo depth range, on-board 6-axis IMU. | 159 g |
| Companion | NVIDIA Jetson Nano | 178 g |
| Total mass | - | 1.3 kg |

## Camera (STEEReoCAM Nano) — used to derive the sensor wedge

Full datasheet + lens datasheet are in [`docs/camera_specs/`](camera_specs/). Key numbers, all cited from those documents:

| Parameter | Value | Source |
|---|---|---|
| Lens focal length | 4.3 mm | Lens Datasheet §5 |
| Aperture | f/2.8 | Lens Datasheet §5 |
| **HFOV** | **54°** | Lens Datasheet §5 |
| **VFOV** | **49.5°** | Lens Datasheet §5 |
| DFOV | 77.9° | Lens Datasheet §5 |
| Image sensor | 2× OmniVision OV2311, 1/2.9", monochrome global shutter | Datasheet §3.2 |
| Sensor active area | 4.857 × 3.956 mm | Datasheet §3.2 |
| Pixel size | 3 µm × 3 µm | Datasheet §3.2 |
| Per-eye resolution | 1600 × 1300 (2 MP) @ 30 fps | Datasheet §1 |
| Stereo baseline | 100 mm | Datasheet §1 (mechanical drawing) |
| **Stereo depth range** | **0.95 – 8 m** | [e-con product page / FAQ](https://www.e-consystems.com/nvidia-cameras/jetson-agx-xavier-cameras/stereo-camera.asp#:~:text=Accurate%20depth%20sensing%20with%20a%20flexible%20range%20between%200.95m%20to%208m)|
| On-board IMU | 6-axis (3D accel + 3D gyro) | Datasheet §3 |
| Lens mount | M12 (S-mount), pre-calibrated lens pair | Datasheet §3.1 |

### Camera orientation: forward-facing

The STEEReoCAM Nano sits **rigid on the front of the drone, looking forward**. There is no gimbal. To point the camera at something off-axis the drone has to physically reorient — yaw to swing the camera left/right, pitch to tilt up/down, roll for lateral. In our 2D top-down simulator we model **yaw only**; pitch and roll are abstracted as "optimized for whatever the controller is doing." For real flight, controllers will need explicit pitch control to inspect the ground at low altitudes (the camera looks at the horizon during level hover).

### What the camera sees in the XY plane (forward wedge)

Because the camera is forward-facing and we work in 2D, the relevant geometry is the **horizontal wedge** the camera sweeps through at altitude. From the HFOV of 54° and stereo depth range 0.95 – 8 m:

```
wedge half-angle = HFOV / 2 = 27°
wedge range      = 8.0 m   (stereo depth ceiling — beyond this, depth is unreliable
                            but visual sensing still works; we use 8 m as a
                            conservative coverage range tied to depth quality)
wedge area       = ½ · r² · θ = ½ · 8² · (54° in rad) = ½ · 64 · 0.942 ≈ 30 m²
```

So at any moment a stationary drone covers ~30 m² of forward area. To scan a different direction the drone yaws (`max_yaw_rate ≈ 86°/s` — full 360° scan in ~4.2 s). To cover new ground the drone translates and the wedge slides forward along its heading.

### Why specifically `sensor_range = 1.6 cells` (= 8 m)?

Two compounding choices:

1. **`meters_per_cell = 5.0`** was picked so the discrete grid resolves features at sensor scale — see [`simulation-model.md` → World scale](simulation-model.md#world-scale). It also keeps the energy-model calibration (`motion_coeff_w_per_v2 = 13`) intact.
2. **`sensor_range = 1.6 cells × 5 m/cell = 8 m`** comes directly from the camera's stereo depth ceiling. Beyond 8 m the depth quality drops below the disparity-resolution threshold, so we treat 8 m as the radial coverage limit. The HFOV (54°) is verbatim from the lens datasheet.


## Hover time

Direct calculation, `t = E_pack / P_hover`:

```
E_pack    = 11.1 V × 4.2 Ah = 46.6 Wh = 167,832 J
P_hover   = (m·g)^1.5 / (FoM · √(2·ρ·A_disk))     ← mass-aware, see below
        = (1.3 · 9.81)^1.5 / (0.4168 · √(2·1.225·0.1791))
        ≈ 165 W                              (for the default 1.3 kg build)
t_full    = 167,832 / 165 = 1017 s = 17.0 min    (pack hard-empty)
t_cutoff  = t_full × 0.72            = 12.2 min  (10 V cutoff, ~28 % reserve)
```

**Sanity check vs community data.** Published F450 hover times for 3S packs span ~10–18 min depending on payload (1.0–1.8 kg) and capacity (2.2–5.0 Ah): heavier gimbal builds (~1.8 kg + 5 Ah) report ~15 min, lighter no-gimbal builds (~1.0 kg + 2.7 Ah) report ~12 min ([DJI forum][src-dji-forum-flight-time], [DroneVibes][src-dronevibes-battery-size], [HeliFreak][src-helifreak-battery-choices]). Our 1.3 kg + 4.2 Ah config interpolates to 14–17 min; the model's 17.0 min full-pack number sits at the top of that range, and the 12.2 min cutoff sits ~3 min below the lower bound on purpose, as flight reserve.

## Default `max_speed` — derived from rotor induced velocity

The simulator's default `DroneConfig.max_speed = 1.8 cells/s = 9.0 m/s` is **derived from physics for our 1.3 kg build**. The anchor is the rotor's induced velocity at hover (`v_induced`), which sets the scale for forward-flight efficiency: maximum-range cruise is empirically observed at roughly `1.4–1.5 × v_induced` .

```
v_induced(m) = √(T / (2·ρ·A_disk))     where  T = m·g

For 1.3 kg + F450 (9450 props):
  A_disk    = 4·π·(0.1194)²        = 0.179 m²
  T         = 1.3 · 9.81           = 12.75 N
  v_induced = √(12.75 / (2·1.225·0.179))  ≈ 5.4 m/s

rough multirotor estimate:
  v_max_range ≈ 1.48 × 5.4 m/s     ≈ 8.0 m/s
  v_max_endurance ≈ 0.87 × 5.4 m/s ≈ 4.7 m/s   (min total power)

  → picked 9.0 m/s (= 1.67 × v_induced, ~12 % above max-range optimum
                    for some headroom, rounded to a clean 1.8 cells/s)
```


## Cruise (forward flight)

Direct calculation from the energy model `P = P_hover + k·v²`, with `P_hover` mass-aware (see *Hover time* above):

```
v_cruise         = 9.0 m/s = 1.8 cells/s         (default max_speed; see above)
k                = 13 W·(cells/s)⁻²              (profile-power adder)
P_cruise(1.3 kg) = 165 + 13·1.8² = 207.12 W
t_full           = 167,832 / 207.12 = 810 s = 13.5 min
t_cutoff         = t_full × 0.72 = 9.75 min      (10 V cutoff, ~28 % reserve)
range_cutoff     = 9.75 min × 60 · 9.0 m/s = 5.27 km
```

[src-f450-times]: https://www.amainhobbies.com/dji-flame-wheel-f450-quadcopter-drone-combo-kit-dji-nzm450c1/p297771
[src-multirotor-power]: https://arxiv.org/pdf/2209.04128
[src-3s-lipo]: https://www.hawks-work.com/pages/rc-battery-4200-xt60
[src-dji-forum-flight-time]: https://forum.dji.com/thread-4327-1-1.html
[src-dronevibes-battery-size]: https://www.dronevibes.com/forums/threads/dji-f450-battery-size.12486/
[src-helifreak-battery-choices]: https://www.helifreak.com/showthread.php?t=406895
[src-dji-translational-lift]: https://forum.dji.com/thread-91091-1-1.html
[src-ardupilot-power-feedback]: https://discuss.ardupilot.org/t/feedback-needed-your-power-consumption-in-hover-climb-cruise-descent-etc/32200
[src-translational-lift]: http://quadcopter101.blogspot.com/2014/02/flight-school-5-power-efficiency-hover.html
[src-bauersfeld]: https://rpg.ifi.uzh.ch/docs/Arxiv21_Bauersfeld.pdf
