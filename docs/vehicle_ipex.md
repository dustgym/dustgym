# The modelled vehicle: IPEx (RASSOR is the precursor)

dustgym models the **ISRU Pilot Excavator (IPEx)** — NASA's 30 kg-class lunar
regolith excavator. The **Regolith Advanced Surface Systems Operations Robot
(RASSOR)** is its **precursor/pilot**: the TRL-4 counter-rotating bucket-drum
proof of concept that IPEx evolved from. Where this repo says "the rover", it
means IPEx; RASSOR appears only as the lineage and as the physical test platform
the IPEx team used for wheel/drum/auto-dig characterisation.

All numbers below are sourced. The provenance tags map to the modules that hold
them: `terrain_authority/ipex_specs.py` (energy/battery/geometry/mobility/drum
constants, each carrying its tag) and `terrain_authority/test_ipex_specs.py`
(arithmetic checks on these real inputs).

## Primary sources (all in `publication/nasa_lunabotics/`, gitignored)

| Tag | Paper |
|---|---|
| `[SCHULER24]` | Schuler et al., *IPEx TRL-5 Design Overview*, AIAA ASCEND 2024 (NTRS 20240008162) |
| `[WHEELTEST]` | Zhang, Schuler et al., *IPEx Wheel Testing in Lunar Regolith Simulant*, ASCE Earth & Space 2024 |
| `[BDSCALE]` | Schuler, Nick et al., *IPEx Bucket Drum Scaling Experimental Results*, ASCE Earth & Space 2022 |
| `[BUCKLES]` | Buckles, Schuler et al., *IPEx — Development of Autonomous Excavation Algorithms*, NASA KSC |
| `[CLOUD]` | Cloud, Nick et al., *The IPEx Autonomy Test-Site*, NASA KSC |
| `[MAGIC24]` | Mueller, Schuler, Reiners (Caterpillar), *IPEx Digital Twin Autonomy Challenge*, MaGIC 2024 |
| `[RASSOR13]` | Mueller et al., *RASSOR* (precursor), IEEE Aerospace 2013 |

## Vehicle specification (sourced)

| Quantity | Value | Source |
|---|---|---|
| Class / dry-mass target | 30 kg-class | `[SCHULER24]` |
| Mission | excavate up to 10,000 kg regolith, 42 kg/hr, 70 km, 11 days, lunar south pole | `[SCHULER24]` |
| Total moved (model range) | 5,000–10,000 kg | `[SCHULER24]` |
| Drive speed (nominal) | 0.30 m/s | `[SCHULER24]` |
| Obstacle traversal | rocks up to 7.5 cm | `[SCHULER24]` |
| Inclination (ConOps nominal) | up to 15° | `[SCHULER24]` |
| Slope driving test | 20° incline | `[WHEELTEST]` |
| Wheels | 4, skid-steer (no steering actuators, no suspension) | `[SCHULER24]` |
| Wheel diameter | 30.5 cm (r = 0.1524 m) | `[WHEELTEST]` |
| Skid-steer kinematic track (z, on RASSOR 2 platform) | 0.5207 m | `[WHEELTEST]` Eq.1 |
| Wheel actuator | ThinGap LSI 75-12, Harmonic Drive CSF 14-80LW (80:1) | `[SCHULER24]` |
| Bucket drums | pair, counter-rotating, single drum actuator + single arm actuator | `[SCHULER24]` |
| Drum actuator | ThinGap LSI 75-30, Harmonic Drive CSF 20-160LW (160:1) | `[SCHULER24]` |
| Drum operational speed | ~25 RPM | `[SCHULER24]` |
| Regolith per cycle | up to 30 kg (15 kg min success threshold) | `[SCHULER24]` |
| Avg regolith / drum (S/M/L) | 3.80 / 7.30 / 24.98 kg | `[BDSCALE]` |
| Drum tangential / linear-cut ratio | 8.5× | `[BDSCALE]` |
| Cut depth | ≤ 50% of scoop opening (anti-bridging) | `[BDSCALE]` |
| Predicted excavation arm load | 18.5 N·m | `[SCHULER24]` |
| Bus voltage (test sweep) | 47.6 / 53.2 / 58.8 V | `[SCHULER24]` |
| Battery (current build) | 12S Li-ion, ~30 Ah (~44 V, ~1332 Wh) | project lead 2026-06-02 |
| Cameras | 8 (≥4: stereo pair + side-facing + between-drums) | `[SCHULER24]` |
| Localisation | stereo visual odometry + lander-fiducial pose; **no GPS** | `[SCHULER24]`,`[CLOUD]` |
| Scale factor RASSOR 2 → IPEx | ~0.7 (1-D) | `[SCHULER24]` |

## Why dustgym's terramechanics matters: the excavation gap

The **official** IPEx digital twin `[MAGIC24]` decomposes as:

- **Physics — Project Chrono**: generic component models (chassis/arms/drums/
  wheels/radiator), motor+gear train with efficiency, battery state of charge;
  ground deformation via **Chrono SCM (Soil Contact Model)** with Bekker
  equations and Lunar Sourcebook soil properties. Crucially, SCM *"supports the
  bulldozing effect and realistic deformation, **but not excavation**."*
- **Autonomy + sensors — CARLA** (Unreal): 8 cameras + LED, IMU, south-pole
  lighting, craters/rocks.

dustgym is an **independent Godot + Chrono twin** (NOT the official CARLA/Unreal
LAC entry). Its differentiator is precisely the gap `[MAGIC24]` names: dustgym's
core is a **mass-exact excavation terramechanics** — Bekker pressure-sinkage,
the Janosi–Hanamoto slip ladder, Lyasko low-g, and the weight-coupling K10 chain
(drum mass → sinkage/slip/energy). It models the cut/haul/fill that SCM cannot.

## Auto-dig grounds the weight-coupling (K10)

`[BUCKLES]` describes IPEx's "auto-dig" control loop, which dustgym's drum-mass
sensing mirrors:

- **Control loop**: drum torque is the *process variable*, arm position the
  *control variable*. Setpoints on front and rear drums are equalised so the
  horizontal dig forces cancel (the counter-rotating-drum principle).
- **No force sensors**: actuator models estimate joint torque from current +
  speed; accumulated drum mass is estimated from shoulder-joint currents during
  intermittent lifts (commanded a mass, stops when reached).
- **Rock hazard / stall**: rocks ≥ 10 cm threaten autonomy — the digger reaches a
  **stall state** (fails to advance, needs intervention), or a drum grabs a
  buried rock and pulls it out. This grounds dustgym's slip-entrapment "stall"
  model and the negative-obstacle / rock-hazard masks.

## Mobility / wheel grounding, and the sim-geometry reconciliation

`[WHEELTEST]` ran 10 wheel geometries on the **RASSOR 2** platform (66 kg) fitted
with IPEx-sized 30.5 cm wheels, in the KSC Regolith Test Bed (120 t of BP-1, 8×8
×1.1 m), tracked by OptiTrack. Findings dustgym relies on:

- A **slip vs power** trade: taller/square grousers slip less but draw more power;
  cleat pattern barely matters. The chosen baseline (wheel #5) has short rounded
  grousers — "good holistically, best at nothing."
- **Slip-entrapment is real**: drawbar-pull "full slip" is when *"the robot is not
  moving forwards and instead the rotation of the wheels causes it to dig itself
  deeper in the regolith."* This is dustgym's `slip_alpha_to_slip` ladder
  entrapping near ~45°.
- RASSOR Gen-1 `[RASSOR13]` climbed a 20° slope but **failed a 30° loose mound**
  (sheared/avalanched). The planner's `max_traverse_slope_deg` default (25°) sits
  between the IPEx **nominal** 15° (`NOMINAL_SLOPE_DEG`) and that ~30° failure.

**Two selectable bodies (both physics models, at every stage).** The rover body is a
choice in the `vehicles.py` registry, each entry carrying its own geometry
(gauge / wheelbase / wheel-radius / CG) and render assets:

- **`ez_rassor`** (the default geometry): gauge 0.57 m, wheelbase 0.40 m, wheel r
  0.18 m — the **MIT EZ-RASSOR URDF** stance (`rover.py` globals + the default
  rendered mesh). This is what the wheel-track stamping and `stability.py` use when
  no vehicle is selected, so the default is **byte-identical** to before.
- **`ipex`**: wheel r 0.1524 m (sourced), skid-steer track 0.365 m (= 0.7 × the
  RASSOR-2 0.5207 m), wheelbase + CG `[CALIB]` — the flight-scale body. Its render
  mesh is the **CC0 self-authored primitive** (`scripts/gen_ipex_mesh.py` →
  `godot_sidecar/assets/ipex/`), since no public IPEx CAD exists.

The selection threads through every stage: `RoverSimEnv(vehicle=…)` (RL + tip-over
physics), `vehicles.geometry_of(name)` (the exact `stability.stability` kwargs),
`bodies.json` `_vehicles[name]` (the browser pickers), and the Godot sidecar
(`--rover-assets` / `--rover-gauge` / `--rover-wheelbase`, defaulting to the
EZ-RASSOR stance). Energy / drum / terramechanics are shared (IPEx-grounded); the
two bodies differ in **geometry → tip-over physics + render mesh**. The flight-IPEx
geometry is the principled fix to the original "is the render geometry wrong?"
question: the EZ-RASSOR stance is correct *for the EZ-RASSOR mesh*; `ipex` is the
genuine flight-scale body, authored rather than mislabeled.

Both bodies are **headless-render-verified** (Godot 4.6.3 + xvfb + Vulkan): the
default assembles the EZ-RASSOR mesh (AABB 1.83×0.66×1.70 m), and `--rover-assets
res://assets/ipex --rover-gauge 0.3645 --rover-wheelbase 0.30` assembles the CC0
IPEx primitive at its smaller flight scale (AABB 1.26×0.58×0.99 m). Evidence:
`godot_sidecar/out/body_ez_rassor.png`, `body_ipex_cc0.png`.

## BP-1 is the terrestrial test simulant, not the lunar surface

BP-1 (Black Point 1, basalt-derived mare analog) is the **Earth-g GMRO Regolith
Test Bed** simulant: bulk density ~1.75 g/cm³ compacted, shear-vane 27–32 kPa,
penetrometer 206–226 kPa (`[WHEELTEST]`,`[BDSCALE]`). dustgym's terramechanics
core models the **lunar surface** (real LOLA DEMs) with the Lunar Sourcebook
density profile (`constants.RHO_SURFACE` 1250 → `RHO_DEEP` 1920 kg/m³), so BP-1's
numbers are kept in `ipex_specs.py` (`BP1_*`) as sourced provenance for the test
bed, **not** wired into the lunar physics. A BP-1 Bekker `k_c`/`k_phi` profile is
deliberately *not* fabricated (those moduli are not in these papers).

## Autonomy, the test-site, and the LAC mapping objective (P6)

`[CLOUD]` documents the **IPEx autonomy test-site**: a 21.3 × 33.5 m (~750 m²)
enclosed bin with granular material, scattered rocks, a full-scale lander model,
and six 1400 W lights to replicate the low-sun-angle / long-shadow lunar south
pole. The lander carries **16 AprilTag fiducials** (15.24 × 15.24 cm) for
autonomous docking and localisation, and IPEx ran a multi-day continuous TRL-5
ground demonstration, downlinking telemetry to **generate a map of the worksite**
under representative comms delay/bandwidth. This grounds:

- dustgym's **worksite** scope and its **AprilTag-on-lander pose-vs-truth** check
  (12.7 mm / 7.15°, container-gated).
- dustgym's **map-relative localisation** (`localization.register_to_dem`,
  scan-to-DEM registration) — IPEx has no GPS and localises against landmarks +
  a prior map, exactly the "overlay" regime dustgym implements.

The **Lunar Autonomy Challenge** `[MAGIC24]` (JHU/APL, Caterpillar, Embodied AI)
tasks university teams to, using IPEx's digital twin: *"Map a simulated lunar
surface… develop terrain height maps and identify rocks given power and data
budgets."* That objective **is** dustgym's P6 map-channel reward (survey/build a
height + hazard map under power+data budgets) — the largest deferred PRD item,
now explicitly anchored to the official challenge spec rather than invented.

## Where this lands in the code

- `terrain_authority/ipex_specs.py` — all sourced constants (energy, battery,
  geometry, mobility envelope, drum capacity, BP-1 reference) + `spec_record()`
  provenance dump; `terrain_authority/test_ipex_specs.py` checks the arithmetic.
- `terrain_authority/stability.py` — tip-over SSA ("don't tip") on the sim rover
  geometry; `rover_env.py` adds the tip-over terminal + stability-margin obs.
- `planet_browser/mission_planner.py` — `DRUM_KG = REGOLITH_PER_CYCLE_KG` (30 kg),
  `slip_alpha_to_slip` entrapment ladder, `negative_obstacle_mask` ("don't fall
  in holes"), slope gating.
- `planet_browser/localization.py` — scan-to-DEM map-relative localisation.
