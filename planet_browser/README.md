# planet_browser — lunar build planner front-end + mission-control report

The product face of `foss_ipex`: load a body (Moon / Mars / …), queue build orders, and the planner
sequences + optimizes them under real terramechanics + battery + time, validates the plan on the
conserved authority, and returns a 2-3 page mission-control report. This is the SimCity-style planning
loop, not a thesis. Standalone; imports the conserved Tier-2 core from `../roversim/terrain_authority`.

**What the planner does** (all grounded, no synthetic data):
- **Terrain-aware + authority-validated** — sites are slope-gated on the real LOLA DEM; the plan is
  executed on the conserved `column_state` for mass-exact feasibility (drift 0).
- **Pluggable algorithm × objective** — `nearest / greedy / two_opt / or_opt / lk / brute / held_karp`
  + an **`auto`** dispatcher (brute ≤7 trips, Held-Karp-seed + LK-polish 8–16, LK above), optimizing
  any of `time / energy / power / distance / charges / mass` or a weighted multi-objective; `/compare`
  ranks them with a Pareto frontier.
- **Precedence (I9)** — order-level "before → after" constraints honored by every algorithm (SOP-aware).
- **Hazard routing + slip energy** — hauls route around craters on a slope costmap (Dijkstra), drive/haul
  cost is slip-adjusted (`135/(1-slip)`) with exact `m·g·Δh` gravity-lift; **endurance/range** readout
  (per-sortie km, DEM reachability, body-correct day/night timescale).
- **Closed-loop autonomy (P12)** — `autonomy.py`: an AutoNav-style belief estimator (Kalman, uncertainty)
  + a plan→execute→estimate→**replan** controller that manages the battery from the estimate.

## Run

```bash
# from this directory, with the runtime venv (deps in requirements.txt: numpy + matplotlib + scipy + pyproj)
PYTHONPATH=. /mnt/projects/07_runtime_system/venv/bin/python server.py --port 8770
# then open http://127.0.0.1:8770/ in a browser
```

Non-polar maps: `dem_import.py` reprojects a cylindrical (lat/lon) DEM product to the local metric grid
via `pyproj` (e.g. LOLA `ldem_4`); see `fixtures/ldem4_equator_*` for a tiny real equatorial fixture.

In the browser: pick a body, pan/zoom/tilt, add build orders to the **BUILD QUEUE** (kind, x/y in
meters, footprint, depth), then **Plan mission** to optimize the sequence and open the report PDF. The
**DRUM SENSOR** panel (bottom-left) shows the drum-fill inferred from motor current (ICE-RASSOR, no load
cell) with an offload decision; the **sensor noise** checkbox toggles seeded noise (off = deterministic).

A plan can also be generated headlessly:

```bash
python mission_planner.py            # writes reports/<date>_mission_plan.pdf + .md (demo mission)
```

## Pieces

| File | What it is |
|---|---|
| `index.html` | CesiumJS browser (NASA Solar System Treks tiles) + build-order queue + live regolith estimate. |
| `server.py` | Stdlib `http.server` (no framework). Serves the front-end + `bodies.json` + `/reports/`; `POST /plan` runs the planner and returns the report URL. |
| `mission_planner.py` | Cut-fill balancing → **pluggable sequencer × objective** (`optimize_sequence`: nearest/greedy/2-opt/Or-opt/LK/brute/Held-Karp/auto; `compare_algorithms` + Pareto) → terrain-aware + authority-validated (`validate_plan`) → slip-adjusted hazard routing → endurance/range → battery-aware mid-task recharge → 3-page PDF + markdown report. Grounded in `ipex_specs` + `bodies.json`. |
| `autonomy.py` | Closed-loop autonomy (P12, the AutoNav model): `Belief` + Kalman `estimator` (`predict`/`update_*`), `execute_leg` (slip-adjusted true telemetry), `run_closed_loop` (plan→execute→estimate→replan + reserve-aware recharge). Runs in the conserved-model sim. |
| `dem_import.py` | Reproject a non-polar (cylindrical lat/lon) DEM product to the local metric grid via `pyproj` (P4); real LOLA `ldem_4` fixture in `fixtures/`. |
| `gen_bodies_json.py` | Generates `bodies.json` (per-body terramechanics + an `_ipex` energy block) from the `.py` source of truth (`terrain_authority/bodies.py` + `ipex_specs.py` + `constants.py`). Re-run after editing those. |
| `bodies.json` | Generated, read-only mirror (the browser can't import `.py`). |
| `test_mission_planner.py` | The P1 round-trip tests: the queue→Mission adapter, a queued mission writing a real PDF, the live `/plan` endpoint, and the sinter gate. |

## The `/plan` contract

`POST /plan` with a build-order queue:

```json
{ "name": "South-Pole Site", "body": "moon", "charger": [0, 0],
  "orders": [
    { "action": "Level pad", "kind": "cut",  "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04 },
    { "action": "Build berm", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10 }
  ] }
```

Optional fields: `"algorithm"` (`auto` default · `nearest/greedy/two_opt/or_opt/lk/brute/held_karp`),
`"objective"` (`time` default · `energy/power/distance/charges/mass` · or a weighted `"time:0.5,energy:0.5"`),
and `"precedence": [["Grade road","Build berm"], …]` (before→after by order action).

Returns `{ "ok": true, "pdf": "/reports/...pdf", "md": "...", "totals": {...}, "validation": {...},
"timeline": {...}, "endurance": {...} }`, or `400` for an unknown body / malformed order / sinter order
(sinter is a real conserved primitive but **gated off** until its `[CALIB]` energy/density are grounded —
`terrain_authority.constants.SINTER_ENABLED`). `POST /compare` runs every algorithm and returns them
ranked by the objective with a Pareto flag.

Coordinates are a **local site frame in meters** (charger at `0,0`); the globe pick selects the site,
the queue places orders around it. There is no fabricated lat/lon to meter projection.

`POST /sense` with `{ "true_mass_kg": 25, "noise_frac": 0, "capacity_kg": 30, "seed": 0 }` returns the
drum-fill sensing for a given true mass: `{ "current_a", "inferred_kg", "uncertainty_frac", "lower_kg",
"upper_kg", "offload", ... }`. `noise_frac` is the **noise toggle** (0 = off, deterministic; the seeded
Gaussian is reproducible). Drum mass is inferred from the motor-current observable (the 2020/2021 RASSOR
had no load cell); see `terrain_authority/rassor_mass_model.py` (NTRS 20210022781).

## Lunar DEM (work-area inset + expansion)

The Moon **WORK AREA** inset is the real LOLA polar 5 m Haworth tile already in the sim
(`roversim/samples/lunar_dem/haworth_10km_5m`, south-polar stereographic), served at `/dem/hillshade.png`
and auto-shown on Moon. To extend coverage, the sim's `terrain_authority/dem_import` ingests standard
LOLA products:
- **SLDEM2015** (PGDA product 54, LOLA + SELENE TC merge): **±60° only** (no pole), ~60–100 m/px, FLOAT IMG
  / JPEG2000 at 128/256/512 ppd — `imbrium.mit.edu/DATA/SLDEM2015/`. Good for mid-latitude sites.
- **South-pole LOLA DEM** (PGDA product 66, "A New View of the Lunar South Pole from LOLA"): the polar
  complement that **does** cover Haworth / the construction work area.
Neither is web-tiled (no WMTS), so they are ingested as DEMs (sim) or rendered to hillshade (inset), not
draped as globe tiles.

**Cite (SLDEM2015):** Barker, M. K., Mazarico, E., Neumann, G. A., Zuber, M. T., Haruyama, J., Smith, D. E.,
"A new lunar digital elevation model from the Lunar Orbiter Laser Altimeter and SELENE Terrain Camera,"
*Icarus* 273 (2016) 346–355. https://doi.org/10.1016/j.icarus.2015.07.039

## Grounding

All constants come from the `.py` source of truth: IPEx energy/battery from `ipex_specs.py`
(Schuler et al., ASCEND 2024, NTRS 20240008162; 12S/30Ah pack), per-body terramechanics from the
bodies sysrev (`bodies.py`). The recharge power, sinter-head power, and reserve fraction are tagged
`[CALIB]`. No synthetic data.
