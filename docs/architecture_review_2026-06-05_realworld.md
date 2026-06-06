# dustgym — full architectural deep review + real-world-mission gap analysis (2026-06-05)

Method: 7 parallel reviewers, each deep-reading every class/def in its subsystem and **run-verifying**
(executing tests, the live server, timing the hot loop) — conserved core, terrain/DEM, RL/learning,
mission planner, autonomy/closed-loop, server/cockpit/packaging, and SLAM/perception/sensor-bridge.
Baseline confirmed: **725 tests pass, coverage ~95.7%, mypy clean (51 files), ruff-F clean.** No
synthetic-data / stub / `NotImplementedError` violations in shipped logic (the one
`NotImplementedError`, `score_pose.py:134`, is a reserved seam with a real implemented alternate path;
`synthetic_feed.py` fakes a SLAM stream but is report-only and gates nothing).

This review answers the forward question explicitly: **what does this need to conduct real-world mission
planning, optimization, SLAM, obstacle avoidance, and construction — like vehicles on Earth — with a
constant input → plan → verify → execute → reassess loop, and how do we output the model/plans?**

---

## 1. Verified bugs + honesty corrections (severity-ranked, file:line)

### HIGH
- **GeoTIFF RATIONAL (type-5) tag parse is broken — blocks ingesting arbitrary real survey maps.**
  `dem_import.py:142` over-reads (16 bytes vs 8) on type-5 tags; `_decode` (`:160`) then mismatches →
  `struct.error`. Any standard GeoTIFF carrying `XResolution`/`YResolution` (tags 282/283, which GDAL /
  tifffile / most writers emit by default) crashes `load_lola_geotiff`. The committed Haworth tile
  happens not to carry them, so the green suite never hits it: **the entire hand-rolled GeoTIFF reader
  has zero test coverage.** This is the single biggest blocker to "load an arbitrary real-world DEM."

### MAJOR — honesty corrections (these qualify claims shipped earlier today)
- **The P6 map-channel dig gate is a NO-OP beyond a counter** (`autonomy.py:183-186`). It increments
  `map_observe_more` but does **not** add a survey station, dwell, change coverage, or change energy —
  contrast the pose dig-ready gate just above (`:176-179`) which actually loops `update_pose`. So "the
  map-channel reward is **closed** into the loop" (claimed in commit `7235b13` + the PRD) is **overstated**:
  today it is *reported* + a cosmetic counter, not *acted on*. The reward is computed and surfaced; it does
  not yet change behaviour. (Correctness note: I shipped that "closed into the loop" wording this morning;
  it should read "computed + surfaced; gating is a counter, not yet an action.")
- **The per-leg pose "fix" is self-referential** (`autonomy.py:192,196`): `predict` sets the believed pose
  to `telem["new_pose"]` (= commanded site = truth in sim), then `update_pose` fuses that same value, so the
  measurement always equals the estimate. In self-simulation it is harmless (belief == truth); on real
  telemetry this seam must take an **independent** fix or it can never correct drift — it only collapses σ.
  So "per-leg map/landmark fixes bound the dead-reckoning drift" is structurally a confidence injection, not
  a correcting measurement, until a real independent pose arrives.
- **UI copy overstates execution.** The Metrics pane empty-state says "press ▶ Execute + watch … to
  **stream the closed-loop telemetry**" (`index.html:273`), but "execute + watch" is a client-side
  `requestAnimationFrame` **replay** of the static `timeline` array `/plan` already returned — not a stream,
  not measured telemetry. (The Perception pane copy is scrupulously honest by contrast — align Metrics to it.)

### MED / MINOR
- **`bodies.params_for_body('moon')` friction 35° vs `constants.PHI` 37°** (`bodies.py:54` vs
  `constants.py:77`) — two "lunar" sources of truth disagree on φ (benign now; a real split).
- **`slip_sinkage_equilibrium` reports non-monotone `sinkage_m`** across the two entrapment branches
  (`slip.py:121-128`): a steeper entrapped slope can report shallower sinkage (the `entrapped` flag + slip
  are correct; only the sinkage telemetry is an artifact).
- **`Dust/WorkSite-v0` default is synthetic terrain.** `registration.py:94` registers it with `{}` →
  `worksite_env.py:34` `_bumpy_base = rng.random()*roughness`; the real-Haworth path works but needs
  `bundle_dir=` (only the demo scripts pass it). A legitimate procedural generator, but the out-of-the-box
  gym env is toy terrain, not real LOLA.
- **Doc drift:** `dem_import.py:13-14` claims "no pyproj"; pyproj 3.7.2 **is** installed and
  `planet_browser/dem_import.py` already uses it. The two DEM-ingest paths are **forked** (polar in
  `terrain_authority`, cylindrical-reproject in `planet_browser`).
- `_recharge` doesn't account the return-to-charger drive energy (`autonomy.py:163`, documented); energy σ
  grown from nominal not true spend (`autonomy.py:198`, defensible). Both PRD-consistent simplifications.

No mass-conservation, NaN, or correctness defects in the conserved core. Mass drift is **0.0** (not just
~1e-16) through cut/dump/sandpile/sinter; `physical=True` is the default on the drive loop; the slip ladder
genuinely entraps (~40°) and recovers; the Earth soil+gravity override works end-to-end and is the Bekker
model's *most* valid regime. The per-step physics is **sub-ms: 0.64 ms on 256² (1571 Hz), 0.12 ms on 64²
(8064 Hz)** — real-time capable.

---

## 2. Real-world mission capability map (where we are vs what "vehicles on Earth" needs)

| Capability | Today (run-verified) | Gap to real-world autonomy |
|---|---|---|
| **Planning** | Action-level: consumes pre-decomposed cut/fill/sinter orders; 8 volume-balanced structure templates. | **Goal/tolerance-driven decomposition** — generate the order set that *achieves* "pad to ±2 cm / grade to X% / bear Y kPa"; today it only **checks** flatness post-hoc and a feasible plan can still miss it. No lift/compaction-pass/surface-finish specs. |
| **Optimization** | 7 sequencers (nearest→LK, brute≤7 exact, Held-Karp≤16 exact-on-distance) + weighted multi-objective + Pareto; honest exactness ceilings. | Cannot express **temporal/risk** constraints: deadlines, **op-windows** (sunlit/thermal — *reported* in endurance, never a *constraint*), precedence-with-lag (wait for a berm to settle), risk/variance-penalized routing (the belief carries σ but routing uses the mean), resource calendars (one shared charger, no contention). |
| **SLAM** | AprilTag fiducial pose channel (12.7 mm/7.15°, **container-gated, not reproducible on this checkout**); `rover_localize.py` math verified (4e-16 m round-trip). | **No continuous SLAM in any live loop.** rtabmap (`slam_bringup.launch.py`) is the only real SLAM and is **never run** (needs the ROS2 container + multi-frame MCAP from a Godot egress that's absent). `synthetic_feed.py` fakes the pose stream (report-only, gates nothing). Nothing localizes the rover live today. |
| **Obstacle avoidance** | Static **keep-out circles** + slope costmap → impassable cells; least-cost Dijkstra detours (verified +27.6% around a keep-out). | **No sensor-based obstacle DETECTION** (no rock detector produces `observed_rocks`; clasts are read from authored truth, never perceived); no **dynamic/discovered-obstacle replan**; no **continuous haul-path deconfliction** (multi-vehicle is site-exclusive but corridors can cross); no polygonal/time-varying hazards. |
| **Construction** | Conserved, mass-exact cut/haul/dump/grade/fill + sinter (gated); as-built flatness measured on the **real** terrain (fails a uniform cut on a slope, correctly). | Tolerance-driven order generation (above); berm-profile/bearing acceptance; repose/compaction enforcement; tool-wear; contact-force settling (the Chrono oracle, host-gated). |
| **Real-time reassess loop** | A **real** closed loop — plan→execute→Kalman-estimate→recharge-replan — but in **self-simulation** (the conserved model is both world and model). Sub-ms physics. | **No live ingress** (no cmd_vel-out / odom-imu-camera-in); **no streaming output** (no WS/SSE — `/plan` is one synchronous response); **reassess fires only on recharge** (not on discovered obstacles, pose drift, or map gaps); **verify runs once** at plan time, not continuously. |

**The single clean seam to make execution real is `autonomy.execute_leg` (`autonomy.py:117`, one call site
at `:188`)** — input `(belief, leg, dem, params)`, output a fixed telemetry dict. Swap its body for a
telemetry reader and the loop drives a real rover. But it is a **per-leg return-value** seam, not a
per-tick async stream, so real-time also means converting the loop from leg-granular-synchronous to
tick-granular-asynchronous + adding the command-out / telemetry-in channels (the PRD's P7).

**What already exists and helps:** the numpy world has a closed drive loop (`drive.drive_step` always
`physical=True`) and a reverse command seam (`drive.poll_cmd_vel` reads a `{v,omega}` JSON file, safe-stop
on missing) — but it is **not wired** to the server, to ROS, or to the Godot renderer. The two frozen
file-mediated contracts (Seam-1 state fields → Godot; Seam-2 `sensors.json`+PNG → ROS2) are real and
parallel-buildable. The self-optimizing energy loop (`self_optimizing.py`) is a genuine
execute→observe-gap→learn→re-price reassess mechanism (held-out error 12.4%→<1%), and is the cleanest
learned→deployed artifact in the repo (3 polyfit coefficients).

---

## 3. How do we output the model / plans? (the central question)

Today the planner emits a **rich machine-readable JSON envelope** (`/plan`: `totals`, `validation` incl.
as-built RMSE, `timeline` frames, `endurance`, `autonomy`, `perception`) + a human **PDF/markdown** report
+ the **state-field contract** (`io_fields` `.rf32`/`.r8` + `metadata.json`, language-neutral, real-time
friendly) + (for learning) SB3 `.zip` / torch `state_dict` written only to `/tmp`. **What it does NOT have
is a machine-EXECUTABLE plan IR** — the `timeline` is a *forecast for animation*, not a command stream;
there is zero `cmd_vel`/`Twist`/`nav_msgs`/`to_ros`/`export_plan` anywhere. A real rover cannot consume the
output; it would have to reverse-engineer twists from animation frames.

The answer is a **layered output strategy**, build-order top to bottom:

1. **Plan IR (the keystone new artifact).** A versioned, serializable, ordered list of **typed actions** —
   `GoTo(x,y) · Excavate(site,depth,mass) · Haul(from,to,loads) · Dump(site,target_h) · Grade · Recharge` —
   each with **preconditions** (battery ≥ X, drum ≤ cap, map_coverage ≥ gate), **expected duration/energy ±
   tolerance**, and the **precedence DAG**. The data already exists internally (`trips` + `per_trip` + `tl`);
   it needs only a schema + emitter. Add `schema_version` + `plan_id` (UUID) so a consumer can pin a contract
   and correlate a plan to an execution. JSON now; protobuf/flatbuffer if bandwidth matters.
2. **ROS lowering.** Plan IR → `nav_msgs/Path` waypoints + per-leg `geometry_msgs/Twist` budgets + dig/dump
   as action-server goals. `execute_leg` already computes the per-leg drive/slip truth — the lowering point
   exists. For the sim, lower to the existing `drive.poll_cmd_vel` twist-file seam (no new physics needed).
3. **Map / world-model output = the existing state-field contract** (`io_fields`). The conserved authority's
   terrain, density, disturbance, state-label rasters + `metadata.json` (`world_bounds_m`, gravity,
   provenance, quadtree sidecar) are already the real-time-friendly, shader-samplable, language-neutral map
   model. Heightmap is derived (`datum + mass/ρ`), never stored — so the map is always self-consistent.
4. **Learned-model output.** The conserved authority is *code*, never learned, so only the small command
   policy (an SB3 `.zip` / a distilled `torch state_dict`) + the 3-coefficient energy model need serializing
   — add a checkpoint export + versioning (today they go to `/tmp`, uncommitted) and a `load_policy()` /
   inference seam in the server (none exists). The conserved-vs-learned split keeps the artifact tiny and the
   reward unhackable.
5. **Streaming telemetry + reassess deltas (the real-time I/O surface).** Add `GET /plan/stream` (SSE) or
   `WS /telemetry` that emits the `legs[]`/`timeline`/`belief` frames **as the loop produces them** (today
   they're collected then returned in one shot — `MP.build_timeline` / `run_closed_loop` just need to become
   generators behind a `StreamingResponse`); a `POST /cmd_vel` (or `/step`) ingress wired to the existing
   `drive.py` integrator (the physics seam exists; the server has no route to it — the biggest missing input
   piece); a `POST /replan {plan_id, current_state}` that re-prices remaining orders from a supplied rover
   state instead of re-POSTing the whole mission; and a **re-plan delta** (which legs changed + *why* —
   discovered obstacle / energy overrun / flatness miss — + new-vs-old ordering) so an operator or executive
   can accept/reject.
6. **Execution log.** A `POST /telemetry/{plan_id}` ingest + stored **measured-vs-planned residuals**,
   turning "execute + watch" from a forecast replay into a real plan-vs-actual viewer.

**One line:** the planner's *math* is real and the JSON is rich, but its output is built to be **read**, not
**executed** — the work is (a) a versioned executable **Plan IR** + a ROS lowering, (b) wiring the
already-existing physics command/telemetry seams up to a **streaming** server surface, and (c) versioned
checkpoint export for the (small) learned parts. The map/world model already has its contract.

---

## 4. Sequenced roadmap to real-time, real-world mission execution

**P0 — make the loop actionable + honest (small, mostly wiring):**
1. Fix the **map-channel gate** so it acts (dwell/add a survey station + re-score, or feed a replan), not just
   counts; re-word the "closed into the loop" claim until it does. Fix the **self-referential pose fix** to
   take an independent measurement field. Re-word the Metrics "stream" copy.
2. Fix the **GeoTIFF type-5 parser** + add a real-TIFF test → ingest arbitrary survey DEMs; unify the forked
   polar/cylindrical ingest; correct the stale "no pyproj" doc.
3. Define the **Plan IR** (typed actions + DAG + preconditions + tolerances + `plan_id`/`schema_version`) and
   emit it from `/plan`; add the ROS / twist-file lowering.

**P1 — close the real loop:**
4. **Streaming I/O**: SSE/WS telemetry out + `POST /cmd_vel`/`/step` in (wire to `drive.py`) + `POST /replan`
   from current state + the re-plan delta.
5. **Continuous verify + event-driven reassess**: run validate/coverage per-leg on the *observed* map; add
   replan triggers for discovered obstacles, pose-σ breach, map-coverage gaps (today only energy fires).
6. **Sensor obstacle detection** → dynamic keep-outs (the `obs_map_producer` back-projects points; add rock
   segmentation → costmap injection) + continuous haul-path deconfliction for multi-vehicle.

**P2 — fidelity + host/data-gated:** continuous SLAM (stand up the ROS2 container → `bag_seq_writer` →
rtabmap → live `map→base_link` TF, replacing dead-reckoning); the dense map-channel RMSE tier (COLMAP/render,
CUDA-gated); the Chrono SCM force oracle (PyChrono-with-vehicle host); goal/tolerance-driven order generation;
op-window/deadline/risk constraints in the objective grammar; checkpoint export/versioning for the learned
policy; drivetrain η.

**Bottom line:** dustgym is a genuinely trustworthy, conserved, sub-ms **offline batch construction planner**
with an honest closed loop that runs **in self-simulation**, a rich machine-readable JSON output built to be
*read*, and well-engineered but **gated** SLAM/render/ROS contracts. To "conduct a real mission like vehicles
on Earth" it needs three things, in order: (1) an executable **Plan IR** + ROS lowering (how plans are
output), (2) **streaming I/O** wiring the already-built physics command/telemetry seams to the server, and (3)
the **perception loop** closed for real (live SLAM + sensor obstacle detection), which is the host/container-
gated tier. The physics, the optimization, the conserved map model, and the energy grounding are real today;
the gap is execution plumbing and live perception, not the science.
