# Plan (a): Load-bearing terramechanics + closed drive loop → Tier-2 RL env

**Scope:** option (a) from the 2026-06-01 review — make terrain *response* real (pressure-sinkage +
slip-sinkage, "slippy dirt"), close the drive loop with a `cmd_vel` integrator, and wrap a Gymnasium
environment with an honest control reward, calibrated against the Chrono SCM oracle already in the repo.
**Out of scope:** Tier-3 granular excavation forces (drum torque/throughput — needs Chrono::GPU DEM that
does not exist here), perception-in-the-loop RL (needs an observed-map producer that does not exist).

**Repo:** `/mnt/projects/foss_ipex/roversim/` @ HEAD `9bf20f9`. **Owner:** John McCardle (Aaron hosting).
All work on a **feature branch**, keep John's authorship and his honesty-tag conventions
(`[FIXED]/[CALIB]/[UNKNOWN]`), coordinate before pushing anywhere.

> **Progress (2026-06-01):** branch `feat/load-bearing-terramechanics`. **Phase 1 COMPLETE** —
> 0.2 ✅ (IPEx mass 30 kg-class), 1.2 ✅ `636f062` (Bekker solver + JSON config), 1.3 ✅ + 1.4 ✅ `49c06bf`
> (wired opt-in into `four_wheel_pass`/`conform_pose`; Lyasko low-g). 22/22 new + 19/19 legacy green;
> end-to-end chain mass-exact. **Phase 2 COMPLETE** `418858e` — `slip.py` slip-sinkage ladder (traction
> budget, Janosi-Hanamoto, slip inversion+entrapment, compaction resistance, runaway/recovery); slip a
> real sinkage driver in `four_wheel_pass`. slip 12/12 + terramech 24/24 + legacy 19/19 green; validated
> runaway past ~45° + back-off recovery. Phase 0.3 oracle ⏸ deferred to euclid (FIX-1/FIX-2 in
> `DEFERRED_FIXES.md`). **Phase 3 COMPLETE** `f0b7798` (closed loop:
> `rover.step_pose` + `drive.py` + `drive_cmd.py`; ROS/policy drives by twist, slip closes the loop —
> flat full-traversal, 55° entrapment; drive 11/11 + 47 pytest + 19/19). **Phase 4 COMPLETE** `cd54bc1` —
> `RoverSimEnv` (gym-optional) over `drive_step`; honest control reward; domain randomization from the
> sourced envelopes; core 9/9 (gym.Env path = FIX-3). **In-repo plan (Phases 1–4) DONE.** 58 pytest +
> 19/19. Remaining external/deferred: FIX-1/FIX-2 (euclid oracle), FIX-3 (gymnasium env_checker), optional
> Phase 5 (learned world model). Live table in `PRD.md`.

---

## Why this is the right investment (from the repo's own analysis)

`docs/lac_reimplementation_eval.md:22-25` is explicit: the *official* LAC twin has only **cosmetic
deformation** — rocks immovable, no mass conservation, no excavation (the dig task is disabled in the
2024-25 mapping-only edition). foss_ipex's distinctive contribution is exactly **the layer LAC omits:
mass-conserving, excavation-grade terramechanics**. Making the moduli load-bearing is not gold-plating;
it is the project's differentiator and the prerequisite for any RL where terrain response matters.

---

## Hard design constraints (must not break)

1. **Backward compatibility.** `wheel_pass`/`four_wheel_pass` default to `compaction=0.12` and the green
   suite + the `tread_track` scene depend on that exact behavior (`rover.py:50,149`). The force path is
   **opt-in and additive** (compute compaction from load *only when asked*), mirroring how John already
   added `four_wheel_pass` alongside `wheel_pass` "kept intact because tests depend on it"
   (`rover.py:20-24`). The 19/15 suite must stay green as a regression gate.
2. **Mass conservation stays exact.** Every new op remains a conserved transform of `mass_areal`; height
   re-derives via `column_state.derive_height()` (`column_state.py:98-104`). Sinkage maps to a
   density/thickness change, never a free height edit. Assert it (the existing `invariant-1`/`invariant-2`
   pattern, `tests.py:78-83`).
3. **Determinism preserved.** No new unseeded RNG. The authority is currently RNG-free in the dynamics
   (verified); domain randomization (Phase 4) uses explicit seeds only.
4. **TDD, real data only.** Test first (must fail) → implement → run → validate against the **real Chrono
   oracle table**, not a synthetic distribution. Tests live in a persistent module, not inline.

---

## Real parameters this plan stands on (verified in `constants.py`)

| Symbol | Value | Line | Tag | Role here |
|---|---|---|---|---|
| `g` | 1.62 m/s² | `:26` | [FIXED] | weight-on-wheels |
| `RHO_SURFACE` / `RHO_DEEP` | 1300 / 1920 kg/m³ | `:55,59` | [CALIB] | density bounds (compaction cap) |
| `Z_T` | 0.12 m | `:64` | [CALIB] | loose-layer thickness |
| `COHESION` (c) | 170 Pa | `:73` | [CALIB] | Bekker cohesion |
| `PHI` (φ) | 37° | `:77` | [CALIB] | internal friction |
| `N_SINKAGE` (n) | 1.0 | `:96` | [CALIB] | sinkage exponent |
| `K_C` | 1400 | `:100` | [CALIB] | Bekker cohesive modulus |
| `K_PHI` | 820 000 | `:105` | [CALIB] | Bekker frictional modulus |
| `K_SHEAR` (K) | 0.018 m | `:109` | [CALIB] | Janosi-Hanamoto shear modulus |
| `SLIP_C1` / `SLIP_C2` | 0.4 / 0.3 | `:114,115` | [UNKNOWN] | slip-sinkage θ_m=(c1+c2·s)·θ_f |
| `THETA_R` | 35° | `:125` | [UNKNOWN] | repose (already live) |

Wheel geometry is real and known: radius 0.18 m, gauge 0.57 m, wheelbase 0.40 m, contact width 0.18 m
(`rover.py:111-112,298`). **Missing real input: rover mass / weight-on-wheels** — see Phase 0.

---

## Phase 0 — Branch, sourced inputs, calibration oracle (no new physics yet)

**0.1 Branch.** `git checkout -b feat/load-bearing-terramechanics` in `roversim/`.

**0.2 Source the one missing real number — RESOLVED (2026-06-01).** A load-driven sinkage needs
weight-on-wheels, and the repo had none IPEx-faithful (`docs/chrono_bringup_log.md:216` uses a 25 kg
test cylinder; `docs/ezrassor_assets.md:128,200` warns the EZ-RASSOR URDF masses are "toy demo values,
not IPEx-faithful — reuse topology+dimensions only"). **Sourced from the NTRS TRL-5 design overview**
(`ascend24-ipex-trl-5-design-overview`, NTRS 20240008162, p.2): *"The IPEx project is developing a
**30 kg-class excavator**."* Low mass is the deliberate design thesis — counter-rotating bucket drums
cancel horizontal dig reaction, so IPEx does NOT rely on high mass for tractive force. Drum payload is
**up to 30 kg per excavation cycle** (p.~12), so the rover is ~30 kg dry and up to ~60 kg laden.

Add to `constants.py` (honesty-tagged):
- `ROVER_MASS_DRY_KG = 30.0   # [CALIB] (ascend24 TRL-5, "30 kg-class")`
- `DRUM_PAYLOAD_MAX_KG = 30.0 # [CALIB] (ascend24, 30 kg/cycle, 15 kg min)`
- static per-wheel load `F_WHEEL_N = (ROVER_MASS_DRY_KG + drum_payload_kg) * g / 4` → **≈12.2 N/wheel
  dry, ≈24.3 N/wheel laden**. The payload-dependent weight-on-wheels is itself a path-dependent
  dynamic (excavating loads the drums → more sinkage → more slip), which the closed loop should expose.

**0.3 Stand up the Chrono oracle as a calibration harness.** New `scripts/chrono/calibrate_sinkage.py`
that reuses the working `chrono_scm_rover.py` SCM path (`SetSoilParameters` already takes
`Kphi,Kc,n,cohesion,friction,Janosi_K` — the **same** `constants.py` moduli) at lunar g, sweeps
{normal load × pass-count}, and writes a real table `samples/calib/scm_sinkage.json`:
`(load_N, pass_index) → {sinkage_m, sinkage_plastic_m, sigma_yield, tau}` from `GetNodeInfo`
(`chrono_scm_export.py:173,181-182`). This is the **real measured-by-SCM curve** the surrogate fits to.
SCM is semi-empirical single-layer Bekker, so it is a *consistency* oracle for sinkage, not first-
principles truth — documented as such.

*Effort: small (gated on 0.2).* Deliverable: a real sinkage table on disk, no surrogate change yet.

---

## Phase 1 — Load-bearing static pressure-sinkage (the keystone)

**1.1 Tests first** (`terrain_authority/test_terramechanics.py`, new persistent module):
- `test_sinkage_monotone_in_load`: `bekker_sinkage` strictly increases with normal load.
- `test_multipass_paving`: pass 2 over a compacted rut sinks *less* than pass 1 (denser soil, higher
  bearing) — the spec §6 paving effect, now emergent from physics not a constant.
- `test_matches_scm_oracle`: surrogate sinkage reproduces `samples/calib/scm_sinkage.json` within a
  stated tolerance (e.g. ≤20% RMS across the load sweep) — **real-data validation**.
- `test_mass_conserved_load_path`: total mass unchanged after a load-driven `four_wheel_pass`
  (regression of `invariant-1`).
- `test_legacy_constant_path_unchanged`: with `compaction=0.12` (default), output is byte-identical to
  today (regression — protects the 19/15 suite + `tread_track`).

**1.2 Implement `bekker_sinkage(load_N, contact_area_m2, *, n, k_c, k_phi, b, density)`** in `rover.py`
(or a new `terramechanics.py` imported by `rover.py`). Pressure `p = load_N / contact_area`;
invert Bekker `p = (k_c/b + k_phi)·zⁿ` → `z = (p / (k_c/b + k_phi))^(1/n)`. Reads the currently-dead
`K.K_C/K.K_PHI/K.N_SINKAGE`. Map sinkage `z` → a per-cell density bump that yields the same thickness
drop (so mass stays conserved; `Δthickness = z` over the contact, `density ↑` accordingly, capped at
`RHO_DEEP`). This **replaces the single line** `rover.py:74` / `:183` only on the opt-in path.

**1.3 Per-wheel normal load from `conform_pose`.** Extend `conform_pose` (`rover.py:301`) to also return
`normal_load` per wheel = `F_WHEEL_N` projected on the local plane normal `up` (it already computes the
contact plane and `up` at `rover.py:368-372`) plus static fore/aft transfer from pitch. Thread it into a
new opt-in arg `four_wheel_pass(..., compaction=None, loads=None)`: when `compaction is None`, derive
per-wheel compaction from `bekker_sinkage(loads[k], ...)`.

**1.4 The 1g→⅙g correction (the headline [CALIB] hinge).** Apply the Lyasko-2010 reduced-gravity
reduction to `n, k_phi, c` at lunar g (low-g lowers them, increases sinkage). Add as a tagged function
`lyasko_reduce(params, g_ratio=1/6)  # [CALIB] (lyasko2010)`. **Validate the corrected surrogate against
the SCM oracle run at lunar g** (Phase 0.3). Be explicit in the docstring: there is no lunar ground
truth, so this validates internal consistency, not absolute lunar fidelity — the honest framing the repo
already uses for `[CALIB]` hinges.

*Effort: medium.* Outcome: the Bekker moduli are load-bearing; sinkage responds to weight, slope, and
pass history, fit to the in-repo oracle.

---

## Phase 2 — Slip-sinkage ladder ("slippy dirt", the path-dependent failure)

**2.1 Tests first** (extend `test_terramechanics.py`):
- `test_traction_budget`: available thrust = `c·A + N·tanφ` (Janosi-Hanamoto shear with `K_SHEAR`);
  slip ratio `s` rises when demanded thrust exceeds budget.
- `test_slip_sinkage_feedback`: higher `s` → θ_m=(`SLIP_C1`+`SLIP_C2`·s)·θ_f → extra sinkage → higher
  `s` (the runaway loop). On a slope past a threshold it diverges (Spirit-mode entrapment); reducing
  commanded thrust arrests it.
- `test_slip_mass_conserved`: slip-sinkage that excavates material rearward (bow-wave + rut wall)
  **redistributes** mass but conserves the total (this is a design decision — real slip-sinkage moves
  material, it does not only compact; keep Σmass exact while allowing local push to the rut walls).

**2.2 Implement a per-wheel traction step** consuming the now-live `SLIP_C1/SLIP_C2/K_SHEAR/COHESION/PHI`.
Promote `slip` from the render-only metadata hint (`rover.py:223`) to a **state variable** fed back into
the pose integrator (Phase 3): commanded motion vs achieved motion diverges under slip.

*Effort: medium.* Outcome: the genuine "slippy dirt" — slip, sinkage, and entrapment are now emergent and
path-dependent. This is the single most demonstrative argument for a closed loop over a procedural
generator (spec §1, §6).

---

## Phase 3 — Close the drive loop (`cmd_vel` integrator)

**3.1 The missing primitive.** Add `step_pose(rc, yaw, twist, dt, *, cell_m) → (rc', yaw')`: a
differential-drive integrator (the `(state, command) → next_pose` that does not exist today; motion is a
precomputed `spiral_rc` replay consumed in `drive_spiral.py`'s per-frame loop). ~20-30 lines,
deterministic, unit-tested (straight, arc, in-place yaw).

**3.2 Reverse command seam.** New `scripts/demo/drive_cmd.py` (do **not** edit `drive_spiral.py` — keep
the open-loop demo intact) that, per step: poll a `cmd_vel` directory for a twist (file-mediated, the
same decoupling as the frozen `INTERFACE.md` seam) → `step_pose` → `conform_pose` (now returns loads) →
`four_wheel_pass(loads=...)` (load-driven sinkage + slip) → `derive_height` → `io_fields.save_scene`.
The loop is now closed: a controller (Nav2 or the RL policy) drives, and the rover's own motion reshapes
the scene and slips on what it made.

*Effort: small.* Outcome: ROS (or a Python policy) can drive the simulated robot; slip closes the loop.

---

## Phase 4 — Gymnasium env + honest control reward

**4.1 `terrain_authority/rover_env.py` — `RoverSimEnv(gymnasium.Env)`** over the **headless** authority
(no Godot, no PNG; verified sub-millisecond per step on the 256² @ 2 cm grid):
- `reset(seed)` = `io_fields.load_scene(scene_dir)` → `ColumnState` (the construction already shown in
  `drive_spiral.py`).
- `step(action=twist)` = the Phase-3 loop, returning:
  - **obs**: local heightmap patch + pose + slip + sinkage + an IMU-like (pitch/roll/accel) channel
    derived from `conform_pose` deltas.
  - **reward** (honest, *control* reward from true state, NOT perception-faked): path/goal progress
    − slip-event penalty − energy(load·distance) − excess-sinkage − entrapment terminal. For
    berm-shaping tasks, add a deposition-to-spec term from the true `mass_areal`/`state_label`.
  - **done**: entrapment (Phase 2 runaway), goal reached, or step budget.
- `action_space`/`observation_space` declared; deterministic under seed.

**4.2 Domain randomization API.** Sample episode params from the **honesty-tagged envelopes** already in
`constants.py` (`THETA_R` 30-47°, `SLIP_C1/C2` [UNKNOWN] ranges, the moduli [CALIB] spreads, Hurst
bands). These are *sourced* ranges, not invented — the tags are the randomization spec (spec §7.5).

**4.3 Honesty rail.** Document that this is a *control* environment: the reward reads ground-truth state.
**Perception-in-the-loop RL is a separate, larger track** that first needs the observed-map producer
(no DEM-building node exists; `depth_map.py` is per-frame only) + a `score_map` comparator (the
`Scorecard.map_*` slots are pre-shaped but never computed). Do **not** wire the synthetic SLAM feed
(`synthetic_feed.py`) into any reward — it is truth+noise, not real localization.

*Effort: small-to-medium.* Outcome: a credible, deterministic, domain-randomizable Tier-2 RL environment
where slip and sinkage are real.

---

## Phase 5 (optional, later) — learned world model

With a deterministic, headless, sub-ms env, rollouts are cheap → fit an RSSM/Dreamer or JEPA latent
dynamics model as a sample-efficiency layer (clean **lewm** lineage, avoid gwm contamination). Strictly
downstream of Phases 1-4; the env is the data source, the world model is optional.

---

## Verification (100% against this list)

| # | Check | Gate |
|---|---|---|
| V1 | Legacy 19/15 suite green (regression) | `python -m terrain_authority.tests` |
| V2 | New terramechanics suite green | `pytest terrain_authority/test_terramechanics.py` |
| V3 | Mass conserved on every new op (≤1e-9 rel) | invariant-1 asserts |
| V4 | Surrogate sinkage ≤ tolerance vs SCM oracle | `test_matches_scm_oracle` |
| V5 | Determinism: same seed → byte-identical rollout | env replay test |
| V6 | Legacy `compaction=0.12` path byte-identical | `test_legacy_constant_path_unchanged` |
| V7 | Closed-loop demo drives + slips + reshapes | `drive_cmd.py` run artifact |

## Effort + dependency order

`0 (small, BLOCKER: source IPEx mass) → 1 (medium, keystone) → 2 (medium) → 3 (small) → 4 (small-med)
→ 5 (optional/large)`. Phases 1-4 are the deliverable: a real Tier-2 RL env with load-bearing
slip-sinkage. Phase 5 and perception-in-the-loop are separate tracks.

## Open risks / honest unknowns

- **IPEx mass** (Phase 0.2) is a hard input; blocked until sourced.
- **1g→⅙g correction** (Phase 1.4) is the central `[CALIB]` hinge; no lunar ground truth exists, so
  validation is internal-consistency vs the SCM oracle, not absolute fidelity. State it plainly.
- **SCM is semi-empirical Bekker, not granular DEM** — a consistency oracle for *sinkage*, not a source
  of real drum-excavation forces. Tier-3 (drum torque/throughput) remains a separate large build.
- **Slip-sinkage material redistribution** (Phase 2.1) is a modeling choice: keep Σmass exact while
  allowing local bow-wave/rut-wall push; decide the redistribution kernel up front.
- **It is John's repo.** Branch + PR posture; preserve authorship and the honesty-tag discipline.
