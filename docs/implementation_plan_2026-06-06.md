# DustGym Implementation Plan — executing PRD v6.0

**Date:** 2026-06-06 · **Baseline commit:** `0473312` · **Drives:** [`PRD.md`](../PRD.md) v6.0
**Basis:** [`prd_gap_analysis_2026-06-06.md`](prd_gap_analysis_2026-06-06.md),
[`architecture_review_2026-06-06_full.md`](architecture_review_2026-06-06_full.md)

This turns the PRD §10 roadmap + §7 requirement matrix + the gap-analysis work order into a single
dependency-ordered execution plan. Each work package (WP) names its deliverable, the PRD requirement
IDs it closes, its acceptance gate (the PRD `V` column), its blocking dependencies, and any external
data gate. `★` marks the critical path. Effort is T-shirt (S < ~1 day, M ~2-4 days, L ~1-2 weeks,
XL > 2 weeks) and is *engineering* time, not wall-clock.

## 0. Sequencing principles (non-negotiable)

1. **No release while any `RB-*` is open** (PRD §4.2). Phase 0 is the gate; nothing in Phases 1-5
   counts as "done" product until Phase 0 closes.
2. **Build the six authoritative artifacts first, everything else is a view** (PRD §6.1):
   `WorldState`, `VehicleState`, `VehicleModel`, `BeliefState`, `PlanResult`, `ExecutionEvent`.
   Reports, Plan IR, playback, validation, autonomy must not recompute independently.
3. **Never build autonomy on an unvalidated seam** (gap analysis). Time-sync, atomic scene
   publication, and the plan contract precede SLAM/nav.
4. **Data-gated `[G]` items do not block code.** The 10 Open Decisions (PRD §13) run as a parallel
   de-risking track; until a value is sourced it stays `[PROPOSED]`/`[ASSUMPTION]` behind config and
   makes **no capability claim**. No `[UNKNOWN]` is replaced by a guessed constant.
5. **Status is four columns (I/X/V/Q), not a checkmark.** A WP is "done" only at PRD §15 Definition
   of Done (merged + product consumes it + success/failure tests + qualification where required +
   documented + no contradictory status elsewhere).

## 1. Critical path (one line)

```
WP0.0 requirement manifest + CI truth (RB-02)
  → WP0.1 domain validation (RB-01)
  → WP0.3 PlanResult ★ (RB-03)  ─┬→ WP0.4 per-vehicle IR (RB-04)
                                 └→ WP0.5 VehicleModel threading (RB-05)
  → WP0.6 installed server (RB-06)              [Phase 0 exit: all RB-* = D]
  → Phase 1 vehicle/posture twin
  → Phase 2 nav spine (PlanResult + time-sync + atomic scene are prerequisites)
  → Phase 3 solar active-perception
```
Phases 4 (construction-under-mutation) and 5 (fleet/product) overlap Phases 2-3 once `PlanResult`
and `WorldState` mutation exist. The data de-risking track (§8) runs continuously alongside.

## 2. What the recent vehicle work already advanced

The 2026-06-06 two-selectable-bodies work (`0473312`) partially advances several v6 rows — useful to
not re-do, but **none are closed**:

- **VT-01** (typed VehicleModel: geometry/mass/battery/drum/render): `I=P` — `vehicles.Vehicle` now
  carries geometry + render assets, two bodies registered, render-verified. Still missing: a single
  *typed* `VehicleModel` consumed by the **authority + planner** (only env + render consume it today).
- **VT-02 / RB-05** (selecting a vehicle changes *all* numbers): still `N` — geometry drives
  stability + render; energy/drum/terramechanics are still **shared** across bodies. This is the core
  of WP0.5.
- **VT-06** (support polygon / stability margin): `I=P` — `stability.py` SSA per vehicle.
- The `ez_rassor` vs `ipex` split makes WP0.5's cross-vehicle diff tests (VT-02) concrete to write.

## 3. Phase 0 — Truthful baseline + release gates  ★  (exit: all `RB-*` = D)

| WP | Deliverable | Closes | Acceptance (V) | Deps | Effort |
|---|---|---|---|---|---|
| **0.0** | **Requirement manifest + truthful CI** — checked-in machine-readable `requirements.yaml` (one row per PRD ID: I/X/V/Q, owner, source module, acceptance test ID, evidence commit, blocked-by, last-verified); declare `trimesh`; CI runs the *configured* suite across supported Python, tiered T0/T1/T2 separately. | RB-02, PO-03, PO-04, CT-07 | CI installs declared deps and collects the full configured suite (no excluded path); manifest lints against the PRD. | — | M |
| **0.1** ★ | **Domain validation at every public boundary** — one `validation` module (units/finiteness/physical-domain); reject NaN/Inf/negative depth & mass; `ColumnState` validates dims/shapes/dtypes/domains/density/labels/disturbance/datum/ice/inventory at construction; every mutation transactional + invariant-checked; replace removable `assert` with explicit exceptions. | RB-01, CT-01, CT-02, CT-03, CT-06 | T0 property tests: bad inputs raise at the public boundary; mutation invariants hold; mass drift ≤ 1e-9. | 0.0 | L |
| **0.2** | **Atomic scene publication** — write verified rasters atomically, metadata last as the commit marker; schema-validate Python↔Godot↔ROS fields (required/frame/dtype/range). | CT-04, CT-05 | T0/T2: partial write never leaves a loadable-but-invalid scene; schema mismatch rejected. | 0.1 | M |
| **0.3** ★ | **`PlanResult` authoritative artifact** — one immutable, fleet-aware result (allocation, routes, per-vehicle timelines, energy ledger, validation state, acceptance, provenance). Totals, report, Plan IR, autonomy inputs, browser playback become **views** over it. | RB-03, CP-01, FL-01 | T1: totals/report/timeline/IR/playback are byte-consistent with one `PlanResult`; no independent recompute path remains. | 0.1 | L |
| **0.4** ★ | **Per-vehicle Plan IR ledger** — `prev_by_vehicle` position/energy/time/action state; no cross-vehicle position leak. | RB-04, NV-10 | T1: route/energy/time conservation asserted per vehicle on a 2-rover plan; leak test fails on the old path. | 0.3 | M |
| **0.5** ★ | **Typed `VehicleModel` threaded end-to-end** — one model drives contact geometry, mass, battery, drum capacity, drive, terramechanics, planner simulation, Plan IR, endurance, reports (not just env+render). | RB-05, VT-01, VT-02, EP-08 | T1 cross-vehicle diff: `ez_rassor` vs `ipex` produce **different** mass/contact/energy/capacity/plan numbers with asserted expected deltas. | 0.3 | L |
| **0.6** ★ | **Installed-product reality** — coherent `planner`/`server`/`render`/`dev` extras (server extra includes *all* import-time planner deps); reports/profiles/cache to a configurable app-data dir with atomic writes; package or version-download terrain/render assets; fresh-wheel smoke for `dustgym-serve` + every registered env + planner import. | RB-06, PO-01, PO-02 | T2: fresh wheel in a clean venv → `dustgym-serve` starts, plans, writes to the app-data dir. | 0.0 | L |
| **0.7** | **PRD-truth cleanup** — remove the 51 `sys.path` insertions + stale roversim/source-layout text; route unreachable terrain to *infeasible* (no silent straight-line/flat fallback, NV-01). | NV-01, (PRD §14 hygiene) | T1: unreachable goal → explicit infeasible; grep shows no stale layout guidance. | 0.3 | M |

**Phase 0 exit criteria:** RB-01..06 all `D`; CI green on the configured suite across supported Python;
the requirement manifest is the generated status source.

## 4. Phase 1 — Vehicle & posture twin  (exit: one vehicle/arm/drum state drives physics, render, planning, sensors)

| WP | Deliverable | Closes | Deps | Data gate | Effort |
|---|---|---|---|---|---|
| 1.1 | `VehicleState` artifact (pose/vel/arm angles/per-drum fill/battery/thermal/dust/health) + four-drum inventory replacing the global drum. | VT-04 | 0.5 | — | M |
| 1.2 | Arm joints (front/rear: state, limits, velocity, brake, energy) + posture-dependent camera extrinsics. | VT-03, VT-10 | 1.1 | OD-1, OD-2 `[G]` | L |
| 1.3 | Dynamic CG (chassis+arm+drum+fill) → posture-dependent support polygon + per-step stability margin. | VT-05, VT-06 | 1.2 | OD-3 `[G]` | M |
| 1.4 | Counter-rotation balance + asymmetric-dig reaction/yaw/pitch risk model; drum bridging fill-rate (≈half-scoop saturation). | VT-07, VT-08, VT-09 | 1.1 | OD-4 `[G]` | L |
| 1.5 | Guarded posture state machine — `TRANSIT/DIG/DUMP_Z/MEERKAT/BRAKED_HOLD` first, with per-transition slope/arm-range/load/contact/stability/clearance preconditions; `DRUM_WALK/IRON_CROSS/SELF_RIGHT` stay gated. | AM-01, AM-02, AM-03, AM-08 | 1.3 | OD-1, OD-5 `[G]` | L |

Until OD-1..5 are sourced, Phase 1 ships as a `[PROPOSED]` model behind config with **no** capability
claim; `V` can pass on the *logic*, `Q` stays `G`.

## 5. Phase 2 — LAC-derived navigation spine  (exit: repeatable sensor-only nav/mapping benchmark, no truth leak)

Mirror the `[NAVLAB26]` modular pattern (equivalents allowed if they meet acceptance).

| WP | Deliverable | Closes | Deps | Effort |
|---|---|---|---|---|
| 2.1 | Time-sync + strict frames across camera/IMU/command/arm/truth streams. | PM-01 | 0.2 | M |
| 2.2 | Grayscale segmentation (ground/rock/lander/fiducial/sky), truth-mask-free in EVALUATE. | PM-03 | 2.1 | L |
| 2.3 | Illumination-robust features (SuperPoint+LightGlue or equiv) + confidence/inliers; stereo VO with persistent tracks + robust SE(3). | PM-04, PM-05 | 2.1 | L |
| 2.4 | Covariant estimator/factor graph (GTSAM-style) fusing VO/IMU + validated absolute factors; candidate-gated, geometrically-verified, auditable loop closure. | PM-06, PM-07 | 2.3 | XL |
| 2.5 | Robust height map + rock occupancy/probability; coverage/uncertainty/correlation tracking (no dense-pixel independence fallacy). | PM-08, PM-09 | 2.3 | M |
| 2.6 | Coverage routes (overlapping loops/outward spiral) + constant-curvature local arc planner + path tracker + backup recovery (progress-ratio/duration/failure; blockage-vs-slip discrimination). | NV-02, NV-03, NV-04, NV-06, NV-07 | 0.3, 2.5 | L |
| 2.7 | Fixed LAC-style benchmark (loc RMSE, 5 cm height pass-fraction, rock F1, coverage, runtime, failures) across seeds/light/rocks; truth structurally unavailable to estimator (PM-12). | PM-10, PM-11, PM-12 | 2.4, 2.6 | M |

Target: repeatable centimeter-scale localization comparable to `[NAVLAB26]` `0.038-0.067 m` **before**
any parity claim (PM-11).

## 6. Phase 3 — Solar-terrain active perception  (exit: solar evidence + arm/Meerkat decisions improve or safely preserve nav)

| WP | Deliverable | Closes | Deps | Data gate | Effort |
|---|---|---|---|---|---|
| 3.1 | Site/time sun-vector `s(t)` service; terrain horizon/illumination/cast-shadow/incidence/overexposure from terrain+`s(t)`; recompute after excavation (no stale shadow map). | TW-06, TW-07, TW-08 | 0.2 | OD-6 `[G]` | M |
| 3.2 | Shadow-azimuth extraction + rejection (rover/LED shadow, saturation, penumbra, texture); weak yaw factor with covariance (never absolute heading); re-eval on terrain/sun/viewpoint change. | SN-01, SN-02, SN-03, SN-04 | 3.1, 2.4 | — | L |
| 3.3 | Illumination-aware route cost (separable terms) + camera direction/exposure + camera-subset/LED policy within active-camera & power budgets. | SN-05, SN-06, SN-07, EP-06 | 2.6, 3.1 | OD-7 `[G]` | M |
| 3.4 | Posture-dependent observation (arm-angle near-field/horizon; guarded Meerkat) + multi-height feature association; active-perception objective = info per joule·s with stability as a hard constraint. | SN-08, SN-09, SN-10, SN-11, AM-09 | 1.5, 3.2 | OD-1,2 `[G]` | L |
| 3.5 | Full ablation vs no-solar across sun angle/terrain/terrain-change/posture/seed; preregistered improvement threshold; PRD §9.3 acceptance checklist. | SN-12, SN-13 | 3.4, 2.7 | OD-9 | M |

No solar-nav capability claim until all seven PRD §9.3 conditions pass.

## 7. Phases 4 & 5 — construction-under-mutation + fleet/operational product (overlap once `PlanResult` + `WorldState` mutation exist)

- **Phase 4 (CP/TW):** typed footprint grammar (rectangle/circle/corridor/polygon) replacing scalar
  squares (CP-05); construction mutates `WorldState` → routing/illumination/observability/acceptance
  consume the update (CP-09, TW-08); full structure acceptance + uncertainty bands (CP-06, CP-07);
  tool/arm/drum actions in Plan IR + executive (NV-09).
- **Phase 5 (FL/PO/NV):** shared-resource fleet scheduling (charger/pit/dump/vantage/corridor) +
  coordinated replan + 2-rover exact oracle before any superiority claim (FL-03/04/06/07); fleet
  playback + solar view (PO-11/12); ROS lowering + versioned streaming command/telemetry API
  (NV-11/12); lock/SBOM/audit/fresh-install, `CHANGELOG`/SemVer/release manifest, deployment
  image/docs (PO-05/13/14).

## 8. Data de-risking track (parallel, continuous) — PRD §13 Open Decisions

| OD | Needed for | Resolution path |
|---|---|---|
| OD-1 arm pivot geom/limits/speed/brake/lift | VT-03, 1.2, 1.5, 3.4 | Trace `[IPEx-DT-REF]` `[SPEC]` claims to NASA/LAC source; else `[PROPOSED]`. |
| OD-2 camera intrinsics/extrinsics incl. arm-mounted | VT-10, PM, 3.x | LAC simulator camera config; `[NAVLAB26]` setup. |
| OD-3 chassis/arm/wheel/drum/fill mass props | VT-05, 1.3 | NASA IPEx mass breakdown (separate avionics paper); else `[ASSUMPTION]`. |
| OD-4 IPEx drum geom/scoop/per-drum capacity | VT-04, VT-08, 1.4 | Bucket-drum scaling paper (have medium dims) + flight-drum confirm. |
| OD-5 actuator power/efficiency + transition energy | AM-08, EP-06, 1.5 | TRL-5 actuator paper (ThinGap/Harmonic Drive load tables). |
| OD-6 sun-vector/ephemeris lib + frame convention | TW-06, 3.1 | Pick SPICE or a documented ephemeris; fix site/time frame. |
| OD-7 camera response/exposure/LED photometry/dust | SN-06, SN-07, 3.3 | TRL-5 camera paper (IMX547, f/4, 3000 lm LEDs); dust = `[PROPOSED]`. |
| OD-8 NavLab adopt vs reimplement vs baseline-only | Phase 2 | Decision gate before WP2.3/2.4. |
| OD-9 preregistered solar/Meerkat improvement margin | SN-13, 3.5 | Set before running 3.5 ablation. |
| OD-10 first operational target | OPERATE mode scope | Aaron/John + GMRO: sim-LAC-parity vs test-site vs hardware. |

## 9. Verification & evidence (cross-cutting, lands incrementally)

- Test tiers per PRD §9.1: **T0** unit/domain/invariant, **T1** cross-module integration, **T2**
  fresh-wheel/browser-syntax/headless-Godot smoke (all in standard CI); **T3** nav/perception
  benchmark on a scheduled GPU runner; **T4** ROS/test-site/Chrono/calibrated-hardware.
- Every artifact records source commit, config, mode, seed, schema version, input hashes (CT-07).
- The requirement manifest (WP0.0) generates status; the PRD stops carrying present-tense counts.
- KPIs tracked per PRD §11 (mass drift, loc RMSE, 5 cm height pass-fraction, rock F1, info/joule,
  tip count, fresh-install, plan consistency).

## 10. Immediate next sprint — the first five PRs (all Phase 0, in order)

1. **WP0.0** — add `trimesh` to deps; CI runs the configured suite tiered; commit a
   `requirements.yaml` manifest skeleton seeded from PRD §7. *(unblocks honest status)*
2. **WP0.1** — `terrain_authority/validation.py` + `ColumnState` construction/mutation invariants +
   T0 property tests. *(RB-01)*
3. **WP0.3** — introduce a read-only `PlanResult`; make `_mission_totals` + the report consume it
   (first two views). *(RB-03, start)*
4. **WP0.4** — `prev_by_vehicle` in `plan_ir` + per-vehicle conservation tests. *(RB-04)*
5. **WP0.6** — fresh-wheel `dustgym-serve` smoke test + the `server` extra dependency audit. *(RB-06, start)*

Each PR: TDD, one requirement-manifest row flips with an evidence commit, no contradictory status left
behind (PRD §15).

## 11. Risks & how this plan handles them

- **Data gates stall Phase 1/3** → the de-risking track (§8) runs from day 1; gated items ship as
  validated *logic* (`V`) with `Q=G`, never as capability claims.
- **`PlanResult` refactor is invasive** → it is the critical-path keystone; do it early (WP0.3) while
  the surface is smaller, before fleet/solar add consumers.
- **SLAM scope (WP2.4) is XL** → decide OD-8 (adopt vs reimplement) before starting; allow NavLab
  components directly if license-compatible to cut risk.
- **Scope creep into stunts** → AM drum-walk/iron-cross/self-right stay gated (PRD §12 non-goals);
  only Meerkat + braked-hold are in near-term scope.
