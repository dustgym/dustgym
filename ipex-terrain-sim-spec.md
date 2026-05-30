# IPEx Lunar Terrain Simulation — Technical Specification

**Status:** Draft / handoff
**Scope:** Architecture and physics specification for a sensor-faithful lunar terrain simulator supporting IPEx perception and human-in-the-loop (HITL) autonomy development.
**Audience:** Engineering handoff — assumes familiarity with ROS2, terramechanics, and game-engine rendering.

> **IP note:** Intended as U.S. Government work (17 USC §105) — public domain by default. No third-party copyrighted dependencies should be introduced that would compromise public release. Prefer permissively licensed components (Godot is MIT; Project Chrono is BSD-3).

---

## 1. Purpose and Framing

This is **not** a physics simulator that happens to render. It is a **calibrated, sensor-faithful surrogate** whose fidelity is judged *at the camera output*, not at the force vector.

The pass/fail question is: *"Does the perception stack see what it would see on the testbed (and ultimately at the pole)?"* — not *"Do contact forces match a DEM reference?"*

Consequences of this framing:
- Effort spent on force accuracy beyond what changes **visible geometry and dust** is wasted.
- Effort spent on exposed-sublayer albedo, rut/shadow geometry, and dust-field placement is the actual product.
- A high-fidelity granular solver (DEM) is the **calibration oracle**, run offline on a few representative cuts — **never** in the live loop.

### Why a closed loop (not a procedural generator)

The dangerous perception failures on IPEx are **path-dependent** — they exist only because the robot's own motion perturbed the scene it must then perceive:
- Driving into a self-generated dust cloud.
- A wheel sinking and pitching the camera toward a washed-out grazing-sun angle.
- Displacing or uncovering a rock that casts a new deceptive shadow.
- A berm that looks stable, then slumps.

An open-loop terrain generator structurally cannot produce these. A coupled sim surfaces exactly this failure class, which is what supervised-autonomy validation needs.

### RL vs. HITL

Reinforcement learning cannot be flight-certified on this mission timeline. The simulator targets **ground-based HITL robotics objectives**: operator training, perception hardening, and scenario regression — not flight policy learning.

---

## 2. System Architecture

Single-authority dynamics, decoupled rendering, downstream perception evaluation:

```
  ┌─────────────────┐     state fields     ┌──────────────┐    synthetic     ┌───────────────┐
  │  Project Chrono │ ───(height, density,─▶│    Godot     │───imagery + ────▶│ Robot / ROS2  │
  │  (PHYSICS       │   disturbance, dust,  │  (RENDER +   │    sensor data   │ debug env     │
  │   AUTHORITY)    │   ice, clasts)        │  SENSOR MODEL)│                  │ SLAM stack    │
  │                 │                       │              │                  │               │
  │ • Chrono::Vehicle                       │ • lighting   │                  │ • sensor noise│
  │ • terramechanics│                       │ • dust shaders│                 │ • SLAM/mapping│
  │ • clasts (rigid)│                       │ • cam intrinsics                 └───────┬───────┘
  └────────┬────────┘                       │ • dirty lens │                          │
           │                                └──────────────┘                          │
           │  ground truth (true pose, true terrain at time t)                        │
           └──────────────────────────────────────────────────────────────────────────┘
                                    EVALUATION
```

### Authority model (critical)

**Pick one source of truth for dynamics: Chrono owns *all* physics** — rover (Chrono::Vehicle), terramechanics, and clasts as rigid bodies. **Godot is purely renderer + sensor model.**

Do **not** split rigid-body authority across two engines (e.g., rocks in Jolt, soil in Chrono). Divergence between the two solvers is a multi-week debugging sink. This revises any earlier notion of promoting clasts to Godot/Jolt bodies — freed clasts live in Chrono.

Benefit: the sim stays **deterministic and replayable**, which is required for HITL regression and reproducible scenario libraries.

### Two ground-truth comparisons (free)

Because the robot *mutates* the terrain, the architecture yields two independent evaluations:
1. **SLAM pose estimate** vs. Chrono true rover pose.
2. **Built/observed elevation map** vs. Chrono true terrain *at time t* — the actual LAC-style scoring objective, and time-varying because the robot is reshaping the scene.

---

## 3. Fidelity Tiers

Three tiers exist; **this project targets Tier 2.**

| Tier | Description | Use |
|---|---|---|
| 1 | Static procedural terrain + camera filters | Open-loop SLAM data only. Robot "out of the picture." |
| **2** | **Coupled, semi-empirical: analytical terramechanics (Bekker/Janosi/Wong-Reece), heightfield carving, rocks as rigid bodies, dust driven by slip & drum RPM** | **Closes the loop; captures path-dependence; no granular solver. Target.** |
| 3 | Full granular terramechanics (Chrono::GPU DEM, AGX Terrain) | Excavation forces / throughput / actuator loads. Offline calibration oracle. |

**The primary (optics/perception) objective lives at Tier 2.** Tier 3 is needed only if excavation *forces/throughput* enter scope. Even NASA's perception-focused LAC twin is closer to Tier-2 sinkage/slip than full DEM (its task is mapping, not active digging).

**Scope flag (resolve with charter):** whether excavation *forces* are in scope determines Tier 2 vs. Tier 3. Default assumption here: **out of scope → Tier 2.**

---

## 4. Spatial Representation

### Base terrain: heightfield with per-column subsurface state ("stacked heightfield")

Terramechanics is intrinsically **2.5D** — a surface with a depth-density profile beneath each point. **Do not use marching cubes / global voxels for the base terrain:** granular regolith cannot sustain the overhangs/voids that volumetric representations buy you (angle of repose collapses undercuts), and it severs the clean mapping to the terramechanics math.

- **Index:** a **quadtree**, used for LOD, hot-region management, and state labels (`VIRGIN`/`TREAD`/`EXCAVATED`/…). The tree manages *space*; it is **not** the physics substrate.
- **Solve grid:** run the terramechanics solve on a **uniform fine grid** inside each active patch, not across variable-size quad cells. Integrating σ(θ)/τ(θ) or redistributing mass across unequal cells breaks conservation and the Wong-Reece integration.
- **Escape hatch:** if undercut/void fidelity at the drum face ever proves perception-relevant, drop a **local** SDF/voxel patch into the active excavation zone only — never a global volumetric terrain.

### Spatial LOD — keyed to interaction, not distance

Three live zones; everything else static. Bounds cost regardless of map size.

| Zone | Fidelity | Contents |
|---|---|---|
| Far field | Render only, no physics | Procedural heightfield. Most of the map. |
| Under wheels (rolling) | Active terramechanics, moving window | Wong-Reece sinkage, compaction update, rut carving, multi-pass density bump |
| Under drums (digging) | Highest | Material removal, berm deposition, clast uncovering, optional local DEM/voxel patch |

### Resolution anchors

- Active-zone heightfield cell: **1–3 cm** (driven by hazard/feature scale and the ~10–20 cm contact patch).
- Far field: 10+ cm with shader detail.
- Column profile: a few layers or a parametric density curve — enough to capture loose-over-dense and the cut exposing the firm layer. **No fine vertical voxelization.**
- Render resolution may be 5–10× physics resolution via detail normals / procedural micro-displacement; physics never needs to know.

### Physics ↔ render interface

The handoff is a set of **texture-encoded state fields** Godot samples in shaders:
`heightmap`, `compaction/density map`, `disturbance map`, `dust-deposition map`, `ice/volatile map`.

Physics owns geometry + material state at coarse resolution; rendering owns appearance at fine resolution. Physics says *"this column is freshly cut, dense sublayer exposed, disturbed"*; the shader produces the albedo/roughness/normal + dust accumulation, then the camera-intrinsics + dirty-lens post chain finishes it. The two clocks need not match (terramechanics surrogate runs cheap, hundreds of Hz on the active patch; render at sensor frame rate).

---

## 5. Data Model

### 5.1 Global physics — fixed constants & per-scenario site config

| Field | Symbol / units | Value / range | Notes |
|---|---|---|---|
| Surface gravity | g (m/s²) | 1.62 | Fixed. ⅙ Earth. Drives everything below. |
| Grain specific gravity | G_s | 3.0–3.32 (≈3.1) | Well-constrained; sets solid density. |
| Solar irradiance | S (W/m²) | ≈1361 | Thermal/optics; same as 1 AU. |
| Atmospheric drag | — | none (vacuum) | Dust is **ballistic, not suspended.** |
| Sun elevation | θ_sun (deg) | 0–7° (polar) | Site config; grazing → extreme shadows. |
| Site latitude / illumination | — | scenario | Shadow geometry + thermal cycle. |
| PSR / cold-trap flag | bool | per-site | Gates the volatile regime (§5.2). |

### 5.2 Global physics — genuine unknowns / calibration parameters

> All Bekker parameters are **gravity-sensitive**: lowering gravity decreases the sinkage exponent *n*, frictional modulus *k_φ*, and cohesion *c*, while *k_c* and φ show little change; sinkage **increases** as gravity drops under the same load (Earth-fit parameters under-predict lunar sinkage). The *k_c*/*k_φ* figures below are classic Apollo-era (Mitchell/Costes) fits — **calibration starting points, not ground truth.**

| Field | Symbol / units | Range (typical) | Notes |
|---|---|---|---|
| Surface bulk density | ρ₀ (g/cm³) | 1.1–1.5 (≈1.30) | Loose top layer. |
| Deep bulk density | ρ_∞ (g/cm³) | 1.8–2.0 (≈1.92) | Below ~100 cm. |
| Density transition depth | z_t (cm) | 10–15 | Loose-over-dense; sets self-limiting sinkage. |
| Cohesion | c (kPa) | 0.1–1.0 (≈0.17) | Interlocking-driven; ↓ in low-g. |
| Internal friction angle | φ (deg) | 30–50 (→55 at depth) | ~g-independent. |
| Bekker cohesive modulus | k_c (kN/m^(n+1)) | ≈1.4 (calibrate) | ~g-independent. |
| Bekker frictional modulus | k_φ (kN/m^(n+2)) | ≈800–820 (calibrate) | **↓ in low-g.** Wide uncertainty. |
| Sinkage exponent | n | 0.8–1.0 (≈1.0) | ↑ with density; **↓ in low-g.** |
| Shear deformation modulus | K (cm) | 1.0–1.8 (≈1.8) | Often fixed when identifying c, φ. |
| Slip-sinkage coeffs (θ_m) | c₁, c₂ | c₁≈0.4, c₂≈0.3 | θ_m=(c₁+c₂·s)·θ_f. Genuine unknowns. |
| Angle of repose / critical angle | θ_r (deg) | 30–47 | Finer → steeper; highland steeper than mare; **steeper in low-g** via relative cohesion. Wide envelope. |
| Bulking / swell factor | SF | 1.1–1.3 | In-situ→loose density drop. Closes cut/fill loop. |
| Median grain size | D₅₀ (µm) | 40–130 (≈70) | **Fine** (silty fine sand), poorly sorted, angular. |
| Ice / volatile mass fraction | w_ice (%) | 0 (dry) – 5.6 ± 2.9 (PSR) | LCROSS-derived. Site-dependent; gates regime flag. |
| PSR temperature | T (K) | <110 (PSR) | Cold-trap threshold. |
| Rock size-frequency | CFA(d) | site-dependent | Golombek-style cumulative fractional area; samples buried-clast field. |

### 5.3 Per-column terrain state (dynamic)

| Field | Symbol / units | Range / domain | Notes |
|---|---|---|---|
| **Areal mass** | m (kg/m²) | ≥0 | **The conserved invariant.** Everything else derives from / modifies this. |
| Bulk density (current) | ρ (g/cm³) | 1.1–2.0 | Compaction state; drives strength. |
| Surface elevation | z_surf (m) | — | *Derived*: z = m/(area·ρ). **Never stored independently.** |
| Exposed-sublayer depth | z_cut (cm) | 0–~100 | Loose layer removed; sets local strength + albedo. |
| State label | enum | VIRGIN / TREAD / EXCAVATED / SPOIL / COMPACTED_BERM | Discrete class over continuous state. |
| Disturbed flag | bool | — | Loose → eligible for sandpile relaxation each tick. |
| Cumulative load / pass count | N or J | ≥0 | Drives multi-pass compaction. |
| Local critical angle | θ_r,local (deg) | inherits global | Per-cell override when cemented. |
| Ice / volatile fraction | w (%) | 0–~6 | Feeds regime flag + sublimation + optics. |
| Regime flag | enum | GRANULAR / CEMENTED | *Derived* from w vs threshold. CEMENTED disables relaxation → brittle/blocky failure. |
| Embedded-clast refs | list | — | Rocks promoted to Chrono rigid bodies when uncovered. |
| Surface albedo / maturity | a | fresh ↔ weathered | Fresh cut bright/dense; desiccated lag darkens; frost transient. |
| Exposure time / temp (optional) | t_exp, T | — | Thermal + sublimation decay + frost re-condensation. |

---

## 6. State Transitions

Discrete **labels**, continuous **state**. **Bookkeep mass, derive height** (`height = mass / (area × density)`). This is load-bearing for berm building (see bulking, §7).

| Transition | Trigger | Effect |
|---|---|---|
| VIRGIN/TREAD → **TREAD** | wheel pass | density ↑, height ↓ slightly, strength ↑ (multi-pass "paving") |
| any → **EXCAVATED** | drum cut | remove mass → drum inventory; expose dense sublayer (higher strength, brighter albedo); height ↓ |
| any → **SPOIL** | dump | add mass from inventory at loose density; height ↑ |
| SPOIL → **COMPACTED_BERM** | wheel pass over spoil | density ↑ — deliberate "build a real structure" step |
| (loose cells) | every tick | **sandpile relaxation** (§7) |

### Multi-pass effect (important for haul roads)

Pass 1 compacts the rut; passes 2+ meet **denser, stronger, higher-bearing** soil → sinkage *decreases*, traction improves: the rover **paves its own road**. Failure flip-side: a high-slip pass that shears/loosens material *degrades* the path instead.

### Rocks are not a soil problem

"Hitting a rock" is rigid-body contact (embed/displace/climb) — Chrono handles it natively. Only the **regolith** needs terramechanics; don't let rocks drag the design toward DEM. Three consequences of uncovering a clast: (1) drum jam/deflect if larger than ingest size → excavation fault to detect; (2) new mobility obstacle; (3) **new hard shadow at grazing sun → deceptive perception feature** (the loop-closure payoff).

### Two distinct sinkage modes (do not conflate)

- **Static bearing sinkage** — settles into loose top layer until rising bearing capacity balances weight. Self-limits fast/shallow in ⅙ g (sub-cm to a few cm). Mostly benign.
- **Slip-sinkage** — high slip ratio *s* shears/excavates material rearward; θ_m migrates rearward (θ_m=(c₁+c₂s)θ_f); wheel digs in; slip rises further → **runaway entrapment** (the Spirit-rover failure). Visual: rim below grade, soil bow-wave in front, sharp-walled rut behind. **This is the failure HITL operators most need to recognize early**, and it is purely path-dependent.

---

## 7. Berm Building, Bulking, and Collapse

### Bulking (swell factor) — why mass, not height, is the invariant

Cut in-situ regolith (~1.6–1.9 g/cm³) and dump it → it expands as voids open between angular grains → loose spoil (~1.3 g/cm³ or lower). **A bucket deposits more volume than the hole it left.** Bookkeep in height and cut/fill never reconciles; bookkeep in **mass** with density mediating height and the cycle closes exactly:

```
in-situ (dense) → drum inventory → deposited spoil (loose, taller per kg)
                → optionally driven over → re-compacted (COMPACTED_BERM)
```

### Why berms may be a primary task

ISRU berms support landing-pad blast protection, roads, and foundations. **(Hypothesis — not a confirmed IPEx requirement; verify with charter.)** If building a berm *to spec* is a mission objective, the sim must model deposition well enough to **score task success**, not just locomotion — raising the fidelity bar on deposition specifically.

### Sandpile cellular automaton (collapse / repose)

Grid-native answer to collapsing piles:
1. Dump adds mass to cells under the drum.
2. Relaxation sweep checks each **loose** cell's slope to neighbors.
3. Any cell exceeding the **critical angle** topples excess mass downhill until all ≤ repose.

O(active cells), every tick, loose regions only. Produces avalanches, repose-angle slopes, and slumping on overbuild/undercut. Two knobs:
- **Repose angle** θ_r — wide-envelope calibration parameter (see §5.2; reduced-gravity effect on granular flow is genuinely unsettled in the literature).
- **Cohesion term** — lets piles stand briefly steeper and hold transient undercuts before failing. This **metastability is itself a perception hazard** (a "stable" berm that later slumps breaks a map) and is worth simulating.

---

## 8. Dust, Volatiles, and Optics

### Dust — model in rendering, NOT in mass balance

No atmosphere → no suspension; lofted fines follow ballistic arcs and **land** (meters to tens of meters). Material is globally conserved; bucket-drum excavation is gentle (low-velocity, counter-rotating). The electrostatic coating on lenses/rover is µg–g against a kg-scale budget — **negligible for conservation.**

- Keep cut→inventory→dump a **closed** balance; **assert it in tests.**
- Model dust **entirely** in the rendering/sensor layer.
- Tie dust emission to **disturbed-mass-rate** (slip × normal load, or drum RPM) so the dust field is causally bound to robot action — no CFD/DEM plume model.
- Dust *accumulation* (progressive lens occlusion, albedo/contrast degradation) = material-state-over-time via shaders + exposure bookkeeping. Relevant to IPEx's electro-dynamic dust-shield subsystem (model it degrading and clearing).

### Volatiles / outgassing — thin, slow, mostly optical; gated on PSR flag

- **Dry sites (equatorial/mare):** fresh shallow exposure is mainly an **optical/thermal** effect (brighter denser cut face), not gas release. Impact gardening + shallow thermal skin depth mean shallow material isn't far from equilibrium.
- **PSR / cold-trap sites (IPEx ISRU regime):** excavating icy regolith into sunlight drives sublimation (basis of "thermal mining"), but **rate is throttled hard** — regolith is a strong insulator and a sub-mm desiccated lag crust forms almost immediately and chokes further loss (lab: no measurable bulk loss over 20 h, just a thin crust). **No dramatic venting.**

Implications:
- **Mass:** keep sublimation out of the conservation invariant, or model as a minor optional decay on an exposed-ice field. Not a bulk sink.
- **Optics (where it lives):** albedo transient (bright frost/ice → darker desiccated lag), plus possible **frost re-condensation** on cold surfaces (shadowed cut walls, cold rover parts) — both are path-dependent scene changes that break SLAM.
- **State model:** one **ice/volatile field per column** feeds three consumers — the regime flag (granular vs. cemented/brittle failure), the optional sublimation decay, and the optics layer (frost albedo + re-condensation).

### Camera intrinsics / dirty lens

Godot's camera is FOV-based: set a **custom projection matrix** and apply lens distortion (Brown-Conrady / radial-tangential) as a **post-process shader**. (CARLA gives this natively; here it's a few hundred lines you own.) Dirty-lens veiling/occlusion is the final post stage after dust accumulation.

### Lunar lighting — Godot's strong suit

Single hard directional source, no atmospheric scatter, near-black shadows, extreme dynamic range, brutal low-sun-angle long shadows — well-matched to a modern Forward+ renderer. These grazing-angle conditions are exactly IPEx's perception challenge.

---

## 9. Regolith Domain Notes (intuition corrections)

- **Fine, not coarse.** D₅₀ ≈ 40–130 µm (silty fine sand), poorly sorted, large sub-20 µm dust fraction. Coarse clasts float in a fine matrix; the **matrix governs trafficability and dust.**
- **Angular, minimally eroded.** No water/wind rounding → sharp grains, glass-welded agglutinates → pervasive and abrasive.
- **Cohesive despite being dry** — mechanical interlocking ("like Velcro / paper clips"). Crisp-walled bootprints / rut walls that **hold shape** (not slumping dry sand) — a key visual signature to render.
- **Depth-density gradient dominates.** Loose ~1.30 surface over dense ~1.92 below ~100 cm; strength (φ and apparent cohesion) **rises with density**, so it is depth/compaction-dependent, not constant. This is the hinge for the three terrain states and multi-pass paving.
- **Regional variation:** mare slightly finer/denser than highlands; mature regolith finer/more agglutinate-rich than fresh ejecta. **South-polar PSRs are the least characterized** and possibly most different (ice-cemented behavior) → treat polar mechanical properties as a **parameterized unknown with a wide envelope.**

### Robot design context

Counter-rotating bucket drums (RASSOR heritage) cancel horizontal excavation reaction forces — necessary because in ⅙ g there's too little weight-on-wheels to anchor digging. Downsizing keeps absolute forces within reactable limits. **Implication for the sim:** because forces are engineered small, the Tier-2 analytical layer need not be force-accurate to high precision to be useful — it must be **geometry- and state-accurate.**

---

## 10. Validation Strategy

- **Conservation invariants (assert in tests):**
  1. Total mass: Σ(column mass) + drum inventory + (optional sublimation sink) = constant.
  2. Height–density consistency: z always recomputed from m and ρ, never set directly.
  Get both asserting green and the cut/dump/collapse cycle won't silently drift.
- **Calibration oracle:** run Chrono::GPU DEM (or AGX Terrain) offline on a few representative cuts/wheel passes; fit the Tier-2 analytical parameters (Bekker/Janosi/Wong-Reece, repose, swell) against it. DEM never runs in the live loop.
- **Determinism:** single physics authority + replay → reproducible scenario library for HITL regression.
- **Two-channel eval:** SLAM pose vs. true pose; observed map vs. true terrain at time t.

---

## 11. Candidate Tooling

| Role | Option | License | Notes |
|---|---|---|---|
| Render + sensor model | **Godot 4.x (Forward+)** | MIT | Source access; Linux/Python-friendly; FOV camera needs custom projection + distortion shader. |
| Physics authority | **Project Chrono** (Chrono::Vehicle, SCM deformable terrain) | BSD-3 | PyChrono bindings; used in NASA/academic rover-wheel studies. |
| Calibration oracle (Tier 3) | Chrono::GPU DEM / **DEM-Engine** | BSD-3 | Granular DEM for offline calibration. |
| Commercial excavation (if Tier 3 in scope, budget permitting) | AGX Dynamics (Algoryx) Terrain; Vortex Studio (CM Labs) | commercial | Dig-grade soil interaction; heavy-equipment heritage. |
| ROS2 bridge | compiled-module `godot_ros` **or** `rosbridge_server` (websocket) | — | Module = tighter, maintain a Godot fork; rosbridge = looser, JSON/CBOR overhead on high-rate sensor streams. |

### Known integration frictions
- No native ROS2 in Godot; both bridge options are third-party / low bus factor.
- No URDF/SDF import → rebuild kinematic tree or write a converter.
- Frame mismatch: Godot **Y-up** vs. ROS **Z-up right-handed** (REP-103) — a steady source of TF bugs.
- No `ros2_control` / MoveIt / Nav2 hooks; build sensor noise models yourself.

---

## 12. Open Questions / Charter Dependencies

1. **Excavation forces in scope?** → decides Tier 2 vs. Tier 3. (Default assumption: out → Tier 2.)
2. **Berm-to-spec a scored mission objective?** → raises deposition fidelity bar. (Blast-berm rationale is inference, not confirmed.)
3. **Volatile-bearing (PSR) deployment site?** → gates the entire ice/volatile regime; if yes, polar mechanical properties are a wide-envelope unknown.
4. **ROS2 bridge choice** — compiled module vs. rosbridge — given sensor data rates.
5. **Simulant / dataset for calibration** — which Earth simulant or DEM reference anchors the Tier-2 fit (and the 1g→⅙g correction).

---

*End of specification.*
