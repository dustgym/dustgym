# Procedural Lunar-Construction Challenge Platform — architecture

**Status:** design / north-star (2026-06-02). The umbrella over `rl_construction_design.md` (which is
the *agents* layer). Grounded in the actual foss_ipex sim. Marks what exists vs gap vs research-bet, and
holds the honesty rails so this stays a *validated benchmark*, not a tech demo.

> **Ownership note.** The Tier-2 physics core is John McCardle's `foss_ipex` (PR #1). This platform is a
> research direction built *on top of* that core; anything destined upstream is coordinated with John.

---

## 0. What this is (name it precisely)

Not "a renderer that emits sensor frames." The target is **a procedural lunar-construction challenge
platform**: randomized maps × conserved real physics × construction agents × *authored challenges* ×
scoring, with an optional SimCity-like builder front-end.

**The one-line framing:** it is the **Lunar Autonomy Challenge's missing layer, generalized into a
generator.** The official LAC (JHU APL / NASA) is mapping-only, fixed scenarios, CARLA, with *token*
(cosmetic) deformation. This platform is *procedurally-randomized maps with conserved excavation-grade
terramechanics and authored construction objectives* — exactly the layer LAC omits, turned into an open,
randomizable benchmark/sandbox. That positioning (open construction-autonomy benchmark for the Moon) is
the fundable, publishable artifact.

---

## 1. The product split (non-negotiable)

Two layers that must stay separate:

- **Validated benchmark core** (the science): conserved physics + procedural maps + the declarative
  challenge schema + deterministic scoring. This is what makes it a *benchmark* and what a paper/grant
  rests on.
- **Game / UX shell** (engagement): interactive map, authoring UI, multi-agent visualization, HITL.
  Sits *on top* and may never compromise the core's conservation/determinism/honesty.

If game-feel ever leaks into the physics core, it stops being a benchmark and becomes a toy. Keep the
core headless, deterministic, conserved; render and "game" are consumers of it (the existing frozen
state-field seam already enforces this consumer relationship).

---

## 2. The six layers (what exists vs gap)

| Layer | What it is | In code today | Gap |
|---|---|---|---|
| **L1 Procedural map** | randomized terrain: Neukum craters, Golombek boulders, fbm roughness, real LOLA, PSR/ice, sun geometry | `procgen*.py`, `dem_import.py`, `dem_overlay.py`, `tiles_mosaic.py` — calibrated to real stats; DR via `terramechanics.domain_randomize` | compose into a *map* (multi-feature, km-scale) under one seed |
| **L2 Physics** | conserved mass, slip, sinkage, cut/fill/berm/compact | the Tier-2 authority (`column_state`, `rover`, `slip`) — conserved, calibrated | force-accurate excavation = Tier-3 (offline oracle, FIX-1/2) |
| **L3 Scale / LOD** | tile + stream a big map; pay only for active regions | interaction-keyed quadtree (`quadtree.py`, `refinement.py`); 21 MB-corridor vs 4 GB-dense demo | multi-site (many simultaneous active regions) + many agents |
| **L4 Construction agents** | skill library → build berms/pads/roads | designed in `rl_construction_design.md`; primitives map to existing cut/haul/dump/grade | training the skills + the planner |
| **L5 Challenge/scenario system** | author missions: seed + objective + constraints + scoring | **nothing yet** | **the keystone — build first (§3)** |
| **L6 Game / viz shell** | interactive map, authoring UI, multi-agent | Godot render side exists (`godot_sidecar/`) | the UX/authoring shell (furthest out) |

The substrate (L1-L3) largely exists; L5 is the connective gap; L4 is the research; L6 is the shell.

---

## 3. The challenge schema (the keystone)

A *challenge* is one declarative object. SimCity scenarios, RL curriculum tasks, and LAC-style
competition tasks are all instances of it — building this unifies maps + physics + agents + scoring.

```jsonc
Challenge {
  id, name, difficulty_tier,
  map: {
    seed: int,                              // determinism -> reproducible challenge
    base: "procedural" | "lola:<tile>",     // synthetic or real DEM backbone
    extent_m: [w, h], cell_m,
    dr_envelope: {                          // SOURCED ranges (the honesty tags ARE the DR spec)
      slope_deg:[lo,hi], k_phi:[lo,hi], n_sinkage:[lo,hi], cohesion:[lo,hi],
      slip_c1:[lo,hi], slip_c2:[lo,hi], theta_r:[lo,hi], boulder_k:[lo,hi], hurst:[lo,hi]
    },
    sun: { azimuth_deg, elevation_deg, mission_clock },
    features: { craters:{...}, boulders:{...}, psr_zones:[poly], no_go:[poly] }
  },
  objective: {
    type: "traverse"|"excavate"|"flatten_pad"|"build_berm"|"fill_crater"|"build_road"|"clear"|"compose",
    region: polygon|bbox,
    target_heightmap: ref|parametric,       // H_target for terrain-matching objectives
    spec: { tolerance_m, height_m, slope_max, ... }
  },
  constraints: { max_energy, max_time_steps, max_slip_events, keepout:[poly], payload_kg_limit },
  scoring: {
    primary: "H_error"|"completion"|"map_accuracy",
    terms: { energy:w, time:w, slip:w, quality:w },   // multi-objective
    success_threshold
  },
  eval: { train_seeds:[...], heldout_seeds:[...], deterministic:true }
}
```

This one schema yields: a `traverse` challenge = LAC navigation; `flatten_pad`/`build_berm` =
construction; `heldout_seeds` = a generalization benchmark; `difficulty_tier` + the curriculum ladder =
SimCity-style scenario progression. The generator is `seed -> (map, objective, constraints, scoring)`;
the runner executes an agent against it and emits a scorecard.

---

## 4. The architectural invariant (carried from the agents layer)

**Authoring is declarative; agents command; the conserved physics authority mutates terrain; scoring
reads ground truth.** No agent or challenge writes the DEM directly. Mass conservation, slip coupling,
and determinism are therefore guarantees by construction, which is what makes terrain-matching scores
(`-||H_current - H_target||`) unhackable and challenges reproducible.

---

## 5. Scale plan (the SimCity enabler) + honesty

- **Tiling + interaction-keyed quadtree LOD** is the reason a big map is tractable: cost is O(active
  region), not O(map). Demonstrated as 21 MB resident for a 2 cm corridor vs 3.99 GB for a dense tiling
  of the same 220 m window. That is the SimCity-scale lever, and it already exists in seed form.
- **Multi-site / multi-agent** (several simultaneous build sites + rovers) is the real systems gap:
  multiple active regions, scheduling, shared map state. Tractable, but genuine engineering — not free.
- **Compute split:** most agents run on the **cheap headless authority** (sub-ms/step); the **render**
  (perception/HITL views) is reserved and rate-limited (the 725 ms PNG-egress bottleneck caps live
  camera perception). A map-scale world is mostly headless physics + on-demand rendering.
- **Determinism = reproducible challenges.** Seeded generators + conserved authority → a challenge seed
  reproduces the exact map + dynamics, so leaderboards and held-out evaluation are meaningful.

---

## 6. "Real physics" honesty (the rail that matters most)

At map/game scale, **"real physics" means conserved, calibrated, sensor-faithful Tier-2** — not full
granular DEM everywhere (impossible at scale, and the spec never claimed it). Specifically:
- **Modeled (real, conserved):** mass (exact), loose-over-dense density, slip/sinkage-coupled mobility,
  cut/fill/bulking/berm, sandpile repose, angular clasts, Hapke grazing-sun optics + shadows.
- **Tier-3 oracle (offline only):** force-accurate excavation (drum torque, dig reaction), granular DEM
  — run on representative cuts to *calibrate* Tier-2 (FIX-1/2, euclid), never live at map scale.
- **Not modeled (do not claim):** actuator/tool wear, thermal/power budget over the lunar day, full
  multibody Chrono::Vehicle. Flag these; don't silently zero them in scoring.

This is the project's existing honesty-tag culture applied at platform scale: the claim is "conserved
excavation-grade Tier-2 with a Tier-3 calibration oracle," which is both true and the LAC differentiator.

---

## 7. Why it's novel / fundable (positioning)

| | Official LAC (JHU APL/NASA) | This platform |
|---|---|---|
| Maps | fixed scenarios | **procedural, randomized, seeded** |
| Terrain physics | CARLA, *token* deformation | **conserved excavation-grade Tier-2** (mass, slip, cut/fill) |
| Task | mapping-only (elevation + rock flag) | **+ construction** (excavate / berm / pad / road) |
| Excavation | disabled in the 2024-25 edition | **first-class, mass-conserving** |
| Reuse | one challenge | **a generator + curriculum + held-out benchmark** |

The distinctive contribution is precisely the layer LAC omits: *conserved, mass-balanced,
excavation-grade terramechanics with authored, randomizable construction challenges.* That is an open
construction-autonomy benchmark for the Moon — relevant to ISRU, landing-pad/berm/road autonomy, and
HITL operator training.

---

## 8. Evaluation / leaderboard

Per-challenge scorecard (objective metric + multi-objective terms); **generalization** = performance on
held-out seeds (the DR envelope makes this meaningful); **difficulty tiers** = the curriculum ladder
(waypoint → slip → pile → fill → pad → berm → arbitrary `H_target` → multi-skill mission) as built-in
progression. Agents are comparable because the challenge + seed + scoring are declarative and
deterministic. This is the same scorecard that doubles as the RL reward and the LAC-style competition
metric.

---

## 9. Build roadmap (reachable-now first, gated items sequenced)

- **P0 — now, Tier-2, no new fidelity:** the **challenge schema + procedural map-and-challenge generator
  + deterministic scoring runner** (L5). Wraps the existing generators (L1) + authority (L2). Output: a
  library of seeded, scored challenges (traverse → flatten-pad → berm). Doubles as the RL curriculum
  generator.
- **P1 — agents:** train the skill library against P0 challenges (`rl_construction_design.md`); scripted
  skill-selector → learned options policy.
- **P2 — scale:** multi-site maps + multi-agent on the quadtree LOD (L3); km-scale tiling/streaming.
- **P3 — perception track:** camera-obs + the §10 map channel (observed-map producer + `score_map`);
  shadow-from-relief encoder. Gated on render throughput.
- **P4 — world model + game shell:** JEPA/RSSM latent dynamics for sample-efficiency (L4); the
  SimCity-style authoring/visualization UX (L6).
- **Cross-cutting deferred:** Tier-3 force calibration (euclid PyChrono oracle, FIX-1/2); render-in-loop
  throughput (FIX for camera-scale RL).

---

## 10. Related docs

- `rl_construction_design.md` — the **agents layer** (skills, hierarchy, curriculum, world model).
- `plan_tier2_slip_rl.md` — the **Tier-2 physics build** (Phases 1-4, done) the platform stands on.
- `DEFERRED_FIXES.md` — the gated items (FIX-1/2 oracle calibration; FIX-3 done).
- `docs/ipex-terrain-sim-spec.md` — the original sensor-faithful Tier-2 thesis (the spec).

---

## 11. First concrete step

Build **P0**: a `Challenge` dataclass/JSON schema + a `generate(seed) -> (ColumnState map, objective,
constraints, scoring_fn)` generator over the existing procedural + DR machinery, plus a `run(agent,
challenge) -> scorecard` deterministic runner. Seed it with three challenges spanning the difficulty
ladder (a traverse, a flatten-pad, a build-berm). That is the keystone L5 brick, the "plan challenges"
capability, and the RL curriculum generator — all reachable today on the Tier-2 authority.

---

*Through-line: a declarative challenge over a conserved, procedurally-randomized Tier-2 physics map,
solved by commanding (never terrain-editing) agents, scored deterministically against ground truth —
the LAC's missing construction-and-randomization layer, turned into an open generator, with the game
shell as an optional consumer of the validated core.*
