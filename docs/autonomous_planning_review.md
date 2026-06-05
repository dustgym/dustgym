# Autonomous-planning review — capabilities, limits, and the multi-vehicle design (2026-06-04)

A focused four-agent review of the autonomous-planning subsystem of the single dustgym software, on the
question the PRD states capabilities for but not ceilings: **what can the planner actually do, where are its
hard limits, and what would single- vs multi-vehicle production planning require, across different
instructions, designs, and site layouts.** Every finding is evidence-backed at `file:line` and run-verified.

## Verdict

The planner solves the **single-rover, cut-fill-balanced, recharge-coupled routing** problem genuinely and
honestly, with real exact baselines, real precedence, and real conserved-authority validation. But its
autonomy is **action-level, single-vehicle, open-loop-replan, and silently capped**: instructions are
enumerated primitives (not goals), there is **zero multi-vehicle planning** (the "learned ≫ greedy
multi-vehicle" claim is unsubstantiated), the closed loop runs against its own model with no fault handling,
and optimality degrades without warning past small trip counts. None of this is dishonest in the code (the
tests, constants, and `[CALIB]` tags all check out) — the PRD simply states the capabilities without their
limits. This review supplies the limits and a multi-vehicle design.

## 1. Single-vehicle planning — capabilities and the ceilings

**Real and solid:** 7 sequencers (auto/nearest/greedy/two_opt/or_opt/lk/brute/held_karp), multi-objective +
Pareto, SOP precedence respected by every sequencer, hazard routing (Dijkstra slope costmap), conserved
authority plan-validation, battery-aware recharge grounded in verified IPEx constants.

**The hard limits (the ceilings the PRD should state):**
1. **No exact solver on the real objective past 7 trips.** `brute` is the only sequencer exact on the chosen
   (simulated, recharge-coupled) objective, and only to `BRUTE_MAX_TRIPS=7`. Held-Karp (`HELD_KARP_MAX_TRIPS=16`)
   is exact on **driving distance only** (it assumes dig energy dominates and is order-independent) and is a
   seed for LK polish on 8-16.
2. **Silent degradation above 16 trips.** `auto` drops to `lk` local search from a nearest seed with **no
   quality bound and no user-facing warning** — a 50-trip plan is presented identically to the exact ≤7 case.
   Held-Karp's DP table is `2^16 x 16`, an ~18-20-trip RAM/time wall.
3. **Infeasible precedence is an unguarded cliff.** A cyclic/unsatisfiable SOP DAG makes `brute` raise
   `ValueError` and `_held_karp` return `[]` → a **silently "successful" 0-trip plan** with zero energy/time.
   There is no acyclicity/feasibility precheck; the precedence test only covers feasible DAGs.
4. **The objective grammar cannot express real constraints** — no deadline/makespan-with-time-windows, no
   sun/comms/thermal windows (K9 ⬜), no soft constraints, no risk/uncertainty term, no per-leg feasibility
   penalty. It optimizes an unconstrained-in-time world.
5. **Routing + validation are 2.5-D, static, grid-coarsened.** No dynamic hazards, no no-go polygons, no 3D;
   `validate_plan` caps the authority grid at 500 cells, validates on a flat synthetic mantle (not the real
   DEM surface), and does **not** check slip-energy realizability.

*Evidence:* `mission_planner.py:226-227,432-468,502-525,564-565,1031-1119`.

## 2. Multi-vehicle — the gap, the design, and the substrate limits

**State: zero.** Both `mission_planner.plan_and_simulate` and `terrain_authority/scheduler_env.py` are
strictly **one-rover / one-drum** (single `self.rc` position, scalar `drum_inventory`, single leg counter;
`action = Discrete(num_regions)` selects which region this one rover visits next). The `vehicles` parameter
exists only to be rejected (`vehicles != 1: raise`). The **"learned ≫ greedy multi-vehicle" claim is
unsubstantiated** — the real M4 result (`beam_search` 24 legs vs greedy 28 / PPO 27) is single-rover trip-leg
*makespan* ordering, not parallelism or conflict. No `n_rover`/`fleet`/`auction`/`cbs`/`deconflict`/task-
allocation code exists anywhere in the repo. (The raise message also still says the scheduler lives in
"roversim" — a stale string to purge.)

**What production multi-vehicle requires (all absent):** task allocation (which rover does which order);
spatial+temporal deconfliction (collision, shared-corridor/work-site contention); shared-resource scheduling
(one charger, one borrow pit, the ISRU plant); coordinated replanning; a fleet instruction/API surface;
heterogeneous fleets.

**The staged design (fits the conserved-authority substrate; learned/search commands, authority mutates):**
- **Layer 1 — Allocation:** `allocate(mission, rovers) -> {rover: [orders]}` *above* `optimize_sequence`.
  Sequential-greedy / regret-insertion bidding (bid = marginal `_simulate` cost via the existing scorer),
  graduating to a market/SSI auction; MILP/VRP as the small-N exact oracle (≤3 rovers). Per-rover order
  subsets then flow through the **unchanged** `_build_trips → precedence → optimize_sequence → _simulate`.
- **Layer 2 — Deconfliction:** prioritized planning over a cell/corridor **reservation table** (reusing the
  `routed_distance` paths) + shared work-site time-windows; CBS fallback when incomplete. Collision is a
  **scheduling constraint, not physics** (see substrate limit below).
- **Layer 3 — Shared resources:** the charger becomes a queued single/k-server (a rover waits → wait is real
  makespan, fixing K8); borrow-pit depletion, drum, and the ISRU plant become locked decrementing resources
  in a fleet `_simulate`.
- **Layer 4 — Coordination:** `run_closed_loop` extended to N shared-world `Belief`s; re-clear allocation on
  recharge / model-error / pit-empty events (AutoNav market re-clearing).
- **Tractability:** 2 rovers — exact VRP oracle viable, the only regime to validate "learned ≫ greedy"
  against an exact baseline. 5 — auction + prioritized planning + queued charger (the charger queue becomes
  the dominant makespan term). 20 — auction + prioritized planning only; contention caps useful fleet size.

**Hard substrate limits (structural, not missing-feature):**
- The per-cell `ColumnState` authority has **no multi-body dynamics** — two rovers in one cell is mass-legal
  but physically meaningless, so collision can only be a planning constraint, never a simulated event.
- The single global **drum is a scalar** — N rovers need N drum states or an authority refactor.
- The **exact-solver caps are per-rover and small** — joint exact allocation+routing blows past brute-7 /
  Held-Karp-16 immediately, so "optimal multi-vehicle" is a 2-3 rover / ≤~10-order claim only.
- The **single flat charger model (K8)** makes any current multi-vehicle makespan optimistic until queued.

## 3. Instruction / design / layout expressiveness — the limits

- **Action-level only, not goal-level.** The user enumerates every `cut`/`fill` and depth; there is no
  "build a flat pad to ±2 cm, you sequence it." The goal-level `Challenge.objective`+`tolerance` schema and
  the action-level `Mission` schema are **disconnected** — the planner never consumes the goal schema.
- **The J4 grammar gap.** `mission_from_dict` accepts only `name/body/orders/charger/date/precedence`;
  `budget`, `scoring`, `priority`, `keepout` are **silently dropped**. `BuildOrder.footprint_m2` is a scalar
  area rasterized as an **axis-aligned square** — a 15x2 m road becomes a 5.48 m square; no shape, orientation,
  corridor, or polygon.
- **No structure composition or user-defined structures** — 8 hard-coded templates, hard-coded borrow offsets;
  no multi-tier, curved, graded-slope, ramp, retaining, or benched designs.
- **No as-built acceptance (I11 ⬜).** `validate_plan` checks mass conservation + the **center cell's** slope
  on a **flat synthetic mantle**, never the built structure vs spec (flatness-RMSE, berm profile, repose).
- **Coordinate disconnect (M11).** Order `x,y` anchors to the auto-found flattest patch, not the user's globe
  lat/lon pick; one tile per mission; multi-site (E2) unsupported.

## 4. Closed-loop autonomy — the limits

`run_closed_loop` is a real AutoNav skeleton (Kalman belief over pose/energy/drum, reserve-aware recharge,
re-sequence on recharge) but it is **open-loop-replan over a self-simulator**: it executes its own energy
model (not telemetry, not perception), **battery is the only replan trigger**, there is **no fault detection
or handling** (mid-leg entrapment / failed dig / empty pit are unrepresentable), and on dig-dominated missions
the estimator buys ~nothing (slip = 0.01% of total) while pose σ runs open to **11.5 m** by dead reckoning
without the (Godot-gated) perception fix. Single-vehicle by construction. Single→multi blocks: shared belief,
deconflicted execution, and fleet FDIR all absent (and fleet FDIR needs the fault layer first).

## What this means for the PRD
The PRD states capabilities without ceilings. This version adds: a **Multi-Vehicle (MV) area** (MV1-MV7, with
the design above and the explicit "the multi-vehicle scheduler does not exist; the learned≫greedy result is
single-rover" correction); **explicit single-vehicle planning limits** (the silent-degradation and
infeasible-precedence cliffs); the **J4 goal-level grammar + non-square footprints + I11 acceptance**
requirements; and the **autonomy limits** (self-sim, no-fault, single-vehicle). These are the ceilings on
autonomous planning, single and multiple vehicle.
