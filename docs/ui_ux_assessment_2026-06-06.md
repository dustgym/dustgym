# UI/UX assessment — "everything to plan a mission?" + "production-worthy for NASA?" (2026-06-06)

Honest evaluation of the `planet_browser` cockpit (grounded in the live `scripts/ui_eval.py` screenshots,
docs/ui_eval_2026-06-06.md). Not a cheerleading review.

## Verdict (two questions, two answers)

1. **Does an engineer have everything to plan a (basic, single-rover) construction mission? — YES, end-to-end.**
2. **Is this a production-worthy UI/UX for an operational NASA mission? — NO, not yet.** It is a strong
   **research / demo cockpit** (TRL ~3-4 ground tool), genuinely capable and honest, but it lacks the
   authoring depth, operational-constraint modelling, live execution, and ground-system UX/process a NASA
   operational planner requires.

## What an engineer CAN do today (the complete basic loop — verified in the eval)

Select body + imagery layer → pick a site (globe lat/lon → DEM origin, or auto flattest-anchor) → author build
orders (manual cut/fill + 8 mass-balanced structure templates + from-pad/berm) → set the fleet (vehicle,
tools, soil override, rover count) → add keep-out obstacles → choose algorithm × objective (+ precedence) →
**Plan** → get a mission-control **PDF + markdown report**, an **authority-validated** feasibility + **as-built
acceptance** check, **endurance**, an **autonomy/perception** block, and a machine-executable **Plan IR** →
**Execute + watch** (forecast replay) → **Compare algorithms** (Pareto) → **save/load/export/import** profiles.
The physics, energy (IPEx-grounded), mass balance, and validation behind every number are real and honesty-
tagged. For a basic single-rover earthmoving mission this is a coherent, complete planning workflow.

## Gaps to "production-worthy for NASA" (prioritized)

### A. Authoring depth (the biggest day-1 gap for an engineer)
- **No draw-on-map / click-to-place.** Orders are typed `x,y` in a list; you cannot draw a pad/berm/road on
  the globe or DEM, and the orders are **not rendered at their sites on the map** (they live only in a text
  list). An engineer plans visually; this is the #1 authoring gap (PRD J4/AL4-5).
- **Footprints are scalar → axis-aligned squares.** No polygon / corridor / oriented-rectangle footprints
  (a 15×2 m road becomes a square).
- **No edit-after-add** (only reorder/delete); no goal-level spec ("pad to ±2 cm, you sequence it") — only
  action-level cut/fill (the planner *checks* flatness post-hoc, can't *generate* to a tolerance).

### B. Operational realism the plan can't express
- **No time-windows / deadlines / op-windows** (sun / thermal / comms) as *constraints* — endurance *reports*
  the sun window but the optimizer never respects it.
- **No comms / PSR-thermal power scheduling, no contingency / fault / abort planning, no risk presentation**
  (the autonomy block carries σ, but the UI doesn't surface confidence bands prominently).

### C. Live execution (it's a forecast, not operations)
- **"Execute + watch" is a deterministic replay** of the plan's own forecast, not live rover telemetry —
  there is no streaming feed, no live state in, no re-plan-from-current-state (PRD P13).
- **No localization/SLAM in the loop** (P15, the scan-to-DEM registration just started).

### D. NASA-grade UX + process
- **The status line is one dense, cryptic run-on string** (great for an expert author, opaque to a reviewer/
  operator) — e.g. "report ready · cut 29.3t → fill 29.3t · 125.5 MJ · 29 recharges · ... as-built 3.1 cm ✗
  (>2 cm) · autonomy ✓ 29rch/0rpl · ...". Ops UIs need labeled, grouped, units-explicit, scannable status,
  and must **not rely on color/glyph alone** (✓/✗ + color); the report PDF is the only well-structured surface.
- **No plan versioning / history / diff UI** (the Plan IR has a `plan_id` but no audit trail), **no
  requirements traceability** (plan → requirement → verification), **no review/approval workflow**, **no
  undo/redo, no autosave / session recovery**.
- **In-UI units + data provenance are thin** (the report has provenance + `[CALIB]`/`[ASSUMPTION]` tags; the
  live UI status mostly doesn't) — a NASA reviewer needs provenance at a glance.
- **Accessibility unverified** (dark theme, small dense text; colorblind-safety + contrast not audited).
- **The globe is cosmetically Earth-radius** (per-body ellipsoid is a noted refinement) — a domain reviewer
  will notice the Moon rendered on a WGS84 sphere.
- **Robustness/process:** single-tenant batch server; UI itself unauthenticated (API key optional on POST);
  no multi-user, no rate limiting beyond the report lock.

## Honest framing for a NASA audience

What to claim: a validated, conserved-physics, IPEx-grounded **single-rover construction planner** with an
authorable web cockpit, honest validation + as-built acceptance, a machine-executable plan output, and a
reusable Gymnasium/RL substrate — a credible **research / pre-operational planning tool**. What NOT to claim:
an operational ground system. The shortest path to "production-worthy" is, in order: **(1) draw-on-map +
on-map order visualization + edit + goal-level specs** (authoring), **(2) operational constraints**
(time/op-windows/comms/thermal/contingency in the objective grammar), **(3) live execution** (the P13
streaming I/O + P15 localization), and **(4) ground-system UX/process** (structured status with units +
provenance, plan versioning/audit, requirements traceability, accessibility, review/approval, per-body globe).
