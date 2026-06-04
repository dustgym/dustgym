# Deferred fixes (oracle-dependent) — foss_ipex Tier-2 RL build

Two items are implemented as **mechanisms with sourced DIRECTION but `[CALIB]` MAGNITUDES** that can only
be *quantitatively* fit against the Chrono::GPU DEM controlled load-sweep oracle (plan Phase 0.3, which
needs PyChrono on euclid — not installed on this host). These are honest "blocked, need X" items, not
stubs: the code runs, is mass-conserving, and is validated on the sourced qualitative behaviour today.

---

## FIX-1 — Quantitative sinkage calibration (K_PHI reconciliation + Lyasko magnitudes)

- **Problem.** Two parameter sets disagree: `constants.py` `K_PHI = 820000` (spec §5.2 Apollo-era) vs the
  committed Chrono SCM run's `0.2e6 = 200000` (`scripts/chrono_scm_rover.py:113`), a ~4x gap. And the
  Lyasko reduction fractions in `terramechanics.lyasko_reduce` (`kphi_frac=0.30`, `c_frac=0.30`) are
  `[CALIB]` estimates, not fit to data.
- **Fix.** Run the controlled static load-sweep on euclid (`scripts/chrono/calibrate_sinkage.py`, plan
  Phase 0.3) → `samples/calib/scm_sinkage.json`; least-squares fit `K_PHI`/`K_C` and the Lyasko fractions
  to the SCM table; update `constants.py` / `TerramechanicsParams` defaults.
- **Blocked on.** PyChrono install on euclid (no conda on this host).
- **Validation when fixed.** `test_matches_scm_oracle` tightens from an order-of-magnitude band to
  ≤20% RMS across the sweep.

## FIX-2 — Lyasko sinkage-exponent (n) re-parameterization

- **Problem.** The literature reports low gravity lowering the sinkage exponent `n`, but in the Bekker
  form `p = (k_c/b + k_phi)·z^n` naively lowering `n` at sub-metre sinkage (`p/k < 1`) *decreases* `z` —
  the opposite of the sourced net truth — because `n` is dimensionally coupled to `k_phi`'s units. So
  `terramechanics.lyasko_reduce` defaults `n_frac = 0` (n unchanged) and carries the net sinkage increase
  through the `k_phi` reduction alone.
- **Fix.** Re-parameterize `n` *consistently with* a re-fit of `k_phi` in the new n-units, against the
  same oracle table (FIX-1). Then `n_frac > 0` becomes physically meaningful.
- **Blocked on.** FIX-1 (same oracle dependency).
- **Validation when fixed.** `lunar sinkage > earth` AND `n_lunar < n_earth` hold *simultaneously* against
  the oracle.

## FIX-3 — RESOLVED 2026-06-02 (Gymnasium env_checker + SB3 PPO)

- **Was:** the `gym.Env` path could not be exercised in-sandbox (I had wrongly concluded "no network").
- **Resolved.** The repo's runtime venv (`/mnt/projects/07_runtime_system/venv`) has gymnasium 1.2.2 +
  torch 2.10; SB3 2.8.0 was pip-installed into it. `gymnasium.utils.env_checker.check_env(RoverSimEnv(...))`
  **PASSES with warnings-as-errors** (after bounding `observation_space` to finite ±1e3, commit
  `4dd5c61`). `scripts/demo/train_ppo.py` trains SB3 PPO: untrained 0% goal-reach → trained **100%**
  (40k timesteps), clean learning curve. CEM (`cem.py`) is the pure-numpy fallback that needs no RL lib.
- **Branch reconciliation 2026-06-03:** the ±1e3 obs bound and both scripts (`cem.py`,
  `scripts/demo/train_ppo.py`) were originally only on `feat/rl-training`; a diagnostic found them
  ABSENT on `feat/rl-on-worksite` (so the claim above was false on that checkout). NOW APPLIED to
  `feat/rl-on-worksite`: `rover_env.py` obs bound to ±1e3 (RoverSimEnv re-verified passing strict
  `env_checker` here), and both scripts brought over + import/run-checked (CEM smoke ran). The resolution
  now holds on the working branch, not just `feat/rl-training`.
- **Reproduce.** `PYTHONPATH=<repo> /mnt/projects/07_runtime_system/venv/bin/python scripts/demo/train_ppo.py`.
  Venv quirk (documented): this venv drops user-site when `PYTHONPATH` is set, so SB3 deps must live in the
  venv's own site-packages (`pip install --ignore-installed cloudpickle pandas` was needed).

## FIX-4 — RESOLVED 2026-06-02 (PR #4) — Per-cell deficit-aware DEPOSIT

- **Was.** `ColumnState.cut_to_inventory(mask, mass_per_cell)` took a **per-cell areal field** (a cut
  respects each cell's available excess), but `dump_from_inventory(mask, total_kg)` spread a **scalar kg
  evenly** at loose **spoil density** (bulking). On an uneven deficit the even spread **overshot** cells
  already near target while far cells stayed empty — verified prototyping a cut-haul-fill env (`maxovr` grew
  to 0.15 m while `maxdef` stayed at 0.06 m). The broken haul probe was deliberately not committed.
- **Resolved.** Added the per-cell deposit counterpart to `column_state.py` (PR #4,
  github.com/jmccardle/roversim/pull/4, branch `feat/per-cell-deposit` off `origin/main` 2643fec, commit
  `d58c891`): **`deposit_field(mask, mass_per_cell)`** (mirror of `cut_to_inventory`; mass-conserving,
  drum-scaled) and **`fill_toward(mask, target_height, max_lift_m)`** (raise toward target, never above).
  Key fact exploited: the existing density mix is **volume-preserving**, so a cell's height rises by exactly
  `deposited_areal / spoil_density` regardless of the material already there — exact overshoot-free
  targeting. `dump_from_inventory` is unchanged (still the right primitive for a spoil pile); purely additive.
- **Validation.** `test_deposit_field.py` (7): height-rise identity, no overshoot on an uneven deficit,
  regression contrast vs even-spread dump, mass conservation across a cut→fill cycle, drum-limited scaling,
  bare-cell spoil density, and an **end-to-end cut-haul-fill reaching a raised pad to spec with mass
  conserved** — the `build_berm`/haul primitive even-spread dump could not solve. 67 pytest + 19/19 legacy.

## FIX-5 — Battery thermal derating

- **Problem.** `ipex_specs.battery_energy_j()` uses nominal-temperature capacity (12S ~44 V, 30 Ah →
  1332 Wh). IPEx actuators were qualified at **−35 °C / +40 °C** (TC2); usable pack capacity degrades
  sharply at those lunar-grade extremes, and off-the-shelf cells do not meet that range at all.
- **Fix.** Add a temperature-derating factor to the usable-energy budget (a curve or a conservative scalar)
  once a cell/pack thermal spec is available. Currently `[CALIB]`.
- **Blocked on.** A real cell/pack temperature-vs-capacity curve.

## FIX-6 — Lunar Bekker moduli may be double-Lyasko-reduced (surfaced by the bodies sysrev)

- **Problem.** The per-planet systematic review (`roversim/docs/bodies_sysrev.md`) found that the NASA LTV
  terramechanics white paper (NTRS 20220010732) publishes `k_phi = 820,000 N/m^3, k_c = 1400 N/m^2,
  n = 1.0, c = 170 Pa` **as the LUNAR reference values** — i.e. these are *already* lunar, not Earth-era.
  But `constants.py` labels them "Earth/Apollo-era" and `TerramechanicsParams.lunar()` applies an
  **additional** Lyasko 1g→⅙g reduction on top, which would **double-count** the gravity correction.
- **Fix (additive, non-breaking already in place).** `terrain_authority/bodies.py` `params_for_body("moon")`
  uses the sourced NASA lunar values **directly** (no second reduction) — the literature-correct Moon. The
  open question is only whether `lunar()` itself should drop the extra reduction (changes legacy behavior),
  or whether John intends `constants.py` as a true Earth baseline. **Flagged for John; `lunar()` unchanged.**
- **Blocked on.** John's intent for the `constants.py` baseline (Earth vs lunar). No data needed — a
  modeling-decision clarification.

---

*FIX-1/FIX-2 remain (shared euclid PyChrono load-sweep oracle, plan Phase 0.3). FIX-3 done. FIX-4 done
(per-cell deposit, PR #4). FIX-5 (battery thermal derating) blocked on a real cell thermal curve. FIX-6
(lunar Bekker double-Lyasko) flagged for John, sidestepped in bodies.py. The Tier-2
surrogate is parameter-consistent and qualitatively validated; the RL env is validated (env_checker +
converging PPO); and the energy model (K2) is now grounded in real IPEx data (`terrain_authority/ipex_specs.py`,
Schuler ASCEND 2024 + the 12S/30Ah pack) rather than arbitrary coefficients.*
