# Lane A2-sweep — lunar-day sun-sweep + boulder manifest

Renders a polar boulder field under a sun swept across the lunar day and emits a
`sun_sweep/1.0` dataset manifest (per-frame sun + per-boulder cast-shadow) for the
visual-fiducial experiment. Owns `godot_sidecar/{sun_sweep.gd, boulder_manifest.gd}`.

## Run

```bash
cd godot_sidecar
xvfb-run -a ./render_layers.sh -- --scene ../samples/boulder_field --sun-sweep --stride 12
```

> **`--stride` sets the frame count `n`.** Plain `--sun-sweep` (no `--stride`) inherits
> the sidecar's default stride of **2** → only **2 frames**. Pass `--stride 12` for the
> 12-frame grazing slice. (`n = _seq_stride if ≥2 else DEFAULT_N=12`.)

## Outputs
`godot_sidecar/out/sun_sweep/<scene>/{manifest.json, 000.png … NNN.png}` (gitignored).

## Sourced model & realized sweep (DOCUMENTED ASSUMPTION — not an ephemeris)
- azimuth(t) = (215° + 360·t/`T_SYNODIC_S`) mod 360, `T_SYNODIC_S = 2.551e6 s`
- elevation(t) = ½·`EL_MAX`·(1 − cos(2π·t/`T_SYNODIC_S`)), `EL_MAX = SUN_ELEVATION_DEG_POLAR = 7.0` (`terrain_authority/constants.py:46`)
- **Realized for n=12, step=88560 s:** azimuth **215° → 352.5°**, elevation **0° → 6.08°**
  (squarely in the grazing band GMRO cares about). The manifest records the *realized*
  `az0/az1/el0/el1`; `cadence.n == len(frames)` and the frame-0/N−1 sun invariants hold.
- Boulders are copied **verbatim** from `clasts[]` (sphere → `world_pos==center_m`,
  identity quat); per-frame shadow azimuth = sun+180°, length = 2·r·(1−buried)/tan(el),
  `null` when buried_frac ≥ 1 or el ≤ 0.
