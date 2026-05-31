# Sun-Sweep Manifest Contract (`sun_sweep/1.0`)

**Status:** FROZEN (L0 contract artifact). Authored before the producer exists so the
A2-sweep lane (`godot_sidecar/sun_sweep.gd` + `godot_sidecar/boulder_manifest.gd`,
dispatched by `--sun-sweep`) can implement against a fixed shape, and so downstream
consumers (perception / shadow-hazard demos) can read it without coupling to the
producer.

This document freezes:

1. the on-disk manifest layout and `manifest.json` schema,
2. the `time_delta_s -> (azimuth_deg, elevation_deg)` lunar-day sun **model** (a
   documented assumption, **not** a mission value — see the warning below),
3. the boulder source-of-truth (scene `metadata.json` `clasts[]`).

It does **not** introduce any new sensor-bridge fields; it reuses the existing pose
serialization (`sidecar.gd` `_pose_dict()`) and the existing scene metadata. Coordinate
frames are Godot frame throughout (`frame_convention: "godot"`); REP-103 conversion
remains C1's job (`frames.py`) and is out of scope here.

---

## 1. On-disk layout

```
out/sun_sweep/<scene>/
  manifest.json            # the single index described in §2
  000.png                  # rendered frame 0  (image filename referenced per-frame)
  001.png
  ...
  NNN.png
```

- `<scene>` is the scene name (the directory name under `samples/`, e.g.
  `crater_boulders`, `boulder_field`).
- Frame images are zero-padded 3-digit, starting at `000`, matching the
  `frame_index` (`000.png` <-> `frame_index: 0`). This mirrors the multi-frame
  egress `<NNN>` convention (sensor-bridge contract §7) so tooling that walks one
  also walks the other.
- The image **filename** each frame uses is carried explicitly in the manifest
  (`frames[].image`) so a consumer never has to reconstruct it from `frame_index`.

---

## 2. `manifest.json` schema

```jsonc
{
  "schema_version": "sun_sweep/1.0",
  "scene": "crater_boulders",
  "frame_convention": "godot",          // Godot axes; REP-103 stays C1's job (frames.py)

  "cadence": {                          // the sweep parameters that generated the frames
    "time_delta_s_step": 88560.0,       // seconds advanced between consecutive frames
    "az0": 215.0,                       // azimuth at frame 0   [deg]
    "az1": 352.475,                     // azimuth at frame N-1 [deg] (realized §3 output, n=12, step=88560s)
    "el0": 0.0,                         // elevation at frame 0   [deg]
    "el1": 6.079,                       // elevation at frame N-1 [deg] (realized; within the 0–7° grazing band)
    "n": 12                             // number of frames (== len(frames))
  },

  "frames": [
    {
      "frame_index": 0,                 // monotonic int from 0; matches 000.png
      "time_delta_s": 0.0,              // seconds since sweep start (frame 0 == 0.0)
      "sun": {                          // result of the §3 model at this time_delta_s
        "azimuth_deg": 215.0,
        "elevation_deg": 0.0
      },
      "image": "000.png",              // frame image filename, relative to this manifest

      "boulders": [                     // one entry per scene clast (source-of-truth §4)
        {
          "id": 0,                      // clasts[].id (stable across frames)
          "center_m": [3.8963, 0.0406, 3.25],  // clasts[].center_m (Godot frame, copied)
          "radius_m": 0.0217,           // clasts[].radius_m (copied)
          "world_pos": [3.8963, 0.0406, 3.25], // pose position (Godot frame); see §5
          "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0], // pose orientation; see §5
          "buried_frac": 0.223,         // clasts[].buried_frac (copied)
          "shadow": null                // frame 0 has el<=0 -> no resolvable shadow (§6).
          // A non-zero-elevation frame instead carries an object, e.g.:
          //   "shadow": { "azimuth_deg": <sun_az + 180>, "length_m": <2*r*(1-buried)/tan(el)> }
        }
        // ... one per clast
      ]
    }
    // ... n frames
  ]
}
```

### Field rules

- `schema_version` is the literal string `"sun_sweep/1.0"`.
- `frame_convention` is the literal string `"godot"` and MUST NOT change (sensor-bridge
  contract §8: REP-103 conversion is C1's responsibility, not the producer's).
- `cadence` records the sweep that generated the frames so a consumer can reproduce or
  describe it without re-deriving from the per-frame samples. `cadence.n` MUST equal
  `len(frames)`. `az0/el0` MUST equal `frames[0].sun.{azimuth_deg,elevation_deg}`;
  `az1/el1` MUST equal `frames[n-1].sun.{...}`.
- `frames[].frame_index` is a monotonic `int` starting at `0` (no gaps), and the
  3-digit zero-padded form is `frames[].image` (`000.png` for `frame_index: 0`).
- `frames[].time_delta_s` is seconds since sweep start; `frames[0].time_delta_s == 0.0`.
- `frames[].sun` is exactly the §3 model evaluated at this `time_delta_s`. It carries
  `azimuth_deg` and `elevation_deg` only (degrees), matching the units used by the
  sidecar sun members (`_sun_azim_deg` / `_sun_elev_deg`, `sidecar.gd:105-106`).
- `frames[].boulders[]` carries one entry per scene clast. `id`, `center_m`,
  `radius_m`, `buried_frac` are copied verbatim from the scene metadata `clasts[]`
  (§4). `world_pos` + `quaternion_xyzw` are the boulder pose (§5). `shadow` is the
  per-frame cast-shadow estimate, or `null` when the boulder casts no resolvable shadow
  at this sun (e.g. fully buried, or sun below the local horizon).

---

## 3. The `time_delta_s -> (azimuth, elevation)` lunar-day sun model

> **DOCUMENTED ASSUMPTION — NOT A MISSION VALUE.**
> The real lunar polar sun rate (and the site-specific azimuth/elevation track) is a
> non-public mission parameter (per the LAC-eval note). The model below is a deliberately
> simple, **cited** stand-in so the sweep produces a plausible grazing-light progression.
> It exists to drive a *perception* demonstration (new hard shadows appearing as the sun
> moves), not to predict an actual lunar-day ephemeris. Treat the numbers as illustrative.

### Cited constants / sources

- **Grazing polar elevation band: `0°–7°`.** Source of truth:
  `terrain_authority/constants.py:46` — `SUN_ELEVATION_DEG_POLAR = 7.0`, documented there
  as the `[FIXED]` spec §5.1 polar sun-elevation band (0–7°). The sidecar's live default
  grazing sun is `_sun_elev_deg = 5.0` (`sidecar.gd:105`), inside this band. The sweep's
  elevation oscillates within `[0, SUN_ELEVATION_DEG_POLAR]`.
- **Azimuth reference.** The sidecar's live default azimuth is `_sun_azim_deg = 215.0`
  (`sidecar.gd:106`); the sweep's azimuth is anchored to this default at `time_delta_s = 0`.
- **Lunar day length: ~29.53 days (~708 h).** The lunar synodic period (one sunrise-to-
  sunrise cycle as seen from a fixed point), ~29.53 Earth days ~= 708.7 hours ~= 2.551e6 s.
  This is a standard published astronomical value (lunar synodic month), used here only to
  set the azimuth sweep rate; it is NOT claimed to be the site's true solar track.

### Model definition (Godot frame, degrees)

Let `T_SYNODIC_S = 2.551e6` (~29.53 d ~= 708 h) and `EL_MAX = SUN_ELEVATION_DEG_POLAR = 7.0`.

- **Azimuth** advances linearly, `360°` per synodic period, anchored at the sidecar default:

  ```
  azimuth_deg(t) = (AZ0 + 360.0 * t / T_SYNODIC_S) mod 360.0
  ```

  where `AZ0` is the sweep's start azimuth (defaults to `_sun_azim_deg = 215.0`).

- **Elevation** oscillates in the grazing polar band `[0, EL_MAX]` — a raised half-cosine
  so it stays at or above the horizon (a true polar low-sun grazing band; the sun skims
  the horizon rather than climbing high):

  ```
  elevation_deg(t) = 0.5 * EL_MAX * (1 - cos(2*pi * t / T_SYNODIC_S))
  ```

  This gives `elevation_deg(0) = 0°` (sun on the horizon) rising toward `EL_MAX = 7°` at
  the half-period, never exceeding the documented polar band.

> **Why this model and not a real ephemeris:** see the warning above. The point of the
> sweep is that as `azimuth_deg` rotates and `elevation_deg` skims the 0–7° band, each
> uncovered/exposed boulder casts a *new* long hard shadow — the spec §6/§8 loop-closure
> perception payoff ("new hard shadow at grazing sun -> deceptive perception feature").
> The model just has to move the light plausibly through the grazing band; it does not
> have to be metrologically correct, and this document states that plainly.

The A2-sweep lane MAY expose `cadence` knobs (`time_delta_s_step`, frame count `n`, and
the `az0/az1/el0/el1` it actually produced) but MUST record the realized values in
`cadence` and MUST keep `frames[].sun` consistent with whatever model parameters it used.

---

## 4. Boulder source-of-truth: scene `metadata.json` `clasts[]`

The boulders in the manifest come from the scene's `samples/<scene>/metadata.json`
`clasts[]` array (the same array the renderer places). Each clast entry has the shape
(verified against `samples/crater_boulders/metadata.json` and
`samples/boulder_field/metadata.json`, both `schema_version: "1.0"`,
producer `"terrain_authority (NumPy Tier-2 surrogate)"`):

```jsonc
{
  "id": 0,                          // int, stable per scene
  "center_m": [3.8963, 0.0406, 3.25], // [x, y, z] in metres, Godot frame (y == height)
  "radius_m": 0.0217,               // bounding sphere radius [m]
  "shape": "sphere",                // clast shape tag
  "buried_frac": 0.223              // fraction of the clast below the surface [0..1]
}
```

Manifest rules:

- The manifest copies `id`, `center_m`, `radius_m`, `buried_frac` **verbatim** per
  boulder; it MUST NOT recompute or re-place clasts. `shape` is informational and MAY be
  omitted from the manifest (the contract does not require it).
- The number of `boulders[]` entries per frame equals `len(clasts)` for the scene; the
  set of `id`s is constant across all frames (boulders don't move during a sun sweep —
  only the light moves).
- Scenes with no `clasts[]` (e.g. `flat_compact`) yield `boulders: []`.

---

## 5. Boulder pose (`world_pos` / `quaternion_xyzw`)

Boulder pose MUST be serialized with the existing `sidecar.gd` `_pose_dict()` helper
(`sidecar.gd:808-814`), which returns:

```gdscript
{ "position_m": [p.x, p.y, p.z], "quaternion_xyzw": [q.x, q.y, q.z, q.w] }
```

In the manifest the position is named `world_pos` (it is the boulder's world transform
origin in the Godot frame) and the orientation is `quaternion_xyzw`. For the current
`shape: "sphere"` clasts the position equals `center_m` and the orientation is identity
(`[0, 0, 0, 1]`); when the producer instantiates a posed boulder node, `world_pos` and
`quaternion_xyzw` MUST come from `_pose_dict(boulder.global_transform)` so the pose is
consistent with every other pose the sidecar emits. Frame is Godot; REP-103 conversion
stays C1's job (`frames.py`) — this manifest never converts.

---

## 6. Shadow estimate (`frames[].boulders[].shadow`)

`shadow` is a per-frame, per-boulder cast-shadow descriptor used by the perception demo:

- `azimuth_deg`: the compass direction the shadow points (Godot azimuth, degrees) —
  opposite the sun azimuth for a point on a flat local surface.
- `length_m`: shadow length on the local ground plane [m]. For a flat plane and a clast of
  exposed height `h_exposed = 2 * radius_m * (1 - buried_frac)`, a first-order estimate is
  `length_m = h_exposed / tan(elevation_deg)` (degenerating to a long/undefined shadow as
  `elevation_deg -> 0`; the producer MAY clamp).

`shadow` is `null` when no shadow is resolvable: `buried_frac >= 1.0` (nothing exposed),
or `elevation_deg <= 0` (sun at/below the local horizon). The exact estimator is the
A2-sweep lane's choice; this contract fixes only the field names, units, and the null
rule. The headline behavior the field exists to capture: as the §3 sun model advances,
`length_m` lengthens and `azimuth_deg` rotates, so a newly exposed boulder produces a
*new* long grazing-sun shadow (spec §6/§8 loop-closure perception payoff).
