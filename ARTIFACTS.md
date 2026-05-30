# foss_ipex — Artifact Manifest & Independent Verification

**Verified:** 2026-05-30 (independent end-to-end re-run from the repo, assuming nothing works
until proven). Every documented command was executed from a clean state. Outputs that are
machine-generated (samples rasters, viz PNGs, Godot PNGs) were moved aside or checksummed
first to prove they are *freshly* produced by the command, not stale.

**Verdict: ALL GREEN.** 7/7 conservation tests pass; all 5 scenes re-export with masses
matching the manifest; both matplotlib consumers and all 6 Godot layers + headline render
+ smoke test produce non-black, varied PNGs. One cosmetic precision note (caveins on-disk
float32 mass drift) is documented below — not a defect.

---

## Commands run (exact, with exit codes)

| # | Command (cwd = repo root `/home/john/Development/foss_ipex`) | Exit | Result |
|---|---|---|---|
| 1 | `.venv/bin/python -m terrain_authority.tests` | 0 | **7/7 checks PASS** |
| 2 | `.venv/bin/python -m terrain_authority.scenes` | 0 | Re-wrote 5 scenes (masses match), deterministic (crater md5 identical pre/post) |
| 3 | `.venv/bin/python -m terrain_authority.hexviz samples/crater` | 0 | Terminal ASCII crater bowl; min=-0.14434 max=0.1089 m. No file output (by design). |
| 4 | `.venv/bin/python viz/groundtruth_viz.py samples/crater` | 0 | wrote `viz/out/groundtruth_crater.png` |
| 4 | `.venv/bin/python viz/groundtruth_viz.py samples/boulder_field` | 0 | wrote `viz/out/groundtruth_boulder_field.png` |
| 5 | `.venv/bin/python viz/variety_panel.py` | 0 | wrote variety_panel.png + caveins_filmstrip.png + caveins.gif |
| 6 | `godot_sidecar/render_layers.sh -- --scene ../samples/crater --layers terrain,clasts --pose 2.56,3.0,6.4,2.56,-0.1,2.56 --size 1024x768 --out layer_3_terrain.png` | 0 | wrote layer_3_terrain.png (logged "0 clasts") |
| 6 | `... --scene ../samples/boulder_field --layers terrain,clasts ... --out boulder_terrain_clasts.png` | 0 | "placed 186 clasts"; wrote boulder_terrain_clasts.png |
| 6 | `... --scene ../samples/crater --layers heightmap ... --out layer_1_heightmap.png` | 0 | false-color heightmap |
| 6 | `... --layers state ... --out layer_2_state.png` | 0 | false-color state enum |
| 6 | `... --layers terrain,clasts ... --out layer_4_clasts.png` (crater) | 0 | "0 clasts" -> equals terrain (documented) |
| 6 | `... --layers terrain,dust ... --out layer_5_dust.png` | 0 | "8 dust emitters at disturbed cells" |
| 6 | `... --layers terrain,distortion ... --out layer_6_distortion.png` | 0 | "Brown-Conrady radial" post-process |
| 6 | `godot_sidecar/render.sh render_test.tscn` (smoke test, must stay intact) | 0 | wrote cube_on_plane.png — **smoke test still works** |

Notes on Godot stderr: every Godot invocation prints benign ALSA/audio-driver-failed
warnings (headless box has no sound device; Godot falls back to the dummy audio driver).
These do not affect rendering and the process still exits 0.

---

## D1a — Foundation: physics authority + state-field producer

Pure NumPy Tier-2 surrogate emitting the frozen INTERFACE.md on-disk contract.

| Artifact | Size | Producing command | Description | Status |
|---|---|---|---|---|
| `terrain_authority/` (10 modules) | — | (source) | constants, io_fields, column_state, procgen, sandpile, rover, hexviz, scenes, tests | GREEN |
| `samples/flat_compact/` | 1.39 MB | `python -m terrain_authority.scenes` | 256x256, total_mass **6298.5 kg** (exp ~6298). VIRGIN only, near-zero relief. | GREEN |
| `samples/rolling_hills/` | 1.45 MB | scenes | total_mass **6500.8 kg** (exp ~6501). fbm loose top, disturbance≤0.02. | GREEN |
| `samples/crater/` | 1.30 MB | scenes | total_mass **4606.2 kg** (exp ~4606). EXCAVATED bowl (label 0..2), disturbance to 0.508. | GREEN |
| `samples/boulder_field/` | 1.48 MB | scenes | total_mass **5819.2 kg** (exp ~5819). **186** Golombek-SFD clasts in metadata. | GREEN |
| `samples/crater_caveins/` | 109 MB | scenes | **102** frame snapshots (t000..t101), cadence 4 steps. Ridge slumps peak **+1.90 m -> +0.25 m** at repose. | GREEN |
| `samples/*/preview_*.png` | 21–123 KB ea | scenes | Optional human previews (hillshade/height/state/disturbance); not in Godot hot path. Verified varied. | GREEN |
| Terminal hexviz (no file) | — | `python -m terrain_authority.hexviz samples/crater` | Dependency-free ASCII heightmap; crater bowl clearly legible. | GREEN |

**Conservation invariants (spec §10), independently re-confirmed:**
- Total mass constant across cut->dump->relax, rel_drift **2.99e-16**.
- `height == datum + mass_areal/density` after every op, in-memory max_err **0.0**; on the
  re-exported on-disk rasters the (h - mass/density) spread is **≤6e-8 m** (float32 precision).
- Sandpile relaxation conserves mass (rel_drift 1.75e-16) and leaves all loose slopes
  ≤ θ_r (max 35.57° vs 35.00°, within 1° tol).
- save/load round-trip preserves dims/dtype/row-major.

## D1b — Native matplotlib 3D ground-truth visualizer

| Artifact | Size | Producing command | Description | Status |
|---|---|---|---|---|
| `viz/groundtruth_viz.py` | 13 KB | (source) | 3D bar3d cuboids by state_label + quadtree wireframes + clast scatter. Pure consumer (imports `io_fields.load_scene`). | GREEN |
| `viz/out/groundtruth_crater.png` | 282 KB | `viz/groundtruth_viz.py samples/crater` | Crater bowl, EXCAVATED-vs-VIRGIN coloring, quadtree boxes, legend. extrema full 0..255. | GREEN |
| `viz/out/groundtruth_boulder_field.png` | 392 KB | `viz/groundtruth_viz.py samples/boulder_field` | VIRGIN cuboids + 186 clast spheres + quadtree. extrema full. | GREEN |
| `viz/out/groundtruth_crater_turn0.png` | 229 KB | (prior `--turntable` run; not regenerated this pass) | Turntable azim -80. Varied. | GREEN |
| `viz/out/groundtruth_crater_turn1.png` | 282 KB | (prior `--turntable` run) | Turntable azim -55. Varied. | GREEN |
| `viz/out/groundtruth_crater_turn2.png` | 288 KB | (prior `--turntable` run) | Turntable azim -30. Varied. | GREEN |

## D2 — Godot render-only sidecar (lunar-lit terrain + clasts)

| Artifact | Size | Producing command | Description | Status |
|---|---|---|---|---|
| `godot_sidecar/` (gd + gdshader + tscn + sh) | — | (source) | INTERFACE.md GDScript loader, ArrayMesh active-zone + far-field LOD, lit regolith shader. | GREEN |
| `godot_sidecar/out/layer_3_terrain.png` | 221 KB | render_layers.sh `--layers terrain,clasts` (crater) | Lit crater under ~5° sun, deep shadow + cast rim shadow. extrema 0..225/216/206. | GREEN |
| `godot_sidecar/out/boulder_terrain_clasts.png` | 323 KB | render_layers.sh `--layers terrain,clasts` (boulder_field) | **Headline:** 186 sphere clasts, long grazing-sun shadows. "placed 186 clasts". extrema full. | GREEN |
| `godot_sidecar/out/cube_on_plane.png` | 22 KB | `render.sh render_test.tscn` | Pre-existing smoke test, **left intact, re-confirmed exit 0**. | GREEN |

## D3 — Procgen variety panel + cave-in showpiece

| Artifact | Size | Producing command | Description | Status |
|---|---|---|---|---|
| `viz/variety_panel.py` | 14 KB | (source) | Generator (matplotlib Agg). Pure consumer via `load_scene`. | GREEN |
| `viz/out/variety_panel.png` | 1.18 MB | `viz/variety_panel.py` | 2x2 grazing-sun hillshade: flat_compact / rolling_hills / crater / boulder_field (186 clast markers). | GREEN |
| `viz/out/caveins_filmstrip.png` | 372 KB | `viz/variety_panel.py` | 6-frame slump filmstrip, cadence from parent metadata; ridge +1.90 -> +0.25 m, mass conserved. | GREEN |
| `viz/out/caveins.gif` | 867 KB | `viz/variety_panel.py` | Optional 30-frame animated GIF of the relaxation. | GREEN |

## D4 — Godot layer-toggle compositor (diagnostic layers)

Same crater camera pose `2.56,3.0,6.4,2.56,-0.1,2.56`, 1024x768, unless noted.

| Artifact | Size | Producing command | Description | Status |
|---|---|---|---|---|
| `godot_sidecar/sidecar.gd` / `sidecar.tscn` / `render_layers.sh` | — | (source) | `--scene/--pose/--layers/--out/--size` CLI; composes 6 layers. | GREEN |
| `godot_sidecar/out/layer_1_heightmap.png` | 130 KB | `--layers heightmap` | Unlit false-color elevation ramp. extrema 0..255/255/242. | GREEN |
| `godot_sidecar/out/layer_2_state.png` | 6.3 KB | `--layers state` | False-color enum: grey VIRGIN + amber EXCAVATED. Small (flat enum regions compress hard) but **3 distinct color regions present** (extrema 3..253 / 3..242 / 4..188), semantically correct. | GREEN |
| `godot_sidecar/out/layer_3_terrain.png` | 221 KB | `--layers terrain,clasts` | (see D2) Lit terrain. | GREEN |
| `godot_sidecar/out/layer_4_clasts.png` | 221 KB | `--layers terrain,clasts` (crater) | Crater has **0 clasts** in metadata, so this correctly equals terrain (sidecar logs "0 clasts"). The real clast demo is `boulder_terrain_clasts.png`. **Expected, documented behavior.** | GREEN |
| `godot_sidecar/out/layer_5_dust.png` | 230 KB | `--layers terrain,dust` | Ballistic GPUParticles3D, lunar g. "8 dust emitters at disturbed cells". | GREEN |
| `godot_sidecar/out/layer_6_distortion.png` | 190 KB | `--layers terrain,distortion` | Brown-Conrady radial barrel-warp post-process. | GREEN |

---

## PNG sanity (PIL getextrema + non-black fraction)

Every produced PNG was loaded with PIL, converted RGB, and checked: **none has all-constant
channels**; all have meaningful non-black content (47%–100%). No black/uniform frames.

## Issues found

1. **(Cosmetic, NOT a defect) caveins on-disk mass drift.** The test/manifest report
   "mass_drift 0.00e+00" — that is the *in-memory float64* drift. On the re-exported
   *on-disk float32* rasters, `t000` mass = 4525.5913 kg vs `t101` = 4525.5903 kg, i.e.
   **9.8e-4 kg (~1 mg) over 4525 kg = rel 2.2e-7**. This is exactly float32 storage
   precision (the `.rf32` contract stores `<f4`), not a conservation-logic error. The
   in-memory invariant is genuinely 0. Worth a one-line note in the parent metadata so the
   "drift 0.0" claim is unambiguous about which representation it describes.
2. **layer_4_clasts.png on the crater scene is identical to layer_3_terrain** because the
   crater scene has 0 clasts. This is documented and the sidecar logs it; the genuine
   186-clast demo is `boulder_terrain_clasts.png`. Flagged only so a reviewer isn't
   surprised by the duplicate.

No blocking issues. All documented commands run clean from a fresh checkout.
