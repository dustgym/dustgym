# Third-Party Assets

This repository's own code and content are dedicated to the public domain under
**CC0-1.0** (see [`LICENSE`](LICENSE)). The vendored assets below are the exception:
they retain their **own upstream status** and are called out here.

---

## EZ-RASSOR rover mesh

- **File:** `godot_sidecar/assets/rover_base.glb`
- **Derived from:** `packages/simulation/ezrassor_sim_description/meshes/base_unit.dae`
  in [FlaSpaceInst/EZ-RASSOR](https://github.com/FlaSpaceInst/EZ-RASSOR)
- **Transform applied:** Collada (Z-up) → glTF (Y-up), re-origined to ground-contact;
  conversion is reproducible via [`scripts/convert_rover_mesh.py`](scripts/convert_rover_mesh.py).
- **License:** MIT (reproduced verbatim below). The `.glb` is a format conversion of an
  MIT-licensed work and remains under MIT; attribution is retained here.

> **Excluded on license grounds:** EZ-RASSOR's `extra_models/` props (rocks, lander, ISRU
> plant, etc.) are third-party re-hosted art (clara.io / SketchUp Warehouse) with **no
> stated license** and are **not** used anywhere in this project. Clasts/rocks are generated
> procedurally (Golombek SFD) instead.

### EZ-RASSOR MIT License (verbatim)

```
MIT License

Copyright (c) 2019 Sean Rapp, Ronald Marrero, Tiger Sachse, Tyler Duncan, Samuel Lewis, Harrison Black, Camilo Lozano, Christopher Taliaferro, Cameron Taylor, Lucas Gonzalez, The Florida Space Institute, and The National Aeronautics and Space Administration

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## NASA RASSOR rover mesh (3D Resources)

- **Files:** `godot_sidecar/assets/rassor_nasa/rassor.glb` (the original NASA download, Draco-compressed)
  and `rassor_godot.glb` (a decoded + decimated derivative for the vendored Godot, which lacks a Draco
  plugin). The original is a single static `rassor_shrinkwrap` visual mesh (~0.85 × 0.94 × 1.66 m, real
  RASSOR scale per the glTF POSITION accessor), not articulated parts.
- **Source:** NASA Science 3D Resources — [science.nasa.gov/3d-resources/regolith-advanced-surface-systems-operations-robot-rassor](https://science.nasa.gov/3d-resources/regolith-advanced-surface-systems-operations-robot-rassor/).
  Credit: **NASA/Dewey L. Smith; NASA/Jason M. Schuler** (the KSC Swamp Works / IPEx team).
- **Status:** NASA's 3D Resources are published as **"free and without copyright"**
  ([github.com/nasa/NASA-3D-Resources](https://github.com/nasa/NASA-3D-Resources)), subject to NASA's
  media-usage guidelines (attribution; no implied NASA endorsement). A work of the U.S. Government,
  treated as public-domain / CC0-compatible for this CC0-1.0 repo. The credit + no-endorsement notice
  ride along here per those guidelines.
- **Use:** RASSOR is the precursor to the modelled IPEx; this mesh is a static visual body for the
  `rassor2` vehicle (the RASSOR-2.0 entry in `terrain_authority/vehicles.py`), not the articulated
  per-part assembly the IPEx/EZ-RASSOR render path uses.

---

## LOLA south-pole DEM tile (PGDA Product 78)

- **File (committed):** the cropped 10 km @ 5 m Haworth sample tile under `samples/lunar_dem/` (a pixel-window crop of `Haworth_final_adj_5mpp_surf.tif`). The full 30 km source raster is **not** committed (lives gitignored under `.vendor/lola_raw/`).
- **Source:** NASA GSFC PGDA, *LOLA 5 m/px South-Pole DEMs* — [pgda.gsfc.nasa.gov/products/78](https://pgda.gsfc.nasa.gov/products/78), `data/LOLA_5mpp/Haworth/`.
- **Status:** a **work of the U.S. Government** (NASA GSFC). PGDA publishes **no formal license string**; under the general principle that U.S. Government works are not subject to domestic copyright, the tile is treated as **public-domain / CC0-compatible** for inclusion in this CC0-1.0 repository. This rests on that principle, **not** on a published CC0 license — stated honestly.
- **Frame / datum:** south polar stereographic, MOON_ME (DE421), R = 1737400 m (IAU_2015:30135); Z = surface height above the 1737400 m sphere in metres. **Cite** Barker et al. 2021 (Planet. Space Sci. 203:105119) + Mazarico et al. 2011 (Icarus 211:1066) as scholarly courtesy (see [`papers/CITATIONS.md`](papers/CITATIONS.md)).

> **Not committed (license-segregated):** the higher-detail **2026 Shape-from-Shading** DEMs (Bertone et al. 2026; Zenodo 10.5281/zenodo.17954508) are **CC-BY-4.0, not CC0** — kept reference-only (download script) or, if ever committed, only in a marked CC-BY-4.0 subfolder with an attribution NOTICE. The **Neukum production-function coefficient vector** is cross-checked against MintonGroup/cratermaker, which is **GPL-3.0** (verified 2026-05-31 at github.com/MintonGroup/cratermaker). GPL-3.0 is copyleft, so **no cratermaker code is — or may be — copied into this CC0 repo**; only the numeric coefficients are reused, and those are uncopyrightable scientific facts cited to **Neukum/Ivanov/Hartmann 2001** (the authority), not to cratermaker. cratermaker is therefore *not* a vendored asset here (no code is included).
