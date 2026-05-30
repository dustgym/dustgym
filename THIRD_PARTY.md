# Third-Party Assets

This repository's own code and content are dedicated to the public domain under
**CC0-1.0** (see [`LICENSE`](LICENSE)). The vendored asset below is the exception:
it retains its **own upstream license** and is **not** covered by CC0.

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
