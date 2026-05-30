# Project Chrono Integration Guide — foss_ipex

**Purpose:** Stand up **Project Chrono** as the real physics-authority producer that replaces the
NumPy analytical surrogate (spec §2 authority model; spec §11 candidate tooling). Chrono must emit
the **exact** on-disk state-field contract (`INTERFACE.md`) so it is a drop-in for the surrogate —
producer and consumer share only the `.rf32`/`.r8` + `metadata.json` seam.

**Status:** Research / setup guide. Class/package/flag names were verified against live Chrono docs
(May 2026); anything unconfirmed is flagged inline.

> **⚙️ Bring-up EXECUTED 2026-05-30 — Path A is live.** This plan has since been carried out on this
> box: a conda `chrono` env (Python 3.12) with **PyChrono 10.0.0** runs an `SCMTerrain` rover at lunar
> gravity headless, and a partial `SCM → INTERFACE.md` exporter (`scripts/chrono_scm_export.py`) writes
> a contract-valid scene via the frozen `io_fields.save_scene`. See **[`chrono_bringup_log.md`](chrono_bringup_log.md)**
> for the full as-built record and `scripts/chrono_scm_rover.py` / `chrono_scm_export_demo.py`.
> **Verified corrections to this guide** (the live 10.0.0 API differs from the sketches below):
> - The conda build `py312h98ab86c_677` links **`libgdal.so.37`**; the default solve pulls GDAL 3.12
>   (`.so.38`) and breaks `import pychrono.vehicle`. Fix: pin **`libgdal=3.11`** alongside pychrono
>   (soname map: 3.9→.so.35, 3.10→.so.36, **3.11→.so.37**, 3.12→.so.38).
> - SCM patch setup is **`SetReferenceFrame(ChCoordsysd(...))`**, *not* `SetPlane` (which does not exist).
>   Run Y-up by rotating the **terrain** frame −90° about X (`QuatFromAngleX`); SCM defaults to Z-up.
> - A bare `SCMTerrain` + body advances inside **`sys.DoStepDynamics(dt)`** — do *not* also call
>   `terrain.Synchronize/Advance` (those belong to the Chrono::Vehicle driver loop).
> - PyChrono 10.0.0 exposes **no `__version__`**; identify the build via `conda list` / the module path.
> - Classic solver (conda 23.3.1) is slow (~20–30 min/solve); pinning pychrono shrinks the search space.

**Scope recall:** This project targets **Tier 2** (spec §3) — coupled semi-empirical terramechanics.
Chrono provides Tier-2 mobility via **Chrono::Vehicle + SCM deformable terrain** and the Tier-3
calibration oracle via **Chrono::GPU** (offline DEM, never in the live loop; spec §10).

---

## 1. Environment audit — THIS box (midlife / Debian 12)

Probed 2026-05-30. Present / missing for a Chrono build or PyChrono install:

| Component | Status | Detail (verified) |
|---|---|---|
| OS | OK | Debian GNU/Linux 12 (bookworm), x86_64 |
| CPU | OK | 12th Gen Intel i5-12400, **12 threads** (OpenMP parallelism for SCM/Vehicle) |
| RAM | OK | **125 GiB total, ~122 GiB available** — far above any Chrono build need |
| Disk | **TIGHT** | `/` is **94% full (27 GiB free of 456 GiB)**. A from-source Chrono build tree + CUDA objects is multi-GiB; a conda env is ~3-5 GiB. **Watch this.** |
| GPU | OK | NVIDIA RTX 4090, 24 GiB, driver 570.86.10 |
| CUDA toolkit | OK | **nvcc 12.8** at `/usr/local/cuda-12.8` — Chrono::GPU is tested on exactly CUDA 12.3/12.8 |
| gcc / g++ | OK | **12.2.0** (Debian 12.2.0-14). Chrono needs gcc ≥ 11 → satisfied. C++17 OK. |
| cmake | OK | **3.25.1** at `/usr/bin/cmake`. Chrono root needs cmake ≥ 3.18 → satisfied. |
| make | OK | present. `ninja` **absent** (optional; `make` is fine, or `apt install ninja-build`) |
| git | OK | present |
| swig | OK | **4.1.0** at `/usr/bin/swig` (only needed for from-source Python bindings) |
| Python (system) | OK | 3.11.2 at `/usr/bin/python3` |
| Project `.venv` | OK | `/home/john/Development/foss_ipex/.venv` — **stdlib venv on system Python 3.11.2** (not conda). numpy 2.4.4, scipy 1.17.1, matplotlib 3.10.9, pillow 12.2.0 present. The surrogate + `io_fields.py` run here. |
| **conda / mamba** | **PARTIAL** | No `conda`/`mamba`/`micromamba` on PATH. **A usable conda exists** at `/home/john/text-generation-webui/installer_files/conda/bin/conda` (**conda 23.3.1**, from text-generation-webui). No standalone miniforge/miniconda/anaconda. `~/.conda/environments.txt` references only that install. **No `mamba`** anywhere. |
| Eigen3 | **MISSING** | No `eigen3` dpkg, no `/usr/include/eigen3`, no `Eigen3Config.cmake`. **Required for any from-source build** (`apt install libeigen3-dev`, or let CMake fetch). Not needed for the conda path (bundled). |
| Irrlicht | **MISSING** | No `libirrlicht`/headers. Optional run-time viz for source build (`apt install libirrlicht-dev`). The **conda PyChrono ships Irrlicht prebuilt**, so Path A gets viz with no system install. |
| VSG (VulkanSceneGraph) | **MISSING** | No `libvsg` (Vulkan **loader** `libvulkan1` + NVIDIA ICD are present, but VSG itself is not). VSG is **not** in the conda build (see Path A). Source-build VSG is a from-scratch dependency chain — skip unless needed. |
| OpenMP | OK | `libgomp1` present (gcc OpenMP). MKL/TBB **absent** (optional; Pardiso-MKL not needed for Tier 2). |
| GL utility / xvfb | OK | `libglu1-mesa` present; **`xvfb` present** (2:21.1.7) — already used for the headless Godot path, reusable for headless Irrlicht. |

**Bottom line on the box:** Ideal for **Path A (conda PyChrono)** — only blocker is that conda must be
made reachable (use the text-gen conda, or install miniforge). Fully capable for **Path B (source)**
including **Chrono::GPU** (CUDA 12.8 + RTX 4090 is a supported configuration), with two caveats:
**(1) install `libeigen3-dev`** (and `libirrlicht-dev` if run-time viz is wanted), **(2) disk is at 94%** — free space before a source build.

---

## 2. Path A — PyChrono via conda (fastest to a running rover)

**Recommended first step.** No compiler dance; a working Chrono::Vehicle + SCM in minutes.

### 2.1 Install

The `projectchrono` channel package is `pychrono`, distributed off `conda-forge` for dependencies.
Latest build (verified on Anaconda.org):

- **Version `10.0.0`**, last updated **2026-03-26**. Platforms: linux-64 (this box), win-64,
  macOS-arm64, linux-aarch64. linux-64 build strings cover **Python 3.12 (`py312`) and 3.13
  (`py313`)** — there is **no py311 linux-64 build of 10.0.0**, so the project `.venv` (Python
  3.11.2) **cannot** host PyChrono. Make a dedicated conda env on 3.12.
- Previous stable: `9.0.1` (2024-12-12).

```bash
# Make the existing conda reachable (it's not on PATH):
eval "$(/home/john/text-generation-webui/installer_files/conda/bin/conda shell.bash hook)"
#   — or install Miniforge fresh if you'd rather not borrow text-gen's conda.

# Dedicated env on a supported Python (3.12 for the 10.0.0 linux-64 build):
conda create -n chrono python=3.12
conda activate chrono

# Latest dev build (per official install doc):
conda install projectchrono::pychrono -c conda-forge
#   — or pin exactly to the verified linux-64 10.0.0 build:
# conda install projectchrono::pychrono=10.0.0=py312h98ab86c_677 -c conda-forge
```

> Note: that base conda is **23.3.1** (June 2024). `conda create` works fine; if solves are slow,
> `conda install -n base conda-libmamba-solver` or install Miniforge for `mamba`. Disk: a fresh env
> is ~3-5 GiB and `/` is at 94% — verify free space first.

Source: [Install PyChrono](https://api.projectchrono.org/pychrono_installation.html) ·
[pychrono on Anaconda.org](https://anaconda.org/projectchrono/pychrono) ·
[file list](https://anaconda.org/projectchrono/pychrono/files)

### 2.2 What the conda build includes / excludes

Verified from the install doc and module references:

- **Included:** core multibody, **Chrono::Vehicle** (wheeled rover), **SCM deformable terrain**
  (`pychrono.vehicle.SCMTerrain`), **Chrono::Irrlicht** run-time viz (prebuilt — no system Irrlicht
  needed), Postprocess, FEA, Sensor (camera/lidar; OptiX bundled in recent builds — **verify it
  imports** on first run, this varies by build).
- **Excluded (install doc states explicitly):** **`cascade`, `vsg3d`, and `ROS` modules are NOT in
  the conda PyChrono.** So: no VSG viz (Irrlicht only), no CAD import, **no Chrono::ROS bridge** in
  conda — ROS coupling needs a source build (consistent with spec §11 "no native ROS2").
- **Chrono::GPU (Tier-3 DEM): NOT in the conda build.** It is a CUDA C++ module enabled only at
  source-build time (see Path B §3.4). Path A gives you **Tier 2 only** — which is exactly the
  project target, so this is fine for the live loop.
- **CUDA:** the install doc lists **no CUDA requirement** for PyChrono. SCM/Vehicle run on CPU
  (OpenMP). The RTX 4090 is irrelevant to Path A except via the Sensor module's optional GPU
  ray-tracing.

### 2.3 Minimal PyChrono script shape — wheeled rover on SCM at lunar gravity

This is the **shape** (API verified against the current `SCMTerrain` class and the
`demo_VEH_SCMTerrain_RigidTire` demo), not a tested program. Confirm exact submodule import paths
against your installed build (`import pychrono; print(pychrono.__version__)`).

```python
import pychrono as chrono
import pychrono.vehicle as veh

# --- system: set Chrono's native up-axis to Y, lunar gravity (see §5 frames) ---
sys = chrono.ChSystemSMC()                 # SMC (penalty) contact — standard for SCM/Vehicle
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))  # Y-up, lunar 1/6 g
# (Equivalent intent to the demo's sys.SetGravityY(); set the vector explicitly to pin -1.62.)

# --- SCM deformable terrain patch (current class: SCMTerrain, NOT SCMDeformableTerrain) ---
terrain = veh.SCMTerrain(sys)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))

# Bekker / Janosi soil parameters — SetSoilParameters(Kphi, Kc, n, cohesion, friction, Janosi_k, elastic_K, damping_R)
# Map from spec §5.2 (note: SCM wants SI Pa-based units; convert kPa->Pa, kN/m^... as Chrono expects):
terrain.SetSoilParameters(
    0.2e6,   # Bekker_Kphi   frictional modulus k_phi  (spec k_phi ~800-820 kN/m^(n+2); CALIBRATE — see §6)
    0.0,     # Bekker_Kc     cohesive modulus k_c       (spec k_c ~1.4 kN/m^(n+1))
    1.0,     # Bekker_n      sinkage exponent n         (spec n ~1.0; LOWER in low-g)
    170.0,   # Mohr_cohesion c  (Pa)                    (spec c ~0.17 kPa = 170 Pa)
    35.0,    # Mohr_friction phi (deg)                  (spec phi 30-50)
    0.018,   # Janosi_shear  K  (m)                     (spec K ~1.8 cm = 0.018 m)
    2e8,     # elastic_K     vertical stiffness before yield (Pa/m) — tune
    3e4      # damping_R     damping vs sink speed
)

# Discretized regular grid (cell ~ spec §4 active-zone 1-3 cm). delta = cell size in metres.
#   flat:      Initialize(sizeX, sizeY, delta)
#   heightmap: Initialize(heightmap_file, sizeX, sizeY, hMin, hMax, delta)
terrain.Initialize(5.12, 5.12, 0.02)       # 5.12 m square, 2 cm cells -> 256x256 (matches INTERFACE sample)

# Optional: bulldozing (accumulated displaced soil) and an active domain to bound ray-cast cost
terrain.EnableBulldozing(True)             # off by default; needed for berm/rut soil accumulation
# terrain.AddActiveDomain(chassis_body, center, ChVector3d(dx, dy, dz))  # spec §4 moving window

# --- wheeled vehicle: build/assemble a Chrono::Vehicle wheeled model here ---
#   Use a JSON-specified WheeledVehicle (veh.WheeledVehicle(sys, json_file)) or one of the
#   Chrono::Vehicle data models. NOTE spec §11: NO URDF/SDF import — author the kinematic
#   tree as Chrono::Vehicle JSON/C++, you cannot load the IPEx URDF directly.
#   Each wheel/tire must be registered so SCM ray-casts against it.

dt = 1e-3
while sys.GetChTime() < t_end:
    # driver/powertrain/vehicle Synchronize(time) ...
    terrain.Synchronize(sys.GetChTime())
    # vehicle.Advance(dt) ...
    terrain.Advance(dt)
    sys.DoStepDynamics(dt)
    # ---> every N steps: extract SCM state and write the INTERFACE contract (see §4)
```

Sources: [SCMTerrain class ref](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_s_c_m_terrain.html) ·
[demo_VEH_SCMTerrain_RigidTire.cpp](https://github.com/projectchrono/chrono/blob/main/src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp) ·
[Terrain models](https://api.projectchrono.org/vehicle_terrain.html)

---

## 3. Path B — C++ Chrono from source (full control, ROS, Chrono::GPU oracle)

Needed for: the **Tier-3 Chrono::GPU DEM calibration oracle** (spec §10), the **Chrono::ROS** bridge
(excluded from conda), VSG viz, or any custom C++ module. Slower to stand up.

### 3.1 System deps to add on this box (do not run yet)

```bash
sudo apt install libeigen3-dev          # MISSING — core dependency
sudo apt install libirrlicht-dev        # MISSING — only if run-time Irrlicht viz wanted
# ninja-build                            # optional faster builds
# VSG: not packaged on bookworm; build from source only if VSG viz is required (skip for now)
```
Already satisfied: gcc 12.2 (≥11 OK), cmake 3.25 (≥3.18 OK), CUDA 12.8 (for GPU), swig 4.1 (for Python
bindings), OpenMP (libgomp). C++17 is the minimum standard.

### 3.2 Configure / build outline

```bash
git clone --recursive https://github.com/projectchrono/chrono.git
# build dir MUST be outside the source tree (Chrono forbids in-source builds)
cmake -S chrono -B build-chrono \
  -DCMAKE_BUILD_TYPE=Release \
  -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_MODULE_IRRLICHT=ON \    # run-time viz (needs libirrlicht-dev); or use VSG
  -DCH_ENABLE_MODULE_GPU=ON \         # Tier-3 DEM oracle — needs CUDA (auto-disabled if CUDA/Eigen too old)
  -DCH_ENABLE_MODULE_POSTPROCESS=ON \
  -DCH_ENABLE_MODULE_PYTHON=ON        # optional SWIG bindings alongside (needs swig 4.x — present)
cmake --build build-chrono -j 12
```

Verified module flags (`CH_ENABLE_MODULE_*`) that exist in the current source tree
(`src/CMakeLists.txt`): VEHICLE, IRRLICHT, VSG, OPENGL, POSTPROCESS, PYTHON, CSHARP, **GPU**, DEM,
FSI (+FSI_SPH/FSI_TDPF), SENSOR, MULTICORE, PARDISO_MKL, MUMPS, MATLAB, MODAL, FEA, CASCADE,
PARSERS, PERIDYNAMICS, FMI, **ROS**, SYNCHRONO. (Toggle with ccmake; press Configure repeatedly until
dependent vars resolve — Chrono's documented workflow.)

- **Chrono::Vehicle** has **no extra mandatory deps** (OpenCRG only for `.crg` roads via
  `CH_ENABLE_OPENCRG`). SCM terrain lives inside Vehicle.
- **Irrlicht vs VSG:** pick one (or both) for run-time viz; both optional. Irrlicht is the
  conda-parity choice and is apt-installable. VSG (Vulkan) gives nicer visuals but is an unpackaged
  source dependency on bookworm — **defer**. For a headless server, neither is needed for the
  producer; if you do open an Irrlicht window over SSH, wrap it in **xvfb** (`xvfb-run …`), the same
  headless trick already used for the Godot path (spec §11 names this trap).
- **OpenMP/MKL:** OpenMP auto-detected (present). Pardiso-MKL (`CH_ENABLE_MODULE_PARDISO_MKL`) is an
  optional faster sparse solver — not needed for Tier 2.

### 3.3 Python bindings from source (optional)

`-DCH_ENABLE_MODULE_PYTHON=ON` builds the SWIG bindings against the **system** Python. swig 4.1.0 is
present. This is only worth it if you need a Python API tied to a custom C++ build (e.g. with
Chrono::ROS). Otherwise **prefer conda PyChrono** (Path A) and keep the source tree for C++/GPU.

### 3.4 Chrono::GPU — the Tier-3 DEM oracle (spec §3, §10)

- **What it is:** a CUDA GPU solver for large granular systems via penalty-based **DEM** — exactly
  the offline excavation/wheel-pass oracle spec §10 wants to fit the Tier-2 Bekker/Janosi/Wong-Reece
  + repose + swell parameters against. **Never in the live loop.**
- **Requires:** CUDA — **source build only, not in conda PyChrono.** Tested on Linux with **CUDA
  12.3 and 12.8** → this box's CUDA 12.8 + RTX 4090 is a directly supported configuration. Uses
  Thrust (ships with CUDA). GPU module auto-disables if CUDA is absent or Eigen < 3.3.6.
- **Flag:** `CH_ENABLE_MODULE_GPU=ON`. (Sibling `CH_ENABLE_MODULE_DEM` / external **DEM-Engine**
  is the newer standalone granular engine — also CUDA, also source/oracle-only.)

Sources: [Install Chrono](https://api.projectchrono.org/tutorial_install_chrono.html) ·
[Vehicle module install](https://api.projectchrono.org/module_vehicle_installation.html) ·
[GPU module install](https://api.projectchrono.org/module_gpu_installation.html) ·
[Installation guides index](https://api.projectchrono.org/install_guides.html) ·
[root CMakeLists](https://github.com/projectchrono/chrono/blob/main/CMakeLists.txt)

---

## 4. Extracting SCM state into the INTERFACE.md contract

The producer must emit `heightmap.rf32`, `mass_areal.rf32`, `density.rf32`, `disturbance.rf32`,
`state_label.r8` (+ optional `ice.rf32`) + `metadata.json`, all `width×height` row-major
(`INTERFACE.md` §1/§2). What SCM gives us, and what it does NOT:

### 4.1 What SCMTerrain exposes (verified API)

| Need (contract) | SCM method | Notes |
|---|---|---|
| Surface elevation at a point | `GetHeight(ChVector3d loc) -> double` | Deformed (current) height. Sample on our grid to build `heightmap`. |
| Undeformed reference height | `GetInitHeight(loc) -> double` | Subtract from `GetHeight` → **net sinkage/heave** per cell (drives `disturbance`). |
| Per-node SCM state | `GetNodeInfo(loc) -> NodeInfo` | **Rich per-node terramechanics state** (fields below). |
| Modified nodes (changed this step / all) | `GetModifiedNodes(bool all=false) -> vector<NodeLevel>` | `NodeLevel = pair<ChVector2i, double>` = (grid i,j) → height. Exactly the deformed heightfield; iterate to fill our raster. |
| Surface normal | `GetNormal(loc)`, `GetInitNormal(loc)` | For shading/derived fields if wanted. |
| Visualization mesh | `GetMesh() -> ChVisualShapeTriangleMesh`, `WriteMesh(file)` (OBJ) | Mesh export — heavier than we need; the grid path above is cleaner. |
| Step statistics | `GetNumRayHits/RayCasts/ContactPatches/ErosionNodes()` | Diagnostics, not field data. |
| Underlying loader | `GetSCMLoader() -> SCMLoader` | Lower-level grid access if `GetModifiedNodes` is insufficient. |

**`NodeInfo` struct fields (verified):** `sinkage`, `sinkage_plastic`, `sinkage_elastic`, `sigma`
(normal pressure), `sigma_yield`, `kshear`, `tau` (shear stress). This is a single-cell Bekker/Janosi
state vector — plenty to derive our fields.

### 4.2 Mapping SCM → contract fields

The clean approach: **drive SCM's `Initialize(sizeX, sizeY, delta)` with the same grid geometry as
`metadata.json`** (e.g. 5.12 m, delta 0.02 → 256×256), so SCM nodes line up 1:1 with our raster
cells. Then per export:

- **`heightmap.rf32`** ← `GetModifiedNodes(all_nodes=True)` heights (or `GetHeight` sampled at each
  cell centre). **Important:** the contract says `height` must be *derived* `mass/density`, never
  authored (INTERFACE §4). With SCM you have two reconciliation options:
  1. Treat SCM's deformed height as authoritative geometry **and** back-solve `mass_areal = height ×
     density` so the invariant still holds numerically (pragmatic; SCM is the geometry source).
  2. Keep our mass-bookkeeping surrogate as the conservation authority and use SCM only for the
     *sinkage delta*. (See §4.4 — this is the likely hybrid.)
- **`density.rf32`** ← SCM is **single-layer, constant-density Bekker**: it does **not** track a
  current bulk density field. Derive density from `NodeInfo.sinkage_plastic` / compaction proxy, or
  keep density evolution in the surrogate's multi-pass model (spec §6 paving). SCM won't hand you ρ.
- **`disturbance.rf32`** ∈ [0,1] ← normalize a per-cell "how worked" proxy: `sinkage_plastic` (or
  `GetHeight − GetInitHeight`) clamped/scaled, or accumulated max-sinkage-ever. This is exactly the
  "max-sinkage-ever / pass-count proxy" the contract suggests (INTERFACE §4).
- **`state_label.r8`** ← derive: cells with nonzero plastic sinkage → `TREAD` (1); cells inside the
  drum zone with removed mass → `EXCAVATED` (2); etc. **SCM has no notion of our discrete labels** —
  this stays producer logic, same as the surrogate.
- **`mass_areal.rf32`** ← **SCM does not conserve or expose areal mass.** This is *our* invariant
  (spec §5.3, INTERFACE §4). It must be maintained by the producer's bookkeeping, not read from SCM.
- **`ice.rf32`** ← not a Chrono concept; producer-owned.
- **Rover pose / clasts** → `metadata.json`: get chassis body pose via Chrono body
  `GetPos()/GetRot()` and convert frame (§5); clast rigid bodies likewise → `clasts[]` entries.

### 4.3 What SCM does NOT give you (limits vs spec §4/§6)

SCM is a **single-layer, semi-empirical Bekker–Wong** pressure-sinkage + Janosi-Hanamoto shear model
on a 2.5D height grid. Confirmed gaps against the spec:

- **No loose-over-dense stratigraphy.** SCM soil parameters are uniform — there is **no z_t density
  transition, no exposed-sublayer depth `z_cut`, no per-column density profile** (spec §4 "stacked
  heightfield", §5.3). The multi-pass *paving* effect (spec §6 "denser, stronger, higher-bearing"
  on passes 2+) is **not** native — SCM's bulldozing reshapes geometry but doesn't model a rising
  density floor. **This stays in the surrogate.**
- **Slip-sinkage θ_m migration** (spec §6, the Spirit-rover entrapment mode, `θ_m=(c₁+c₂·s)·θ_f`) is
  **not** a tunable SCM output — SCM gives shear via Janosi `kshear`/`tau` but does not expose the
  rearward θ_m migration the spec calls "the failure HITL operators most need to recognize." If that
  visual signature must be faithful, it stays a surrogate/derived feature.
- **No mass conservation / bulking (swell factor).** SCM bulldozing moves material heuristically; it
  does not bookkeep cut→inventory→dump in kg (spec §7). The conserved-mass invariant and the
  sandpile/repose cellular automaton (spec §7) are **surrogate-side, always.**
- **Excavation/drum digging** is not SCM's job at all — that's Tier-3 (Chrono::GPU) or the
  surrogate's material-removal model. SCM is the **"Under wheels (rolling)"** zone only (spec §4).
- SCM **does** give: bulldozing/accumulated displaced soil (via `EnableBulldozing` + erosion-domain
  nodes, `GetNumErosionNodes`), per-node plastic/elastic sinkage, normal pressure, shear stress.

### 4.4 Recommended hybrid (the realistic integration)

Let **Chrono own rigid-body dynamics + contact + the SCM rut geometry under the wheels**; let the
**surrogate keep the conserved-mass bookkeeping, density stratigraphy, multi-pass paving, slip-sinkage
visual, sandpile relaxation, and excavation/bulking** (everything SCM structurally cannot do). Each
export step: pull SCM's deformed heights + `NodeInfo.sinkage*` into the rolling zone, fold them into
the surrogate's mass/density grid (re-derive height from mass/density to keep the INTERFACE invariant
green), and write the contract. This makes Chrono a *better wheel-rut producer* while preserving the
spec's mass-is-invariant design — and it is exactly the "swap the surrogate on the Under-wheels zone"
move the recommendation calls for (§7).

---

## 5. Frame / axis convention (the Y-up / Z-up trap, spec §11, INTERFACE §3)

- **Chrono's up-axis is configurable, not fixed.** Gravity is set with
  `ChSystem::SetGravitationalAcceleration(ChVector3d)`; the SCM demo uses the convenience
  `sys.SetGravityY()` (gravity along −Y) — i.e. Chrono can run **Y-up**. There is also a Z-up
  convention (`SetGravityZ()` / explicit −Z vector) which is what ROS/REP-103 expects. **You choose.**
- **For foss_ipex, run Chrono Y-up** so it matches **Godot's Y-up** and the **field-space**
  convention frozen in `INTERFACE.md` §3:
  - Field space: `world x = col·cell_m`, `world z = row·cell_m`, `height = value` (up).
  - **Godot mapping (direct):** `godot.x = x`, `godot.y = height`, `godot.z = z`.
  - **Chrono Y-up:** set gravity `(0, −1.62, 0)`; Chrono `X≡field x`, `Chrono Y≡height (up)`,
    `Chrono Z≡field z`. Then Chrono body positions map **1:1** to Godot and to the raster grid — no
    rotation, only the row↔Z / col↔X index convention when writing rasters.
- **ROS bridge (deferred, INTERFACE §3):** REP-103 is **Z-up right-handed**. If/when the ROS bridge
  is built, convert at that seam (`ros.x = x`, `ros.z = height`, `ros.y = −z` or per chosen
  handedness). Running Chrono Y-up means the ROS conversion is a single documented swap at the bridge
  — the trap is *named and localized*, not spread through the producer. (Chrono::ROS, note, is
  source-only — not in conda.)
- **Practical caution:** much of Chrono::Vehicle's heritage is **Z-up**; some vehicle JSON models and
  demos assume Z-up. If a borrowed vehicle model misbehaves Y-up, the cheaper fix may be to run
  **Chrono Z-up internally** and transpose to field/Godot space at the export boundary (a fixed
  axis-swap in the writer) rather than fight a Z-up vehicle model. Decide once, document in
  `metadata.json` notes.

---

## 6. Effort estimate & top gotchas

### Effort

- **Path A — first running SCM-rover-on-slope demo (conda):** **~3-6 hours.** Make conda reachable +
  create py3.12 env + `conda install pychrono` (~15-30 min incl. solve/download), run a stock
  `demo_VEH_SCMTerrain_*` Python demo headless/xvfb (~1 hr), then adapt: lunar gravity, slope, our
  grid size (~2-4 hrs). A *contract-emitting* producer (writing valid `.rf32` from `GetModifiedNodes`)
  on top of that: **+1-2 days** of careful field-mapping and invariant-reconciliation work (§4).
- **Path B — from-source Chrono::Vehicle (+Irrlicht):** **~0.5-1 day** once `libeigen3-dev` is
  installed (configure + a full `-j12` build is tens of minutes to ~1 hr on this 12-thread box;
  the time sink is CMake module wrangling, not compile). **+ Chrono::GPU:** **+0.5-1 day** (CUDA
  compiles are slow; verify GPU module didn't silently auto-disable). Realistically **2-4 days** to a
  confident source build with Vehicle + SCM + GPU + Python bindings, including debugging.

### Top gotchas (ranked)

1. **SCM class rename across versions.** The current/main class is **`SCMTerrain`**; older code and
   much online material uses **`SCMDeformableTerrain`** (+ `SCMDeformableSoil`). On pychrono 10.0.0
   write to `SCMTerrain`; if you pin an older build, the name (and some method signatures) differ.
   **This will break copy-pasted examples.** Verify against your installed version, not memory.
2. **Conda Python pinning.** linux-64 pychrono 10.0.0 is **py312/py313 only — no py311**, so it
   **cannot** live in the project `.venv` (3.11.2). Producer either runs in the conda env, or you
   shuttle state across a subprocess/file boundary (which the INTERFACE seam already makes clean).
3. **No URDF/SDF import (spec §11).** You **cannot** load the IPEx URDF into Chrono — rebuild the
   kinematic tree as a Chrono::Vehicle JSON/C++ model or write a converter. Plan model-authoring time.
4. **Headless Irrlicht.** This is a server; an Irrlicht window needs a display → wrap in **xvfb**
   (already present, already used for Godot per spec §11) or run the producer windowless. VSG is not
   in conda and not packaged on bookworm — don't reach for it.
5. **Mass conservation is NOT SCM's.** SCM does not conserve areal mass or model bulking — the spec's
   load-bearing invariant (§5.3, §7, §10) stays surrogate-side. Don't expect to read `mass_areal` out
   of Chrono.
6. **Disk at 94%.** A source build tree + CUDA objects, or a conda env, is multi-GiB. Free space
   first or the build/install fails late and messily.

---

## 7. Recommendation for foss_ipex

**Take Path A (conda PyChrono 10.0.0) first**, and use it to swap the surrogate **only on the
"Under wheels (rolling)" zone** (spec §4): a Chrono::Vehicle wheeled rover on an `SCMTerrain` patch
at 1.62 m/s², Chrono run **Y-up** to match Godot/field-space (§5), exporting the deformed wheel-rut
geometry into the **frozen INTERFACE contract** via `GetModifiedNodes` / `NodeInfo`. This is the
fastest path to a *real physics-authority* wheel rut and slip-sinkage signature, with zero consumer
(Godot) changes — the whole point of the seam.

**Keep in the surrogate regardless of Chrono** (because SCM structurally lacks them — §4.3):
- The **conserved areal-mass invariant** and **bulking/swell** cut↔fill bookkeeping (spec §5.3, §7).
- **Loose-over-dense stratigraphy**, `z_cut` exposed-sublayer depth, and **multi-pass paving** density
  rise (spec §4, §6) — SCM is single-layer constant-density.
- **Excavation / drum material removal + berm deposition** and the **sandpile/repose relaxation**
  (spec §7) — not SCM's job; that's surrogate now, Chrono::GPU oracle for *calibration* later.
- **Dust, volatiles, ice, optics, state-label semantics** — all rendering/producer-side per spec §8.

**Path B (from source) is deferred** until you need (a) the **Chrono::GPU DEM Tier-3 calibration
oracle** (spec §10) to fit the surrogate's Bekker/repose/swell parameters — which on this box is a
supported CUDA-12.8 + RTX-4090 build — or (b) the **Chrono::ROS** bridge (not in conda). When that
day comes, the only new system prerequisite is `libeigen3-dev` (and `libirrlicht-dev` for viz);
everything else (gcc, cmake, CUDA, swig, OpenMP, xvfb) is already present and verified.

---

### Sources

- [Install PyChrono (conda)](https://api.projectchrono.org/pychrono_installation.html)
- [pychrono — Anaconda.org (projectchrono channel)](https://anaconda.org/projectchrono/pychrono) · [files](https://anaconda.org/projectchrono/pychrono/files)
- [SCMTerrain class reference](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_s_c_m_terrain.html) · [SCMTerrain::NodeInfo struct](https://api.projectchrono.org/structchrono_1_1vehicle_1_1_s_c_m_terrain_1_1_node_info.html)
- [Vehicle terrain models](https://api.projectchrono.org/vehicle_terrain.html)
- [demo_VEH_SCMTerrain_RigidTire.cpp (main)](https://github.com/projectchrono/chrono/blob/main/src/demos/vehicle/terrain/demo_VEH_SCMTerrain_RigidTire.cpp)
- [Install Chrono (from source)](https://api.projectchrono.org/tutorial_install_chrono.html) · [Installation guides index](https://api.projectchrono.org/install_guides.html)
- [Chrono::Vehicle module install](https://api.projectchrono.org/module_vehicle_installation.html)
- [Chrono::GPU module install](https://api.projectchrono.org/module_gpu_installation.html)
- [chrono root CMakeLists.txt](https://github.com/projectchrono/chrono/blob/main/CMakeLists.txt)

*Verified May 2026 against live Chrono docs. Class/flag names checked against the `main` branch and
pychrono 10.0.0; pin-specific signatures must be re-checked against the exact installed version.
Items not confirmable from docs are flagged inline ("verify on first run").*
