# Project Chrono (PyChrono / Path A) Bring-up Log — foss_ipex

Executes `docs/chrono_integration.md` §2 (Path A, conda PyChrono 10.0.0) on this box (midlife,
Debian 12). Goal: stand up a real Chrono physics authority that can eventually replace the NumPy
surrogate behind the frozen `INTERFACE.md` contract. **Nothing here is committed by the agent — the
human reviews/commits.** Producer/consumer share only the on-disk `.rf32`/`.r8` + `metadata.json` seam.

Times are wall-clock on a 12th Gen i5-12400 (12 threads), 125 GiB RAM, NVIDIA RTX 4090. Date: 2026-05-30.

---

## A. Disk + conda audit

### A.1 Disk guard (SAFETY constraint #1: stop if < ~10 GiB free)

```
$ df -h /home/john
Filesystem      Size  Used Avail Use% Mounted on
/dev/nvme0n1p2  456G  359G   75G  83% /
```

**75 GiB free, 83% used.** The integration doc (written 2026-05-30) recorded 94%/27 GiB; the box has
since freed up. **75 GiB >> 10 GiB guard → SAFE to install.** A conda pychrono env is ~3-5 GiB.

### A.2 conda location (doc §2.1: not on PATH; borrow text-gen's conda)

- `which conda mamba micromamba` → none on PATH (expected per doc §1).
- Usable conda: `/home/john/text-generation-webui/installer_files/conda/bin/conda`
- `~/.conda/environments.txt` references only that install + its `env`. No miniforge/miniconda/anaconda, no mamba/micromamba.

```
$ eval "$(/home/john/text-generation-webui/installer_files/conda/bin/conda shell.bash hook)"
$ conda --version
conda 23.3.1
$ conda env list
base   *  /home/john/text-generation-webui/installer_files/conda
          /home/john/text-generation-webui/installer_files/env
```

conda 23.3.1 (June 2024) — `conda create`/`install` work; classic solver may be slow (doc §2.1 note).

### A.3 Channel reachability + build verification

Network is up. Direct HTTPS probes (no conda solver):
- `conda.anaconda.org` → HTTP 302, `repo.anaconda.com` → HTTP 200.
- `projectchrono/linux-64/repodata.json` → HTTP 200, fetched in <1 s.

`conda search` timed out at 20 s — that is the **classic solver indexing conda-forge**, not a
network failure (confirmed by the raw curl above). Verified the exact build the doc names is present
by parsing repodata directly:

```
pychrono 10.0.0 py312h98ab86c_677   (latest 10.0.0 linux-64; also _446.._677 and py313 builds)
```

No py311 build of 10.0.0 (confirms gotcha #2 — cannot live in the project .venv).

---

## B. Create py3.12 env + install pychrono 10.0.0

### B.1 Create env (SAFETY #2: separate env, NOT the project .venv)

```
$ conda create -y -n chrono python=3.12
... Executing transaction: done    (RC=0)
```
Wall time ~9 s (01:33:28 → 01:33:37). Env path: `/home/john/text-generation-webui/installer_files/conda/envs/chrono`.

### B.2 Install pychrono (pinned to the verified build)

```
$ conda install -y -n chrono projectchrono::pychrono=10.0.0=py312h98ab86c_677 -c conda-forge
```
Started 01:33:46. conda 23.3.1's classic solver against full conda-forge is slow (doc §2.1 note);
the solve phase dominates. Disk during install held ~73-74 GiB free (well clear of the 10 GiB guard).

**RESULT: install succeeded, RC=0**, finished 01:50:35 → **~17 min total** (solve dominated; the
classic solver ran at ~96% CPU / ~5.5 GB RSS the whole time — active, not deadlocked). The trailing
UCX / Open-MPI "to enable CUDA support…" lines are **informational notices, not errors** (PyChrono
pulls an MPI stack as a transitive dep). Disk after install: **66 GiB free** (consumed ~9 GiB; still
far above the 10 GiB guard).

```
$ conda list -n chrono | grep -i pychrono
pychrono   10.0.0   py312h98ab86c_677   projectchrono
```

### B.3 BLOCKER then FIX — `pychrono.vehicle` libgdal soname skew

First import probe:
- `import pychrono` → **OK** (core multibody loads).
- `import pychrono.vehicle` → **ImportError: libgdal.so.37: cannot open shared object file**.

`ldd .../pychrono/_vehicle*.so` → only one unsatisfied dep: **`libgdal.so.37 => not found`**.
The dependency solve installed **`libgdal-core 3.12.0`** (provides `libgdal.so.38`), but the pinned
pychrono build `_677` was compiled against the **GDAL 3.9 ABI (soname `.so.37`)**. Pure soname skew —
Vehicle is the module we need for SCM, so this is a hard blocker until resolved.

**Fix:** install the matching GDAL into the env to provide `libgdal.so.37`:
```
$ conda install -y -n chrono -c conda-forge "libgdal=3.9.3"
```
**First fix attempt WRONG (recorded for honesty):** assumed `.so.37` was an *older* GDAL and
installed `libgdal=3.9.3`. That install succeeded (RC=0, ~19 min) but 3.9.3 ships **`libgdal.so.35`**,
not `.so.37` — so `import pychrono.vehicle` still failed. The soname↔version map is the other
direction: **3.9→.so.35, 3.10→.so.36, 3.11→.so.37, 3.12→.so.38.** The pinned pychrono `_677` was
built against **GDAL 3.11** (soname 37); the original solve had pulled 3.12 (soname 38), i.e. one
*too new*, and 3.9.3 was two *too old*.

**Correct fix:**
```
$ conda install -y -n chrono -c conda-forge "libgdal=3.11"
```
Result recorded in B.4.

### B.4 Solver pathology + the route that worked

The `libgdal=3.11` install under conda 23.3.1's **classic solver** ran 30+ min stuck in the SOLVE
phase (100% CPU, ~5.7 GB RSS, never reaching the transaction). This is the slow-solver risk the doc
flagged (§2.1). Attempts and outcomes, in order:

1. `libgdal=3.9.3` — solved in ~19 min, applied, but shipped `.so.35` (wrong; see B.3).
2. `libgdal=3.11` (classic solver) — **abandoned after 30+ min** still solving. Killed mid-solve
   (verified env left at libgdal-core 3.9.3 — solve never applied, nothing partial).
3. Surgical extraction of `libgdal.so.37` from the `libgdal-core-3.11.5` .conda — extracted cleanly,
   but with the env's libs on `LD_LIBRARY_PATH` the 3.11 core still had **2 unmet deps**
   (`libxml2.so.16`, `libxerces-c-3.3.so`) — the env's 3.9.3 stack carries older sonames. Confirms a
   manual drop-in needs the full 3.11 dependency closure, i.e. exactly what a real solve computes.
   **Abandoned** to avoid a half-mixed GDAL stack.
4. `conda install -n base conda-libmamba-solver` (doc §2.1's endorsed speed fix) — itself runs under
   the classic solver the first time; also stalled ~18 min, killed.
5. **Route taken:** `conda install -y -n chrono -c conda-forge "pychrono=10.0.0=py312h98ab86c_677"
   "libgdal=3.11"` — pin pychrono so the solver's search space is small, let it compute the full
   consistent GDAL-3.11 closure. (Result in B.5.)

**Core engine VERIFIED working regardless of the Vehicle blocker** (pure `pychrono.core`):
```
core step OK  t=0.100  y=0.99182   # 1 kg body dropped under (0,-1.62,0); ~0.008 m fall in 0.1 s, correct
irrlicht import OK                  # conda ships Irrlicht prebuilt (doc §2.2) — imports headless
```
So the multibody solver + lunar gravity + Irrlicht module are all good; only SCM (inside Vehicle)
waits on the GDAL soname.

### B.5 Vehicle import — VERIFIED

The pinned solve worked:
```
$ conda install -y -n chrono -c conda-forge "pychrono=10.0.0=py312h98ab86c_677" "libgdal=3.11"
... Executing transaction: done   RC=0   (~21 min; classic solver)
$ ls $CONDA_PREFIX/lib/libgdal.so.37*
libgdal.so.37 -> libgdal.so.37.3.11.0   (soname now present)
```
Import probe (`/tmp/chrono_probe.py`):
```
import pychrono            -> OK   (NOTE: 10.0.0 has NO chrono.__version__ attribute; use module file / conda list)
import pychrono.vehicle    -> OK   (libgdal.so.37 resolved)
resolved SCM class: <class 'pychrono.vehicle.SCMTerrain'>   (gotcha #1 confirmed: SCMTerrain, not SCMDeformableTerrain)
SCM methods present: GetHeight, GetInitHeight, GetNodeInfo, GetModifiedNodes, GetNormal/GetInitNormal,
  Initialize, SetSoilParameters, EnableBulldozing, Synchronize, Advance, GetSCMLoader,
  GetNumRayCasts/RayHits/ContactPatches/ErosionNodes, SetReferenceFrame/GetReferenceFrame (NO SetPlane).
ChBody.EnableCollision = True (use this; SetCollide absent).  QuatFromAngleX, ChCollisionShapeCylinder,
  ChContactMaterialSMC, ChSystemSMC, ChCoordsysd all present.
```

**Final env state:** `python 3.12.12`, `pychrono 10.0.0 py312h98ab86c_677` (projectchrono),
`libgdal-core 3.11.0`, `numpy 2.4.6`, `matplotlib` present. Env size **5.6 GiB**. Disk after all
installs: **132 GiB free**. Env path:
`/home/john/text-generation-webui/installer_files/conda/envs/chrono`.

**Net B summary:** env created + pychrono 10.0.0 installed and importable INCLUDING the Vehicle/SCM
module. The only non-trivial fix was the GDAL soname skew (B.3/B.4): the solver's default GDAL (3.12,
`.so.38`) didn't match the build's link target (3.11, `.so.37`); pinning `libgdal=3.11` alongside the
pinned pychrono produced a consistent, working env. **This is a reproducible gotcha worth adding to
chrono_integration.md** (the conda `_677` build needs GDAL 3.11 / `libgdal.so.37`).

---

## C. Stock SCM demo, headless — VERIFIED

Found two SCM demos shipped in the env:
`.../envs/chrono/lib/python3.12/site-packages/pychrono/demos/vehicle/demo_VEH_DeformableSoil.py`
and `demo_VEH_HMMWV_DefSoil.py`. Used the former (single rigid tractor-wheel mesh on SCM, driven by
a rotation motor). Made a **headless copy** (`/tmp/demo_VEH_SCM_headless.py`, scratch — not committed):
stripped Irrlicht, set `SetGravitationalAcceleration(0,-1.62,0)`, capped at 300 steps, added read-back
prints. Chrono data path set to the env's `share/chrono/data/`.

Key facts this stock demo VERIFIED for 10.0.0 (and which corrected the integration-doc §2.3 sketch):
- Terrain class is `veh.SCMTerrain`. Patch set up with **`SetReferenceFrame(ChCoordsysd(...))`**, NOT
  `SetPlane`. The demo's own comment: *"SCMTerrain uses a default ISO reference frame (Z up). Since the
  mechanism is modeled in a Y-up global frame, we rotate the terrain frame by -90 deg about X."* → run
  Y-up by `QuatFromAngleX(-pi/2)` on the terrain frame (matches integration-doc §5).
- A bare `SCMTerrain` + body advances **inside `sys.DoStepDynamics(dt)`** — the demo does NOT call
  `terrain.Synchronize/Advance` separately (those belong to the Chrono::Vehicle driver loop). My
  rover script was corrected to this single-step pattern.
- Uses `ChSystemSMC` + `SetCollisionSystemType(Type_BULLET)` + `body.EnableCollision(True)`.

Run (windowless, no xvfb needed — gotcha #4):
```
$ python /tmp/demo_VEH_SCM_headless.py
gravity (0,-1.62,0) Y-up lunar
stepping 300 steps @ dt=0.002s headless (no Irrlicht)...
  step    0 ... body=(+0.000,+1.120,-1.500)
  ...
  step  299 ... body=(-0.000,+0.850,-1.492)        # wheel descends under 1/6 g, SCM resists
GetModifiedNodes(True) -> 261 nodes
  GetNumRayCasts() = 884   GetNumRayHits() = 258   GetNumContactPatches() = 1
STOCK_DEMO_OK: SCM stepped headless without error.
```
**stock_demo_ran = TRUE.** SCM is functional end-to-end: ray-casting, contact, deformed-node read-back.

---

## D. Custom rover script — VERIFIED

`scripts/chrono_scm_rover.py`: a single rigid cylinder "wheel" (r=0.15 m, 25 kg) given a forward +X
velocity and a spin, on an `SCMTerrain` patch (**1.28 m, 2 cm cells → 64×64 nodes**, same cell size as
the 256² INTERFACE sample), `ChSystemSMC` + Bullet, Y-up via the -90°-X terrain reference frame, lunar
g, bulldozing on, 400 steps @ 1 ms, windowless.

```
$ python scripts/chrono_scm_rover.py
using terrain class: SCMTerrain
SCM patch 1.28 x 1.28 m, cell 0.02 m -> ~64x64 nodes
stepping 400 steps @ dt=0.001s ...
  step   0  wheel=(+0.0003,+0.1550,+0.0000)
  step 399  wheel=(+0.0974,+0.0547,+0.0001)         # rolled +X ~9.7 cm, sank ~10 cm under 1/6 g
  GetNumRayHits()=194  GetNumRayCasts()=650  GetNumContactPatches()=1  GetNumErosionNodes()=758
GetModifiedNodes(all=True) returned 918 nodes
  deepest modified node (i=9, j=-5) height=-0.01336 m   # a real, readable wheel rut
OK: SCM rover spike stepped without fatal error.
```
**custom_script_ran = TRUE, 400 steps.** Erosion/bulldozing active (758 erosion nodes); 918 deformed
nodes read back; deepest rut -13.4 mm.

---

## E. SCM -> INTERFACE exporter — STUB / PARTIAL

`scripts/chrono_scm_export.py` (library) + `scripts/chrono_scm_export_demo.py` (end-to-end driver).
The exporter reads SCM and writes a **contract-valid** scene dir through the project's frozen
`terrain_authority/io_fields.save_scene` (INTERFACE §7 — the only Python raster writer).

Field mapping realized (per integration-doc §4):
| INTERFACE field | Source in stub | Provenance |
|---|---|---|
| `heightmap.rf32` | SCM deformed node heights (`GetModifiedNodes(True)`, fallback `GetHeight`) | **Chrono-sourced**, re-derived to satisfy §4 |
| `disturbance.rf32` | plastic-sinkage proxy (`GetInitHeight - GetHeight`), normalized to [0,1] | **Chrono-sourced** |
| `state_label.r8` | sinkage>thr → TREAD(1) else VIRGIN(0) | **producer-derived** (SCM has no enum, §4.2) |
| `mass_areal.rf32` | constant 1500 kg/m³ × column thickness | **SURROGATE placeholder** (SCM doesn't conserve mass, §4.3) |
| `density.rf32` | constant 1500 kg/m³ | **SURROGATE placeholder** (single-layer Bekker, §4.3) |
| `ice.rf32` | not written | producer-owned, not a Chrono concept |
| rover pose | `chassis_body.GetPos()` → `metadata.clasts[]` | Chrono-sourced |

```
$ python scripts/chrono_scm_export_demo.py
EXPORT WROTE: /tmp/chrono_scm_scene
files: [density.rf32, disturbance.rf32, heightmap.rf32, mass_areal.rf32, metadata.json, state_label.r8]
NodeInfo @ deepest cell (27,41): sinkage=0.00866 sinkage_plastic=0.00865 sigma_yield=1732.5 kshear=0.00684 tau=429.4
LOAD_SCENE round-trip: heightmap (64,64) f32 [-0.0134,+0.0189]; mass_areal [1480,1528]; density 1500; state_label {0,1}
INTERFACE invariant |height - (mass/density - 1.0)| max err = 3.98e-08   # holds to float32 eps
disturbance in [0,1]: True;  state_label in {0,1}: True
EXPORT_STUB_OK
```
**exporter_status = STUB→PARTIAL.** It DOES: read the deformed-node field, read the per-node
`NodeInfo` Bekker/Janosi state vector (sinkage / plastic / sigma_yield / kshear / tau — exactly §4.1),
rasterize to our grid, write all REQUIRED contract files, round-trip via `load_scene`, and satisfy the
`height = mass_areal/density - datum` invariant (§4) to float32 precision. It does NOT: close mass
conservation (mass_areal/density are honest, clearly-labeled placeholders — that work is permanently
surrogate-side per §4.3/§7); the §4.4 *hybrid* (fold SCM rut into the surrogate's mass grid) is the
next real step, not done here.

**Notes / honest caveats:**
- `GetNodeInfo(loc)` is a point query — sampling at a cell that falls between active ray-cast nodes
  returns zeros; query a known-modified cell (as the stub now does) to get live values. The reliable
  per-node *height* field is `GetModifiedNodes`.
- The stub's SCM-grid (i,j)→raster (row,col) mapping uses a center-shift; for a production exporter the
  SCM reference-frame ↔ field-space index convention (INTERFACE §3, integration-doc §5) must be pinned
  and tested against a known feature, not assumed.
- `pychrono` 10.0.0 exposes no `__version__`; identify the build via `conda list` / module path.

---

## Files added (NOT committed — for human review)
- `docs/chrono_bringup_log.md` (this file)
- `scripts/chrono_scm_rover.py` (goal D — runs)
- `scripts/chrono_scm_export.py` (goal E — exporter stub library)
- `scripts/chrono_scm_export_demo.py` (goal E — end-to-end driver, runs)

Scratch (in /tmp, not in repo): `/tmp/demo_VEH_SCM_headless.py`, `/tmp/chrono_probe.py`,
`/tmp/run_export.py`, scene output `/tmp/chrono_scm_scene/`.

## Recommended next steps (for a human)
1. Add the **GDAL gotcha** to `docs/chrono_integration.md` §6: conda pychrono `10.0.0
   py312h98ab86c_677` links `libgdal.so.37`; the default solve pulls GDAL 3.12 (`.so.38`) and breaks
   `import pychrono.vehicle` — fix is `conda install -n chrono -c conda-forge "libgdal=3.11"` pinned
   alongside pychrono. Also note `SetReferenceFrame` (not `SetPlane`) and the no-`__version__` fact.
2. Speed up future solves: finish installing `conda-libmamba-solver` into base (it stalled under the
   classic solver here; let it run to completion once, or install Miniforge for `mamba`). Each classic
   solve cost ~20-30 min.
3. Build a real **Chrono::Vehicle** wheeled model (authored JSON / kinematic tree — no URDF import,
   gotcha #3) instead of the bare rigid cylinder, so tires register with SCM properly.
4. Implement the §4.4 **hybrid**: fold SCM's deformed rut + `NodeInfo.sinkage*` into the surrogate's
   mass/density grid, re-derive height to keep the invariant green, and emit through `io_fields` —
   replacing the placeholder mass_areal/density with the surrogate's conserved field.
5. Pin and test the SCM-grid ↔ field-space index/axis convention against a known asymmetric feature.


---

## C/D. Scripts drafted (run after install confirms the API)

- `scripts/chrono_scm_rover.py` — single rigid cylinder "wheel" on an `SCMTerrain` patch, Y-up,
  lunar g (-1.62), 1.28 m patch @ 2 cm cells (64x64, same cell size as the 256² INTERFACE sample),
  400 steps @ 1 ms, windowless (no Irrlicht → no xvfb needed, gotcha #4). Resolves
  `SCMTerrain` vs legacy `SCMDeformableTerrain` at runtime (gotcha #1). Reads back
  `GetModifiedNodes(True)` + diagnostics counters.
- `scripts/chrono_scm_export.py` — SCM→INTERFACE mapping STUB. Imports the frozen
  `terrain_authority/io_fields.save_scene` (INTERFACE §7). Sources `heightmap` + `disturbance`
  from SCM; `state_label` is producer-derived; `mass_areal` + `density` are clearly-labeled
  SURROGATE-OWNED placeholders (SCM does not conserve mass — §4.3/§7). Demonstrates `GetNodeInfo`
  read-back of the per-node terramechanics state vector.

---
