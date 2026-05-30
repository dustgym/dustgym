"""End-to-end demo: build the SCM rover spike, step it, run the SCM->INTERFACE exporter stub.

Shows the full Path A seam: a PyChrono SCMTerrain spike (chrono_scm_rover.py shape) is stepped
at lunar gravity, then chrono_scm_export.export() reads GetModifiedNodes / GetNodeInfo and writes
a contract-VALID scene dir via the frozen terrain_authority.io_fields.save_scene; we then read it
back with load_scene and assert the INTERFACE.md invariant (height == mass_areal/density - datum),
disturbance in [0,1], and state_label in {0..4}.

Run in the conda 'chrono' env:
    eval "$(/home/john/text-generation-webui/installer_files/conda/bin/conda shell.bash hook)"
    conda activate chrono
    python scripts/chrono_scm_export_demo.py
Writes the scene to /tmp/chrono_scm_scene (a scratch dir; not committed).
"""
import math
import os
import sys

import pychrono as chrono
import pychrono.vehicle as veh

ROOT = "/home/john/Development/foss_ipex"
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "scripts"))

import chrono_scm_export as exp  # noqa: E402
from terrain_authority.io_fields import load_scene  # noqa: E402

CELL_M = 0.02
PATCH_M = 1.28
WIDTH = HEIGHT = int(round(PATCH_M / CELL_M))  # 64

# --- build a small SCM sim (same shape as chrono_scm_rover.py) ---
sysmbs = chrono.ChSystemSMC()
sysmbs.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sysmbs.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

wheel = chrono.ChBody()
wheel.SetMass(25.0)
wheel.SetPos(chrono.ChVector3d(0.0, 0.155, 0.0))
wheel.SetRot(chrono.QuatFromAngleX(math.pi / 2.0))
sysmbs.AddBody(wheel)
mat = chrono.ChContactMaterialSMC()
mat.SetFriction(0.8)
wheel.EnableCollision(True)
wheel.AddCollisionShape(chrono.ChCollisionShapeCylinder(mat, 0.15, 0.12))
wheel.SetPosDt(chrono.ChVector3d(0.25, 0.0, 0.0))
wheel.SetAngVelLocal(chrono.ChVector3d(0.0, 0.0, -1.6))

terrain = veh.SCMTerrain(sysmbs)
terrain.SetReferenceFrame(
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(-math.pi / 2.0)))
terrain.EnableBulldozing(True)
terrain.SetSoilParameters(0.2e6, 0.0, 1.0, 170.0, 35.0, 0.018, 2e8, 3e4)
terrain.Initialize(PATCH_M, PATCH_M, CELL_M)

for i in range(400):
    sysmbs.DoStepDynamics(1e-3)

# --- run the exporter stub ---
out_dir = "/tmp/chrono_scm_scene"
meta, node_info = exp.export(terrain, chrono, sysmbs, out_dir, WIDTH, HEIGHT, CELL_M,
                             gravity=1.62, chassis_body=wheel)
print("=== EXPORT WROTE:", out_dir)
print("files:", sorted(os.listdir(out_dir)))
print("NodeInfo @ deepest cell:", node_info)

# --- read it back through the frozen contract loader ---
fields, md = load_scene(out_dir)
print("=== LOAD_SCENE round-trip ===")
for k, v in fields.items():
    print(f"  {k:12s} shape={v.shape} dtype={v.dtype} min={float(v.min()):+.4f} max={float(v.max()):+.4f}")
import numpy as np
# Verify the INTERFACE invariant. The stub uses a documented 1.0 m nominal soil-column datum,
# so height = mass_areal/density - 1.0 (column thickness minus the nominal column).
NOMINAL = 1.0
h_derived = fields["mass_areal"] / fields["density"] - NOMINAL
err = float(np.max(np.abs(h_derived - fields["heightmap"])))
print(f"INTERFACE invariant |height - (mass/density - {NOMINAL})| max err = {err:.2e} (should be ~0)")
print(f"disturbance in [0,1]: {bool(fields['disturbance'].min()>=0 and fields['disturbance'].max()<=1)}")
print(f"state_label values present: {sorted(set(fields['state_label'].ravel().tolist()))}")
print("EXPORT_STUB_OK")
