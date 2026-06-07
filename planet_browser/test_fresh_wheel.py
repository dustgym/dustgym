"""WP0.6 (RB-06) — the GOLD acceptance: a fresh wheel installs and the server product runs.

Builds the wheel, installs `dustgym[server]` into a CLEAN venv (proving the server extra carries every
import-time dependency), and in that venv imports the server + runs a real Mars plan (no DEM asset
needed) writing its report to a configurable $DUSTGYM_DATA_DIR. This is slow (a wheel build + a clean
network install ~2-3 min), so it is OPT-IN: set DUSTGYM_WHEEL_SMOKE=1 to run it (release/scheduled gate).
The fast import-graph proxy in test_server_install.py runs in standard CI every time.
"""
from __future__ import annotations

import glob
import os
import subprocess
import sys
import venv

import pytest

if os.environ.get("DUSTGYM_WHEEL_SMOKE") != "1":
    pytest.skip("fresh-wheel smoke is opt-in (slow + network); set DUSTGYM_WHEEL_SMOKE=1",
                allow_module_level=True)

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Run inside the clean venv: server imports (fastapi), planner runs a Mars plan (matplotlib report),
# the report lands in the configured data dir, and the dustgym-serve entrypoint resolves. No DEM, no httpx.
_SMOKE = r"""
import os
os.environ["DUSTGYM_DATA_DIR"] = os.environ["SMOKE_DATA"]
import planet_browser.server                       # FastAPI app builds -> server extra deps present
from planet_browser.server import main             # console_scripts: dustgym-serve = server:main
assert callable(main)
import dustgym                                      # registers the gym envs
from planet_browser import mission_planner as MP
m = MP.mission_from_dict({"name": "wheel", "body": "mars", "charger": [0, 0],
                          "orders": [{"action": "pad", "kind": "cut", "x": 10, "y": 10,
                                      "footprint_m2": 9, "depth_m": 0.05}]})
pdf, md, totals = MP.run(m, stem="wheel_smoke")
assert os.path.exists(pdf) and pdf.startswith(os.environ["SMOKE_DATA"]), pdf
print("FRESH WHEEL OK")
"""


def test_fresh_wheel_server_runs(tmp_path):
    dist = tmp_path / "dist"
    # build just the dustgym wheel via pip's own backend (no `build`/pyproject_hooks toolchain dep).
    subprocess.run([sys.executable, "-m", "pip", "wheel", "--no-deps", "-w", str(dist), _REPO],
                   check=True, capture_output=True, text=True)
    wheels = glob.glob(str(dist / "dustgym-*.whl"))
    assert wheels, "wheel build produced no .whl"

    venv_dir = tmp_path / "venv"
    venv.create(str(venv_dir), with_pip=True)
    py = str(venv_dir / ("Scripts" if os.name == "nt" else "bin") / "python")
    # CLEAN env for the clean venv: drop the parent's PYTHONPATH (points at the repo source),
    # PYTHONHOME/VIRTUAL_ENV, and PYTHONNOUSERSITE so the venv python resolves ITS OWN site-packages.
    clean = {k: v for k, v in os.environ.items()
             if k not in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "PYTHONNOUSERSITE")}
    subprocess.run([py, "-m", "pip", "install", "--quiet", f"{wheels[0]}[server]"],
                   check=True, capture_output=True, text=True, env=clean)

    env = {**clean, "SMOKE_DATA": str(tmp_path / "appdata")}
    r = subprocess.run([py, "-c", _SMOKE], env=env, capture_output=True, text=True, cwd=str(tmp_path))
    assert r.returncode == 0, f"fresh-wheel smoke failed:\nSTDOUT:\n{r.stdout}\nSTDERR:\n{r.stderr}"
    assert "FRESH WHEEL OK" in r.stdout
    # the report was written under the configured data dir, in the clean venv (not the source tree)
    assert glob.glob(str(tmp_path / "appdata" / "reports" / "wheel_smoke.*"))


if __name__ == "__main__":
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        os.environ["DUSTGYM_WHEEL_SMOKE"] = "1"
        test_fresh_wheel_server_runs(__import__("pathlib").Path(d))
        print("ok")
