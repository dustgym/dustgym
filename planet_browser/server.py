#!/usr/bin/env python3
"""server.py -- local server for the planet browser (P1: place -> queue -> optimize -> report).

Serves the static front-end (index.html, bodies.json) + the generated reports, and exposes
POST /plan : a build-order queue (JSON) -> mission_planner -> a mission-control PDF, returned as a
URL the browser opens. No web framework -- stdlib http.server only, so planet_browser/ stays
dependency-light (matplotlib, already used by the report, is the only heavy dep).

    python3 server.py [--port 8770]      # then open the printed URL in a browser
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import autonomy as AUT
import mission_planner as MP
import structures as ST

# plan_render_pipeline lives in roversim/scripts (it drives the Godot sidecar there); MP already put
# roversim/ on the path. Importing it is optional -- /render degrades to a 503 if the binary is absent.
sys.path.insert(0, os.path.join(MP._ROVERSIM, "scripts"))
try:
    import plan_render_pipeline as PRP
except Exception:   # noqa: BLE001 -- /render just becomes unavailable
    PRP = None
_HAWORTH = os.path.join(MP._ROVERSIM, "samples", "lunar_dem", "haworth_10km_5m")

HERE = os.path.dirname(os.path.abspath(__file__))
REPORTS = os.path.join(HERE, "reports")

def _totals_json(totals):
    """JSON-safe totals: numbers -> float, but pass bools and strings (e.g. algorithm/objective) through."""
    out = {}
    for k, v in totals.items():
        out[k] = v if isinstance(v, (bool, str)) else float(v)
    return out


_MOON_DEM = None   # (dem, flattest-anchor); loaded once, reused across /plan requests


def _moon_dem():
    """Load the real Haworth DEM + its auto-selected flattest buildable anchor once and cache it, so
    Moon plans get live I6/M11 slope-gating. Degrades to (None, (0,0)) -> flat check if the bundle is
    absent, rather than failing the request."""
    global _MOON_DEM
    if _MOON_DEM is None:
        try:
            dem = MP.load_haworth_dem()
            _MOON_DEM = (dem, MP.flattest_anchor(dem))
        except Exception:
            _MOON_DEM = (None, (0.0, 0.0))
    return _MOON_DEM


def _autonomy_perception(mission, dem, origin, algorithm, objective):
    """Fold the closed-loop autonomy + the AutoNav estimation (perception) uncertainty into /plan.

    Runs the conserved-model closed loop (plan -> execute -> estimate -> replan) once. The `autonomy`
    block summarizes the controller (recharges/replans/completion + the true-vs-budgeted energy the slip
    truth forces); the `perception` block is the rover's onboard ESTIMATE confidence (pose sigma grows by
    dead-reckoning, drum-fill sigma from the FDC mass-inference model, energy sigma from model error).
    Additive: any failure returns (None, None) so the report still goes out."""
    try:                                               # perception-in-the-loop ON: a SLAM/map pose fix per leg
        cl = AUT.run_closed_loop(mission, dem=dem, dem_origin=origin, algorithm=algorithm,
                                 objective=objective, perception_sigma_m=0.10)
    except Exception:                                  # noqa: BLE001 -- autonomy is additive, never break /plan
        return None, None
    b, legs = cl["belief"], cl["legs"]
    nominal = sum(leg["nominal_J"] for leg in legs)
    true = sum(leg["true_J"] for leg in legs)
    autonomy = {
        "completed": cl["completed"], "n_trips": cl["n_trips"], "n_legs": len(legs),
        "recharges": cl["recharges"], "replans": cl["replans"],
        "perception_fixes": cl["perception_fixes"], "observe_more": cl["observe_more"],
        "final_soc": round(b.soc_frac(), 3),
        "max_slip": round(max((leg["slip"] for leg in legs), default=0.0), 3),
        "true_vs_nominal_energy": round(true / nominal, 3) if nominal else None,
    }
    leg_e_sig = max((leg["energy_sigma_J"] for leg in legs), default=0.0)
    perception = {
        "pose_sigma_m": round(b.pos_sigma_m, 2),               # BOUNDED by the per-leg map/landmark fixes
        "map_fixes": cl["perception_fixes"],                   # pose corrections fused into the belief
        "observe_more_before_dig": cl["observe_more"],         # Uncertainty-layer dig-ready gate firings
        "fix_sigma_m": 0.10,                                   # SLAM/map-match fix precision (AprilTag 12.7 mm best-case)
        "energy_model_sigma_J": round(leg_e_sig, 1),           # slip model-error 1-sigma carried per leg
        "drum_fill_uncertainty_pct": 7.4,                      # FDC mass-inference MPE (2.56% >half full, 7.40% over range)
        "note": ("perception-in-the-loop: a map/landmark pose fix per leg bounds the dead-reckoning drift, "
                 "and the dig-ready gate observes more before digging when the pose estimate is uncertain. "
                 "The dense observed-map RMSE still needs a render (see /render)."),
    }
    return autonomy, perception


_CTYPE = {".html": "text/html; charset=utf-8", ".json": "application/json",
          ".pdf": "application/pdf", ".md": "text/markdown; charset=utf-8",
          ".js": "text/javascript", ".css": "text/css", ".png": "image/png"}


def _plan_stem(payload):
    """Stable, collision-free report stem from the mission (name slug + content hash) -- repeatable,
    no wall-clock, so the same queue regenerates the same file instead of piling up duplicates."""
    name = re.sub(r"[^a-z0-9]+", "-", str(payload.get("name", "mission")).lower()).strip("-") or "mission"
    digest = hashlib.sha1(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:8]
    return f"{name}-{digest}"


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code, obj):
        self._send(code, json.dumps(obj), "application/json")

    def _serve_file(self, path, ctype):
        if not os.path.isfile(path):
            return self._send_json(404, {"ok": False, "error": f"not found: {os.path.basename(path)}"})
        with open(path, "rb") as f:
            self._send(200, f.read(), ctype)

    def do_GET(self):
        route = self.path.split("?", 1)[0]
        if route in ("/", "/index.html"):
            return self._serve_file(os.path.join(HERE, "index.html"), _CTYPE[".html"])
        if route == "/bodies.json":
            return self._serve_file(os.path.join(HERE, "bodies.json"), _CTYPE[".json"])
        if route.startswith("/reports/"):
            name = os.path.basename(route)                 # basename only -> no path traversal
            ext = os.path.splitext(name)[1]
            return self._serve_file(os.path.join(REPORTS, name), _CTYPE.get(ext, "application/octet-stream"))
        if route.startswith("/dem/"):                      # the real LOLA work-area DEM previews (Haworth)
            bundle = os.path.join(HERE, "..", "roversim", "samples", "lunar_dem", "haworth_10km_5m")
            f = {"hillshade.png": "preview_hillshade.png", "height.png": "preview_height.png"}.get(os.path.basename(route))
            if f:
                return self._serve_file(os.path.join(bundle, f), "image/png")
            return self._send_json(404, {"ok": False, "error": f"no dem {os.path.basename(route)}"})
        return self._send_json(404, {"ok": False, "error": f"no route {route}"})

    def do_POST(self):
        route = self.path.split("?", 1)[0]
        if route not in ("/plan", "/sense", "/structure", "/compare", "/render"):
            return self._send_json(404, {"ok": False, "error": f"no route {route}"})
        try:
            n = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError) as e:
            return self._send_json(400, {"ok": False, "error": f"bad JSON: {e}"})
        if route == "/sense":
            return self._sense(payload)
        if route == "/structure":
            return self._structure(payload)
        if route == "/compare":
            return self._compare(payload)
        if route == "/render":
            return self._render(payload)
        algorithm = payload.get("algorithm", "nearest")   # pluggable sequencer + objective (user-selected)
        objective = payload.get("objective", "time")
        try:
            mission = MP.mission_from_dict(payload)
            dem, origin = _moon_dem() if mission.body == "moon" else (None, (0.0, 0.0))
            # I10: hauls routed around hazards on the real DEM; I8 (physical realizability) + I6/M11
            # (slope-feasible siting), local order frame anchored to the auto-selected flattest region.
            pdf, md, totals = MP.run(mission, stem=_plan_stem(payload), dem=dem, dem_origin=origin,
                                     algorithm=algorithm, objective=objective)
            validation = MP.validate_plan(mission, dem=dem, dem_origin=origin)
            timeline = MP.build_timeline(mission, dem=dem, dem_origin=origin,   # P5: execute + watch
                                         algorithm=algorithm, objective=objective)
            endurance = MP.endurance(mission, dem=dem, dem_origin=origin)       # single-charge range
            autonomy, perception = _autonomy_perception(mission, dem, origin, algorithm, objective)
        except (ValueError, RuntimeError) as e:            # bad input / sinter-gated -> honest 400
            return self._send_json(400, {"ok": False, "error": str(e)})
        return self._send_json(200, {
            "ok": True,
            "pdf": "/reports/" + os.path.basename(pdf),
            "md": "/reports/" + os.path.basename(md),
            "totals": _totals_json(totals),
            "validation": validation,
            "timeline": timeline,
            "endurance": endurance,
            "autonomy": autonomy,           # closed-loop controller summary (recharges / replans / completion)
            "perception": perception,       # AutoNav onboard estimate uncertainty (pose / drum-fill / energy sigma)
        })

    def _render(self, payload):
        """Crop a Haworth window at the picked (u,v), plan a flatten, render BEFORE/AFTER in Godot, and
        return the figure URL + earthwork volumes. Slow (two Godot renders); 503 if the binary is absent."""
        if PRP is None:
            return self._send_json(503, {"ok": False, "error": "render pipeline unavailable (Godot binary absent)"})
        try:
            u = float(payload.get("u", 0.5))
            v = float(payload.get("v", 0.5))
            pad_frac = float(payload.get("pad_frac", 0.5))
        except (TypeError, ValueError) as e:
            return self._send_json(400, {"ok": False, "error": f"bad params: {e}"})
        stem = "render_" + hashlib.sha1(f"{u:.4f}_{v:.4f}_{pad_frac:.2f}".encode()).hexdigest()[:10]
        try:
            r = PRP.render_map_area(_HAWORTH, u, v, os.path.join(REPORTS, stem), pad_frac=pad_frac)
        except Exception as e:                             # noqa: BLE001 -- render failure -> honest 500
            return self._send_json(500, {"ok": False, "error": f"render failed: {e}"})
        fig_name = stem + ".png"
        shutil.copyfile(r["figure"], os.path.join(REPORTS, fig_name))
        return self._send_json(200, {
            "ok": True, "figure": "/reports/" + fig_name,
            "cut_vol_m3": round(r["cut_vol_m3"], 2), "fill_vol_m3": round(r["fill_vol_m3"], 2),
            "cut_kg": round(r["cut_kg"]), "extent_m": round(r["extent_m"], 1), "cell_m": round(r["cell_m"], 2),
        })

    def _compare(self, payload):
        """Run every sequencer and return their metrics sorted by the chosen objective (the multi-algorithm
        comparison the UI sorts by)."""
        objective = payload.get("objective", "time")
        try:
            mission = MP.mission_from_dict(payload)
            dem, origin = _moon_dem() if mission.body == "moon" else (None, (0.0, 0.0))
            result = MP.compare_algorithms(mission, objective=objective, dem=dem, dem_origin=origin)
        except (ValueError, RuntimeError) as e:
            return self._send_json(400, {"ok": False, "error": str(e)})
        return self._send_json(200, {"ok": True, **result})

    def _structure(self, payload):
        """Decompose a named structure (Landing Pad / Haul Road / Berm / ...) placed at (x,y) into
        mass-balanced cut/fill orders (structures.decompose). Returns orders the build queue can adopt."""
        name = payload.get("name")
        try:
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
        except (TypeError, ValueError):
            return self._send_json(400, {"ok": False, "error": "x and y must be numeric"})
        params = payload.get("params") or {}
        try:
            orders = ST.decompose(name, x, y, **params)
        except (ValueError, TypeError) as e:
            return self._send_json(400, {"ok": False, "error": str(e)})
        return self._send_json(200, {"ok": True, "name": name, "orders": orders})

    def _sense(self, payload):
        """Drum-fill sensing (ICE-RASSOR): true drum mass -> motor-current observable -> inferred mass +
        offload decision. `noise_frac` toggles seeded sensor noise (0 = OFF). Calibrates a DrumSensor over
        a mass grid up to `capacity_kg`, so the inference is fit-from-data, not hard-coded."""
        try:
            true_mass = float(payload["true_mass_kg"])
        except (KeyError, TypeError, ValueError):
            return self._send_json(400, {"ok": False, "error": "POST /sense needs numeric true_mass_kg"})
        cap = float(payload.get("capacity_kg", MP.RM.REGOLITH_PER_CYCLE_KG))
        noise = float(payload.get("noise_frac", 0.0))      # 0 = noise OFF (deterministic)
        seed = int(payload.get("seed", 0))
        grid = [cap * f for f in (0.1, 0.25, 0.4, 0.55, 0.7, 0.85, 1.0)]
        sensor = MP.RM.DrumSensor.calibrated(grid, capacity_kg=cap, noise_frac=noise, seed=seed)
        current = sensor.current(true_mass)
        inferred = sensor.infer(current)
        dec = sensor.offload(inferred)
        return self._send_json(200, {
            "ok": True, "true_mass_kg": true_mass, "current_a": current, "inferred_kg": inferred,
            "uncertainty_frac": dec.uncertainty_frac, "lower_kg": dec.lower_kg, "upper_kg": dec.upper_kg,
            "capacity_kg": cap, "offload": dec.offload, "noise_frac": noise,
        })

    def log_message(self, *args):                          # quiet (no per-request stderr spam)
        pass


def make_server(port=0, host="127.0.0.1"):
    """A ThreadingHTTPServer. host defaults to localhost (tests + safe default); pass 0.0.0.0 to reach it
    from other devices on the LAN/tailnet. port=0 picks an ephemeral port (used by the tests)."""
    return ThreadingHTTPServer((host, port), Handler)


def main():
    ap = argparse.ArgumentParser(description="planet browser + mission planner server")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; use 0.0.0.0 to reach it over the LAN/tailnet (default localhost)")
    args = ap.parse_args()
    srv = make_server(args.port, args.host)
    host, port = srv.server_address
    print(f"planet browser + planner -> http://{host}:{port}/   (POST /plan, /sense; Ctrl-C to stop)",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        srv.shutdown()


if __name__ == "__main__":
    main()
