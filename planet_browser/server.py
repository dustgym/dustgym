#!/usr/bin/env python3
"""server.py -- ASGI server for the planet browser + mission planner (PRD N7/N8).

FastAPI/uvicorn. Serves the static front-end (index.html, bodies.json), the generated reports, and the
JSON API the browser drives: POST /plan, /sense, /structure, /compare, /render. Production hardening:
Pydantic request models (typed contract + input limits), optional API-key auth on the mutating routes,
CORS, a thread-safe (locked) matplotlib report path, a reports/ TTL sweep, structured access logging
(PRD N10), and /healthz + /metrics.

    python -m planet_browser.server [--port 8770] [--host 0.0.0.0]    # or the `dustgym-serve` entry point

Env knobs (PRD N15 overlay style): DUSTGYM_API_KEY (auth on POST when set), DUSTGYM_CORS_ORIGINS
(comma-list or *), DUSTGYM_REPORTS_TTL_S (report retention, default 86400), DUSTGYM_LOG_LEVEL.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import logging
import os
import re
import shutil
import sys
import threading
import time

import uvicorn
from fastapi import Depends, FastAPI, Header, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, ConfigDict, Field
from starlette.exceptions import HTTPException as StarletteHTTPException

from terrain_authority import config as CFG               # PO-02: configurable application-data dirs
from terrain_authority.io_fields import atomic_write_bytes  # PO-02/CT-04: atomic writes for profiles
from . import adaptive_planner as ADP
from . import autonomy as AUT
from . import mission_planner as MP
from . import structures as ST

# PRD N10: structured logging + observability. Used for access logs, startup, and the additive
# failure paths. Level via $DUSTGYM_LOG_LEVEL.
log = logging.getLogger("planet_browser.server")

_START = time.monotonic()
_REPORT_LOCK = threading.Lock()                 # matplotlib pyplot is process-global + thread-unsafe
_METRICS: dict = {"requests_total": 0, "by_status": {}, "by_route": {}}


def _configure_logging(level: str | None = None) -> None:
    """Configure logging for the server (PRD N10): level from arg, else $DUSTGYM_LOG_LEVEL, else INFO."""
    lvl = (level or os.environ.get("DUSTGYM_LOG_LEVEL", "INFO")).upper()
    logging.basicConfig(level=getattr(logging, lvl, logging.INFO),
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s", force=True)


# plan_render_pipeline lives in scripts/ (it drives the Godot sidecar); MP already put the repo root on the
# path. Importing it is optional -- /render degrades to a 503 if the binary is absent.
sys.path.insert(0, os.path.join(MP._REPO_ROOT, "scripts"))
try:
    import plan_render_pipeline as PRP
except Exception as _prp_exc:   # noqa: BLE001 -- /render just becomes unavailable
    PRP = None
    log.info("render pipeline unavailable (Godot sidecar import failed: %r); /render -> 503", _prp_exc)
_HAWORTH = os.path.join(MP._REPO_ROOT, "samples", "lunar_dem", "haworth_10km_5m")

HERE = os.path.dirname(os.path.abspath(__file__))
# PO-02/RB-06: reports + profiles live in the configurable application-data dir ($DUSTGYM_DATA_DIR,
# else ~/.local/share/dustgym) -- NOT inside the (possibly read-only) installed package. Tests
# monkeypatch these module-level vars to a tmp dir; run() writes reports to the same CFG.reports_dir().
REPORTS = CFG.reports_dir()
PROFILES = CFG.profiles_dir()                      # saved planning profiles (config snapshots), like reports/

_CTYPE = {".html": "text/html; charset=utf-8", ".json": "application/json",
          ".pdf": "application/pdf", ".md": "text/markdown; charset=utf-8",
          ".js": "text/javascript", ".css": "text/css", ".png": "image/png"}

_MAX_ORDERS = 1000   # N8 input limit: refuse absurd build queues before they reach the planner
_MAX_BODY_BYTES = int(os.environ.get("DUSTGYM_MAX_BODY_BYTES", 4 * 1024 * 1024))   # N8: request-body size cap (4 MiB)


def _version() -> str:
    try:
        from importlib.metadata import version
        return version("dustgym")
    except Exception:   # noqa: BLE001 -- not installed (editable/source run)
        return "0.1.0"


def _prune_reports(ttl_s: float | None = None) -> int:
    """Delete report files older than the TTL (default $DUSTGYM_REPORTS_TTL_S or 86400 s). Returns count."""
    ttl = float(ttl_s if ttl_s is not None else os.environ.get("DUSTGYM_REPORTS_TTL_S", 86400))
    if ttl <= 0 or not os.path.isdir(REPORTS):
        return 0
    now, removed = time.time(), 0
    for n in os.listdir(REPORTS):
        p = os.path.join(REPORTS, n)
        try:
            if os.path.isfile(p) and now - os.path.getmtime(p) > ttl:
                os.remove(p)
                removed += 1
        except OSError:
            pass
    return removed


def _totals_json(totals):
    """JSON-safe totals: numbers -> float, but pass through bools/strings (algorithm/objective) and already
    JSON-safe containers (e.g. vehicles_detail = a list of per-vehicle dicts) + None unchanged."""
    out = {}
    for k, v in totals.items():
        out[k] = float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v
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
        except Exception as e:   # noqa: BLE001 -- degrade to flat-check, but surface it
            log.warning("Haworth DEM unavailable; Moon plans fall back to flat slope-check: %r", e)
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
    except Exception as e:                             # noqa: BLE001 -- autonomy is additive, never break /plan
        log.warning("autonomy/perception block folded out (additive; /plan still served): %r", e)
        return None, None
    b, legs = cl["belief"], cl["legs"]
    nominal = sum(leg["nominal_J"] for leg in legs)
    true = sum(leg["true_J"] for leg in legs)
    energy = ADP.price_mission(legs, ADP.learned_model())   # self-learned slip energy applied to this plan
    autonomy = {
        "completed": cl["completed"], "n_trips": cl["n_trips"], "n_legs": len(legs),
        "recharges": cl["recharges"], "replans": cl["replans"],
        "perception_fixes": cl["perception_fixes"], "observe_more": cl["observe_more"],
        "final_soc": round(b.soc_frac(), 3),
        "max_slip": round(max((leg["slip"] for leg in legs), default=0.0), 3),
        "true_vs_nominal_energy": round(true / nominal, 3) if nominal else None,
        # self-optimizing: the LEARNED slip-energy model re-prices the plan toward the executed truth
        "energy_naive_kj": round(energy["naive_J"] / 1e3, 1),
        "energy_learned_kj": round(energy["learned_J"] / 1e3, 1),
        "energy_actual_kj": round(energy["actual_J"] / 1e3, 1),
    }
    leg_e_sig = max((leg["energy_sigma_J"] for leg in legs), default=0.0)
    mc = cl.get("map_channel", {})
    perception = {
        "pose_sigma_m": round(b.pos_sigma_m, 2),               # BOUNDED by the per-leg map/landmark fixes
        "map_fixes": cl["perception_fixes"],                   # pose corrections fused into the belief
        "observe_more_before_dig": cl["observe_more"],         # Uncertainty-layer dig-ready gate firings
        "fix_sigma_m": 0.10,                                   # SLAM/map-match fix precision (AprilTag 12.7 mm best-case)
        "energy_model_sigma_J": round(leg_e_sig, 1),           # slip model-error 1-sigma carried per leg
        "drum_fill_uncertainty_pct": 7.4,                      # FDC mass-inference MPE (2.56% >half full, 7.40% over range)
        # P6 / LAC section 10 map channel, closed into the loop: the executed route's worksite COVERAGE +
        # residual map uncertainty (onboard-observability tier), and the digs gated on local map coverage.
        "map_coverage": round(mc.get("coverage", 0.0), 3),
        "map_uncertainty_m": round(mc.get("mean_uncertainty_m", 0.0), 2),
        "map_observe_more_before_dig": cl.get("map_observe_more", 0),
        "map_survey_time_s": round(cl.get("survey_time_s", 0.0), 1),   # the survey-before-dig gate's real time cost
        "note": ("perception-in-the-loop: a map/landmark pose fix per leg bounds dead-reckoning drift; the "
                 "dig-ready gate observes more before digging when the pose is uncertain OR the dig site's "
                 "local map coverage is low. map_coverage is the onboard-observability tier (what the route "
                 "sees) -- the dense observed-map RMSE is the gated render/COLMAP tier (see /render)."),
    }
    return autonomy, perception


def _plan_stem(payload):
    """Stable, collision-free report stem from the mission (name slug + content hash) -- repeatable,
    no wall-clock, so the same queue regenerates the same file instead of piling up duplicates."""
    import json
    import re
    name = re.sub(r"[^a-z0-9]+", "-", str(payload.get("name", "mission")).lower()).strip("-") or "mission"
    digest = hashlib.sha1(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:8]
    return f"{name}-{digest}"


# --------------------------------------------------------------------------------------------------
# Request models (PRD N8: the typed API contract + input limits). extra="allow" passes through the
# optional per-kind order fields the planner reads; the limits below cap obviously-abusive inputs.
# --------------------------------------------------------------------------------------------------
class Order(BaseModel):
    model_config = ConfigDict(extra="allow")
    action: str | None = Field(default=None, max_length=120)
    kind: str | None = Field(default=None, max_length=40)
    x: float = 0.0
    y: float = 0.0
    footprint_m2: float = Field(default=1.0, gt=0, le=1e8)
    depth_m: float = Field(default=0.0, ge=-100.0, le=100.0)


class PlanRequest(BaseModel):
    model_config = ConfigDict(extra="allow")
    name: str = Field(default="mission", max_length=200)
    body: str = Field(default="moon", max_length=40)
    orders: list[Order] = Field(default_factory=list, max_length=_MAX_ORDERS)
    algorithm: str = Field(default="nearest", max_length=40)
    objective: str = Field(default="time", max_length=40)
    lat: float | None = Field(default=None, ge=-90.0, le=90.0)   # M11: globe site-pick -> order-frame anchor
    lon: float | None = Field(default=None, ge=-360.0, le=360.0)
    vehicles: int = Field(default=1, ge=1, le=16)               # MV: fleet size (>1 -> multi-vehicle plan)


class CompareRequest(BaseModel):
    model_config = ConfigDict(extra="allow")
    name: str = Field(default="mission", max_length=200)
    body: str = Field(default="moon", max_length=40)
    orders: list[Order] = Field(default_factory=list, max_length=_MAX_ORDERS)
    objective: str = Field(default="time", max_length=40)


class SenseRequest(BaseModel):
    true_mass_kg: float = Field(ge=0.0, le=1e5)
    capacity_kg: float | None = Field(default=None, gt=0.0, le=1e5)
    noise_frac: float = Field(default=0.0, ge=0.0, le=1.0)
    seed: int = Field(default=0, ge=0)


class StructureRequest(BaseModel):
    model_config = ConfigDict(extra="allow")
    name: str | None = Field(default=None, max_length=80)
    x: float = 0.0
    y: float = 0.0
    params: dict = Field(default_factory=dict)


class RenderRequest(BaseModel):
    u: float = Field(default=0.5, ge=0.0, le=1.0)
    v: float = Field(default=0.5, ge=0.0, le=1.0)
    pad_frac: float = Field(default=0.5, gt=0.0, le=1.0)


class ProfileRequest(BaseModel):
    name: str = Field(min_length=1, max_length=80)      # saved under a slug of this name
    profile: dict = Field(default_factory=dict)         # the full config snapshot (body/soil/fleet/orders/...)


def require_auth(x_api_key: str | None = Header(default=None, alias="X-API-Key"),
                 authorization: str | None = Header(default=None)):
    """N8: API-key auth on mutating routes. Enabled only when $DUSTGYM_API_KEY is set (open in dev).
    Accepts `X-API-Key: <key>` or `Authorization: Bearer <key>`."""
    key = os.environ.get("DUSTGYM_API_KEY")
    if not key:
        return
    supplied = x_api_key or (authorization or "").removeprefix("Bearer ").strip()
    if not hmac.compare_digest(supplied.encode(), key.encode()):   # constant-time -> no timing oracle
        raise HTTPException(status_code=401, detail="invalid or missing API key")


app = FastAPI(title="dustgym planet browser + mission planner", version=_version())

_cors = os.environ.get("DUSTGYM_CORS_ORIGINS", "*").strip()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"] if _cors == "*" else [o.strip() for o in _cors.split(",") if o.strip()],
    allow_methods=["*"], allow_headers=["*"],
)


@app.middleware("http")
async def _access_log(request: Request, call_next):
    t0 = time.monotonic()
    # N8: reject oversized bodies up front (Content-Length guard) before they reach a handler.
    if request.method in ("POST", "PUT", "PATCH"):
        try:
            clen = int(request.headers.get("content-length") or 0)
        except ValueError:
            clen = 0
        if clen > _MAX_BODY_BYTES:
            return JSONResponse(status_code=413,
                                content={"ok": False, "error": f"request body too large (> {_MAX_BODY_BYTES} bytes)"})
    response = await call_next(request)
    dt = (time.monotonic() - t0) * 1000.0
    raw = request.url.path
    # key the by_route metric on the MATCHED ROUTE TEMPLATE (e.g. /figure/{key}), not the raw client path:
    # the raw path is attacker-controlled, so an unbounded dict would be a memory-DoS. Templates are finite.
    matched = request.scope.get("route")
    route_key = getattr(matched, "path", "unmatched")
    _METRICS["requests_total"] += 1
    sk = str(response.status_code)
    _METRICS["by_status"][sk] = _METRICS["by_status"].get(sk, 0) + 1
    _METRICS["by_route"][route_key] = _METRICS["by_route"].get(route_key, 0) + 1
    log.info('%s "%s %s" %s %.1fms',
             request.client.host if request.client else "-", request.method, raw, response.status_code, dt)
    return response


@app.exception_handler(RequestValidationError)
async def _on_validation_error(request: Request, exc: RequestValidationError):
    """Surface Pydantic validation failures in the {ok:false,error} envelope at 400 (not FastAPI's 422
    default), preserving the API contract. Malformed JSON is reported as 'bad JSON'."""
    errs = exc.errors()
    if any(e.get("type") == "json_invalid" for e in errs):
        return JSONResponse(status_code=400, content={"ok": False, "error": f"bad JSON: {errs[0].get('msg', '')}"})
    msg = "; ".join(f"{'.'.join(str(p) for p in e['loc'][1:]) or e['loc'][-1]}: {e['msg']}" for e in errs[:3])
    return JSONResponse(status_code=400, content={"ok": False, "error": msg or "invalid request"})


@app.exception_handler(StarletteHTTPException)
async def _on_http_exc(request: Request, exc: StarletteHTTPException):
    return JSONResponse(status_code=exc.status_code, content={"ok": False, "error": exc.detail})


# ---- GET: static front-end + generated reports + DEM previews + ops ------------------------------
@app.get("/")
@app.get("/index.html")
def get_index():
    return FileResponse(os.path.join(HERE, "index.html"), media_type=_CTYPE[".html"])


@app.get("/bodies.json")
def get_bodies():
    p = os.path.join(HERE, "bodies.json")
    if not os.path.isfile(p):
        return JSONResponse(status_code=404, content={"ok": False, "error": "not found: bodies.json"})
    return FileResponse(p, media_type=_CTYPE[".json"])


@app.get("/reports/{name}")
def get_report(name: str):
    safe = os.path.basename(name)                       # basename only -> no path traversal
    p = os.path.join(REPORTS, safe)
    if not os.path.isfile(p):
        return JSONResponse(status_code=404, content={"ok": False, "error": f"not found: {safe}"})
    ext = os.path.splitext(safe)[1]
    return FileResponse(p, media_type=_CTYPE.get(ext, "application/octet-stream"))


@app.get("/dem/{name}")
def get_dem(name: str):                                 # the real LOLA work-area DEM previews (Haworth)
    bundle = os.path.join(HERE, "..", "samples", "lunar_dem", "haworth_10km_5m")
    f = {"hillshade.png": "preview_hillshade.png", "height.png": "preview_height.png"}.get(os.path.basename(name))
    if not f:
        return JSONResponse(status_code=404, content={"ok": False, "error": f"no dem {os.path.basename(name)}"})
    path = os.path.join(bundle, f)
    if not os.path.isfile(path):                        # bundle absent (e.g. a wheel install) -> 404, not a 500
        return JSONResponse(status_code=404, content={"ok": False, "error": f"dem preview not available: {f}"})
    return FileResponse(path, media_type="image/png")


# ---- engineer/developer/intern panes: validation figures + runtime config (served from source) ---
_VALIDATION = os.path.join(MP._REPO_ROOT, "validation")


def _validation_figures() -> dict:
    """Map 'category/file.png' -> absolute path for every PNG under validation/. Served from the source
    tree (empty if absent, e.g. a wheel install). The returned keys are the allowlist -> traversal-proof."""
    out: dict = {}
    if not os.path.isdir(_VALIDATION):
        return out
    for root, _dirs, files in os.walk(_VALIDATION):
        for fn in sorted(files):
            if fn.endswith(".png"):
                rel = os.path.relpath(os.path.join(root, fn), _VALIDATION).replace(os.sep, "/")
                out[rel] = os.path.join(root, fn)
    return out


@app.get("/figures")
def get_figures():
    """List the validation figures (engineer pane). key = 'category/file.png'; fetch via /figure/{key}."""
    figs = _validation_figures()
    return {"ok": True, "figures": [{"key": k, "category": k.split("/")[0], "url": "/figure/" + k}
                                    for k in sorted(figs)]}


@app.get("/figure/{key:path}")
def get_figure(key: str):
    """Serve a validation PNG by allowlisted key (only the keys /figures lists -> no path traversal)."""
    p = _validation_figures().get(key)
    if not p:
        return JSONResponse(status_code=404, content={"ok": False, "error": f"no figure {key}"})
    return FileResponse(p, media_type="image/png")


@app.get("/config")
def get_config():
    """Runtime config overlay state (intern/dev pane): config_file + overrides + applied (PRD N15)."""
    from terrain_authority import config as _cfg
    return {"ok": True, **_cfg.describe()}


@app.get("/healthz")
def healthz():
    return {"status": "ok", "version": _version(), "uptime_s": round(time.monotonic() - _START, 1)}


@app.get("/metrics")
def metrics():
    return {"uptime_s": round(time.monotonic() - _START, 1), **_METRICS}


# ---- profiles: save / list / load a planning config snapshot ------------------------------------
def _profile_slug(name: str) -> str:
    return re.sub(r"[^a-z0-9_-]+", "-", str(name).lower()).strip("-") or "profile"


@app.post("/profile")
def post_profile(req: ProfileRequest, _auth: None = Depends(require_auth)):
    """Save a planning profile (the full config snapshot) under a slug of its name, to profiles/."""
    os.makedirs(PROFILES, exist_ok=True)
    slug = _profile_slug(req.name)
    atomic_write_bytes(os.path.join(PROFILES, slug + ".json"),            # PO-02: atomic, no partial profile
                       json.dumps({"name": req.name, "profile": req.profile}, indent=2).encode("utf-8"))
    return {"ok": True, "name": slug}


@app.get("/profiles")
def get_profiles():
    """List the saved profile slugs."""
    if not os.path.isdir(PROFILES):
        return {"ok": True, "profiles": []}
    return {"ok": True, "profiles": sorted(os.path.splitext(f)[0]
                                           for f in os.listdir(PROFILES) if f.endswith(".json"))}


@app.get("/profile/{name}")
def get_profile(name: str):
    """Load a saved profile by slug -> {name, profile}."""
    slug = _profile_slug(name)
    p = os.path.join(PROFILES, slug + ".json")
    if not os.path.isfile(p):
        return JSONResponse(status_code=404, content={"ok": False, "error": f"no profile {slug!r}"})
    with open(p) as fh:
        return json.load(fh)


# ---- POST: the planner API (auth-gated when $DUSTGYM_API_KEY is set) -----------------------------
@app.post("/plan")
def post_plan(req: PlanRequest, _auth: None = Depends(require_auth)):
    _prune_reports()
    payload = req.model_dump(exclude_unset=True)
    try:
        mission = MP.mission_from_dict(payload)
        if mission.body == "moon":
            dem, origin = _moon_dem()                  # (dem, auto flattest anchor)
            if req.lat is not None and req.lon is not None:   # M11: a globe site-pick overrides the anchor
                try:
                    origin = MP.latlon_to_dem_origin(req.lat, req.lon)
                except ImportError:
                    log.warning("pyproj absent ([planner] extra); site lat/lon ignored, using flattest anchor")
                except ValueError as e:
                    return JSONResponse(status_code=400, content={"ok": False, "error": str(e)})
        else:
            dem, origin = None, (0.0, 0.0)
        # RB-03: compute the plan ONCE (incl. as-built validation + endurance); report/timeline/IR and the
        # validation/endurance fields are all VIEWS of this single result (no independent recompute).
        result = MP.plan(mission, dem=dem, dem_origin=origin, algorithm=req.algorithm,
                         objective=req.objective, vehicles=req.vehicles, with_acceptance=True)
        # I10: hauls routed around hazards on the real DEM; I8 + I6/M11 slope-feasible siting.
        with _REPORT_LOCK:                              # serialize the thread-unsafe matplotlib report path
            pdf, md, totals = MP.run(mission, stem=_plan_stem(payload), dem=dem, dem_origin=origin,
                                     algorithm=req.algorithm, objective=req.objective,
                                     vehicles=req.vehicles, result=result)
        validation = result.validation                  # RB-03: from the one result, not a recompute
        timeline = MP.build_timeline(mission, dem=dem, dem_origin=origin,
                                     algorithm=req.algorithm, objective=req.objective, result=result)
        endurance = result.endurance
        autonomy, perception = _autonomy_perception(mission, dem, origin, req.algorithm, req.objective)
        plan_ir = MP.plan_ir(mission, dem=dem, dem_origin=origin,                # the machine-executable plan
                             algorithm=req.algorithm, objective=req.objective,
                             vehicles=req.vehicles, result=result)
    except (ValueError, RuntimeError) as e:             # bad input / sinter-gated -> honest 400
        return JSONResponse(status_code=400, content={"ok": False, "error": str(e)})
    return {
        "ok": True,
        "pdf": "/reports/" + os.path.basename(pdf),
        "md": "/reports/" + os.path.basename(md),
        "totals": _totals_json(totals),
        "validation": validation,
        "timeline": timeline,
        "endurance": endurance,
        "autonomy": autonomy,
        "perception": perception,
        "plan_ir": plan_ir,                             # versioned typed-action plan a rover/ROS executive runs
        "provenance": result.provenance,                # RB-03/CT-07: schema, mode, config, input hash of THE plan
    }


@app.post("/compare")
def post_compare(req: CompareRequest, _auth: None = Depends(require_auth)):
    payload = req.model_dump(exclude_unset=True)
    try:
        mission = MP.mission_from_dict(payload)
        dem, origin = _moon_dem() if mission.body == "moon" else (None, (0.0, 0.0))
        result = MP.compare_algorithms(mission, objective=req.objective, dem=dem, dem_origin=origin)
    except (ValueError, RuntimeError) as e:
        return JSONResponse(status_code=400, content={"ok": False, "error": str(e)})
    return {"ok": True, **result}


@app.post("/structure")
def post_structure(req: StructureRequest, _auth: None = Depends(require_auth)):
    """Decompose a named structure (Landing Pad / Haul Road / Berm / ...) at (x,y) into mass-balanced
    cut/fill orders (structures.decompose). Returns orders the build queue can adopt."""
    if len(req.params or {}) > 32:                      # N8: cap the param dict (decompose also rejects unknown keys)
        return JSONResponse(status_code=400, content={"ok": False, "error": "too many structure params (max 32)"})
    try:
        orders = ST.decompose(req.name, req.x, req.y, **(req.params or {}))
    except (ValueError, TypeError) as e:
        return JSONResponse(status_code=400, content={"ok": False, "error": str(e)})
    return {"ok": True, "name": req.name, "orders": orders}


@app.post("/sense")
def post_sense(req: SenseRequest, _auth: None = Depends(require_auth)):
    """Drum-fill sensing (ICE-RASSOR): true drum mass -> motor-current observable -> inferred mass +
    offload decision. `noise_frac` toggles seeded sensor noise (0 = OFF)."""
    cap = req.capacity_kg if req.capacity_kg is not None else float(MP.RM.REGOLITH_PER_CYCLE_KG)
    grid = [cap * f for f in (0.1, 0.25, 0.4, 0.55, 0.7, 0.85, 1.0)]
    sensor = MP.RM.DrumSensor.calibrated(grid, capacity_kg=cap, noise_frac=req.noise_frac, seed=req.seed)
    current = sensor.current(req.true_mass_kg)
    inferred = sensor.infer(current)
    dec = sensor.offload(inferred)
    return {
        "ok": True, "true_mass_kg": req.true_mass_kg, "current_a": current, "inferred_kg": inferred,
        "uncertainty_frac": dec.uncertainty_frac, "lower_kg": dec.lower_kg, "upper_kg": dec.upper_kg,
        "capacity_kg": cap, "offload": dec.offload, "noise_frac": req.noise_frac,
    }


@app.post("/render")
def post_render(req: RenderRequest, _auth: None = Depends(require_auth)):
    """Crop a Haworth window at the picked (u,v), plan a flatten, render BEFORE/AFTER in Godot, and
    return the figure URL + earthwork volumes. Slow (two Godot renders); 503 if the binary is absent."""
    if PRP is None:
        return JSONResponse(status_code=503,
                            content={"ok": False, "error": "render pipeline unavailable (Godot binary absent)"})
    _prune_reports()
    stem = "render_" + hashlib.sha1(f"{req.u:.4f}_{req.v:.4f}_{req.pad_frac:.2f}".encode()).hexdigest()[:10]
    try:
        with _REPORT_LOCK:
            r = PRP.render_map_area(_HAWORTH, req.u, req.v, os.path.join(REPORTS, stem), pad_frac=req.pad_frac)
    except Exception as e:                              # noqa: BLE001 -- render failure -> honest 500
        log.exception("render failed for (u=%s, v=%s)", req.u, req.v)
        return JSONResponse(status_code=500, content={"ok": False, "error": f"render failed: {e}"})
    fig_name = stem + ".png"
    shutil.copyfile(r["figure"], os.path.join(REPORTS, fig_name))
    return {
        "ok": True, "figure": "/reports/" + fig_name,
        "cut_vol_m3": round(r["cut_vol_m3"], 2), "fill_vol_m3": round(r["fill_vol_m3"], 2),
        "cut_kg": round(r["cut_kg"]), "extent_m": round(r["extent_m"], 1), "cell_m": round(r["cell_m"], 2),
    }


# ---- catch-all 404s (registered last) keep the {ok:false,error} envelope ------------------------
@app.get("/{path:path}")
def _no_get(path: str):
    return JSONResponse(status_code=404, content={"ok": False, "error": f"no route /{path}"})


@app.post("/{path:path}")
def _no_post(path: str):
    return JSONResponse(status_code=404, content={"ok": False, "error": f"no route /{path}"})


def main():
    ap = argparse.ArgumentParser(description="planet browser + mission planner server (ASGI)")
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; use 0.0.0.0 to reach it over the LAN/tailnet (default localhost)")
    args = ap.parse_args()
    _configure_logging()
    _prune_reports()
    log.info("planet browser + planner (ASGI) -> http://%s:%s/   (POST /plan,/sense; /healthz,/metrics; Ctrl-C)",
             args.host, args.port)
    uvicorn.run(app, host=args.host, port=args.port, log_config=None)


if __name__ == "__main__":
    main()
