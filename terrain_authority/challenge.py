"""Challenge system (M1 / H1-H3) — declarative lunar-construction challenges.

A *challenge* is one declarative object that doubles as a SimCity-style scenario, an RL
curriculum task, and a LAC-style competition task:

    challenge = { map (seed + base + DR envelope), objective (+ target terrain),
                  constraints, scoring }

`realize(challenge)` deterministically turns a challenge into a concrete instance: a
mass-conserving `ColumnState` map (seeded procedural terrain + domain-randomized soil)
plus the objective's `target_height` (the H_target the agent must match). Scoring is a
terrain-matching RMSE over the work region. The agent never edits terrain directly (the
physics authority does), so scores are unhackable and runs are reproducible.

This module is the framework (schema + generator + scorer). The env that runs an agent
in it is `terrain_target_env.py`; the runner is `challenge_runner.py`.
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field

import numpy as np

from . import terramechanics as tm
from .column_state import ColumnState

OBJECTIVE_TYPES = ("traverse", "flatten_pad", "build_berm")
BASE_TYPES = ("flat", "ramp", "bumps", "crater", "mound")


# ---------------------------------------------------------------------------
# Declarative schema (frozen, JSON round-trippable)
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MapSpec:
    seed: int = 0
    base: str = "flat"            # flat | ramp | bumps | crater
    grid: int = 64
    cell_m: float = 0.02
    slope_deg: float = 0.0        # for base="ramp"
    roughness_m: float = 0.005    # seeded micro-roughness amplitude (randomizes the map)
    randomize_soil: bool = True   # DR the terramechanics params from the seed
    sun_azimuth_deg: float = 0.0
    sun_elevation_deg: float = 7.0

    def to_dict(self):
        return dataclasses.asdict(self)


@dataclass(frozen=True)
class Objective:
    type: str                                   # traverse | flatten_pad | build_berm
    region: tuple                               # (r0, c0, r1, c1) work-zone bbox in cells
    goal_rc: tuple | None = None                # target cell for traverse
    target_delta_m: float = 0.05                # berm ridge height
    tolerance_m: float = 0.01                   # success H-RMSE threshold


@dataclass(frozen=True)
class Constraints:
    max_time_steps: int = 200
    max_slip_events: int = 1_000_000
    payload_kg_limit: float = 30.0


@dataclass(frozen=True)
class Scoring:
    w_time: float = 0.001
    w_slip: float = 0.1
    w_energy: float = 0.0


@dataclass(frozen=True)
class Challenge:
    id: str
    name: str
    difficulty_tier: int
    map: MapSpec
    objective: Objective
    constraints: Constraints = Constraints()
    scoring: Scoring = Scoring()
    heldout_seeds: tuple = ()

    def to_dict(self):
        return dataclasses.asdict(self)

    def to_json(self, *, indent=2):
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)

    @classmethod
    def from_dict(cls, d: dict) -> "Challenge":
        o = d["objective"]
        objective = Objective(
            type=o["type"],
            region=tuple(o["region"]),
            goal_rc=(tuple(o["goal_rc"]) if o.get("goal_rc") is not None else None),
            target_delta_m=o.get("target_delta_m", 0.05),
            tolerance_m=o.get("tolerance_m", 0.01),
        )
        return cls(
            id=d["id"], name=d["name"], difficulty_tier=d["difficulty_tier"],
            map=MapSpec(**d["map"]),
            objective=objective,
            constraints=Constraints(**d.get("constraints", {})),
            scoring=Scoring(**d.get("scoring", {})),
            heldout_seeds=tuple(d.get("heldout_seeds", ())),
        )

    @classmethod
    def from_json(cls, s_or_path: str) -> "Challenge":
        try:
            return cls.from_dict(json.loads(s_or_path))
        except (ValueError, TypeError):
            with open(s_or_path) as fh:
                return cls.from_dict(json.load(fh))


# ---------------------------------------------------------------------------
# Realized instance (runtime; holds numpy arrays)
# ---------------------------------------------------------------------------

@dataclass
class ChallengeInstance:
    challenge: Challenge
    cs: ColumnState
    base_height: np.ndarray
    target_height: np.ndarray | None     # None for traverse
    goal_rc: tuple | None
    params: "tm.TerramechanicsParams"
    objective: Objective


# ---------------------------------------------------------------------------
# Generator (H2): seed -> concrete map + target, deterministically
# ---------------------------------------------------------------------------

def _build_datum(base: str, grid: int, cell_m: float, slope_deg: float,
                 roughness_m: float, rng) -> np.ndarray:
    """Deterministic (seeded) terrain datum [m] for the requested base type."""
    rr, cc = np.mgrid[0:grid, 0:grid].astype(np.float64)
    if base == "flat":
        z = np.zeros((grid, grid))
    elif base == "ramp":
        z = np.tan(np.radians(slope_deg)) * cc * cell_m            # rises along +col
    elif base == "bumps":
        z = 0.03 * np.sin(rr / 5.0) * np.cos(cc / 5.0)             # ~3 cm undulation to flatten
    elif base == "crater":
        cy = cx = grid / 2.0
        r = np.hypot(rr - cy, cc - cx)
        z = -0.15 * np.exp(-(r / (grid * 0.12)) ** 2)             # central depression
    elif base == "mound":
        cy = cx = grid / 2.0
        r = np.hypot(rr - cy, cc - cx)
        z = 0.20 * np.exp(-(r / (grid * 0.10)) ** 2)              # central mound (cut-dominant)
    else:
        raise ValueError(f"unknown map base {base!r} (expected one of {BASE_TYPES})")
    if roughness_m > 0.0:
        z = z + rng.normal(0.0, roughness_m, (grid, grid))         # seeded -> randomizes the map
    return z.astype(np.float64)


def realize(challenge: Challenge) -> ChallengeInstance:
    """Deterministically realize a challenge into a map + target. Same challenge ->
    identical ColumnState (mass, datum), DR params, and target_height."""
    if challenge.objective.type not in OBJECTIVE_TYPES:
        raise ValueError(f"unknown objective type: {challenge.objective.type!r} "
                         f"(expected one of {OBJECTIVE_TYPES})")
    m = challenge.map
    rng = np.random.default_rng(m.seed)
    params = (tm.domain_randomize(rng) if m.randomize_soil
              else tm.TerramechanicsParams.from_constants())
    datum = _build_datum(m.base, m.grid, m.cell_m, m.slope_deg, m.roughness_m, rng)
    cs = ColumnState(width=m.grid, height=m.grid, cell_m=m.cell_m, datum=datum)
    base_height = cs.derive_height().copy()

    obj = challenge.objective
    r0, c0, r1, c1 = obj.region
    if obj.type == "traverse":
        target = None
    elif obj.type == "flatten_pad":
        target = base_height.copy()
        target[r0:r1, c0:c1] = base_height[r0:r1, c0:c1].mean()    # flat at region mean (mass-neutral)
    elif obj.type == "build_berm":
        target = base_height.copy()
        target[r0:r1, c0:c1] = base_height[r0:r1, c0:c1] + obj.target_delta_m
    else:  # pragma: no cover - guarded above
        raise ValueError(obj.type)

    return ChallengeInstance(challenge=challenge, cs=cs, base_height=base_height,
                             target_height=target, goal_rc=obj.goal_rc, params=params,
                             objective=obj)


# ---------------------------------------------------------------------------
# Scoring (H3 core): terrain-matching RMSE over the work region
# ---------------------------------------------------------------------------

def terrain_rmse(achieved: np.ndarray, target: np.ndarray, region: tuple) -> float:
    """Root-mean-square height error [m] between achieved and target over the region bbox."""
    r0, c0, r1, c1 = region
    d = np.asarray(achieved)[r0:r1, c0:c1] - np.asarray(target)[r0:r1, c0:c1]
    return float(np.sqrt(np.mean(d * d)))


def authored_challenges() -> list[Challenge]:
    """A small v1 library spanning the difficulty ladder (traverse / flatten / berm)."""
    return [
        Challenge(id="c1_traverse", name="Traverse a rough patch", difficulty_tier=1,
                  map=MapSpec(seed=1, base="bumps", grid=64),
                  objective=Objective(type="traverse", region=(0, 0, 64, 64), goal_rc=(32, 56),
                                      tolerance_m=0.0)),
        Challenge(id="c2_flatten", name="Flatten a construction pad", difficulty_tier=2,
                  map=MapSpec(seed=2, base="bumps", grid=64),
                  objective=Objective(type="flatten_pad", region=(20, 20, 44, 44),
                                      tolerance_m=0.01)),
        Challenge(id="c3_berm", name="Build a berm to spec", difficulty_tier=3,
                  map=MapSpec(seed=3, base="flat", grid=64),
                  objective=Objective(type="build_berm", region=(28, 24, 36, 40),
                                      target_delta_m=0.08, tolerance_m=0.02)),
    ]
