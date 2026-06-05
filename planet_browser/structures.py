"""structures.py — composite construction structures (the taxonomy nouns) -> mass-balanced cut/fill orders.

A *structure* is a named build placed at a site that decomposes into the planner's primitive cut/fill
orders (building_taxonomy.md §3), so the UI can offer "Landing Pad / Haul Road / Berm / ..." instead of
raw cut/fill. Balancing is by **volume** (density-invariant, so it holds on any body): a structure that
*consumes* material (berm, foundation, crater fill) pairs the fill with a cut that yields exactly that
volume, closing the conserved cut<->fill loop. Source/grade structures (borrow pit, road, flat pad,
trench) are cut-only; the mission_planner routes the surplus to spoil or other fills.

Each template ``fn(x, y, **params) -> [order dicts]``; an order dict is
``{action, kind: "cut"|"fill", x, y, footprint_m2, depth_m, note}`` matching
``mission_planner.BuildOrder`` / ``mission_from_dict``. ``x, y`` are the local site frame in meters.
"""
from __future__ import annotations

import math
import os
import sys

# single source for the regolith densities (terrain_authority at the monorepo root, planet_browser's parent)
_REPO_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from terrain_authority import constants as _K  # noqa: E402

# Bulking/swell (I7): excavation removes BANK (in-situ) material and places it as LOOSE spoil, which
# occupies MORE volume -> MASS is conserved, not volume. SWELL = loose fill volume per bank cut volume.
RHO_BANK = _K.RHO_DEEP        # in-situ (excavated) density [kg/m^3] (~1920)
RHO_LOOSE = _K.RHO_SPOIL      # loose placed spoil density [kg/m^3] (~1300)
SWELL = RHO_BANK / RHO_LOOSE  # ~1.48 (lunar): loose fill bulks above the bank cut. Planner-side
#                               approximation; the conserved authority (P8/I8) gives the exact per-column mass.


def _o(action, kind, x, y, footprint_m2, depth_m, note=""):
    # keep full float precision (don't round footprint) so a fill consumes EXACTLY the paired cut volume
    return {"action": action, "kind": kind, "x": float(x), "y": float(y),
            "footprint_m2": float(footprint_m2), "depth_m": float(depth_m), "note": note}


# -- balanced structures: the fill consumes exactly the paired cut (volume-conserved) ------------
def landing_pad(x, y, *, side_m=6.0, cut_depth_m=0.05, berm_height_m=0.12):
    """Level a square pad (bank cut) and build a perimeter blast berm from that material (loose fill, bulked)."""
    fill_vol = (side_m * side_m) * cut_depth_m * SWELL          # bank cut -> bulked loose fill (mass-conserved)
    return [
        _o("Level landing pad", "cut", x, y, side_m * side_m, cut_depth_m, f"{side_m:.0f}x{side_m:.0f} m"),
        _o("Perimeter blast berm", "fill", x + side_m / 2 + 4.0, y, fill_vol / berm_height_m, berm_height_m, "from pad cut (bulked)"),
    ]


def habitat_foundation(x, y, *, side_m=5.0, cut_depth_m=0.06, fill_height_m=0.06):
    """Cut a level footing (bank) and place a compacted raised pad from the spoil (loose fill, bulked)."""
    fill_vol = (side_m * side_m) * cut_depth_m * SWELL
    return [
        _o("Cut habitat footing", "cut", x, y, side_m * side_m, cut_depth_m),
        _o("Compacted foundation fill", "fill", x + side_m + 3.0, y, fill_vol / fill_height_m, fill_height_m, "from footing cut (bulked)"),
    ]


def blast_berm(x, y, *, length_m=15.0, width_m=3.0, height_m=0.5, borrow_depth_m=0.3):
    """A loose fill ridge supplied by a nearby borrow pit; the bank cut is sized so MASS balances."""
    borrow_vol = ((length_m * width_m) * height_m) / SWELL     # bank cut to supply the loose berm (mass-conserved)
    return [
        _o("Borrow pit (berm)", "cut", x - 12.0, y, borrow_vol / borrow_depth_m, borrow_depth_m, "source"),
        _o("Blast berm", "fill", x, y, length_m * width_m, height_m, "ridge to spec"),
    ]


def crater_fill(x, y, *, radius_m=8.0, depth_m=0.4, borrow_depth_m=0.3):
    """Fill a crater dip to grade from a nearby borrow pit; the bank cut is sized so MASS balances."""
    fill_area = math.pi * radius_m * radius_m
    borrow_vol = (fill_area * depth_m) / SWELL                 # bank cut to supply the loose fill (mass-conserved)
    return [
        _o("Borrow pit (crater)", "cut", x - 15.0, y, borrow_vol / borrow_depth_m, borrow_depth_m, "source"),
        _o("Crater fill", "fill", x, y, fill_area, depth_m, "to grade"),
    ]


# -- source / grade structures: cut-only (the planner routes the surplus) ------------------------
def borrow_pit(x, y, *, side_m=6.0, depth_m=0.3):
    return [_o("Borrow pit", "cut", x, y, side_m * side_m, depth_m, "material source")]


def haul_road(x, y, *, length_m=30.0, width_m=2.0, cut_depth_m=0.03):
    return [_o("Grade haul road", "cut", x, y, length_m * width_m, cut_depth_m, f"{length_m:.0f}x{width_m:.0f} m corridor")]


def solar_pad(x, y, *, side_m=8.0, cut_depth_m=0.04):
    return [_o("Level solar pad", "cut", x, y, side_m * side_m, cut_depth_m, "flat, low-obstruction")]


def trench(x, y, *, length_m=12.0, width_m=1.0, depth_m=0.4):
    return [_o("Excavate trench", "cut", x, y, length_m * width_m, depth_m, "utility/footing trench")]


STRUCTURES = {
    "landing_pad": landing_pad, "habitat_foundation": habitat_foundation,
    "blast_berm": blast_berm, "crater_fill": crater_fill,
    "borrow_pit": borrow_pit, "haul_road": haul_road, "solar_pad": solar_pad, "trench": trench,
}


def decompose(name, x, y, **params):
    """Return the cut/fill order dicts for structure ``name`` placed at (x, y) [local meters]."""
    fn = STRUCTURES.get(name)
    if fn is None:
        raise ValueError(f"unknown structure {name!r}; known: {sorted(STRUCTURES)}")
    return fn(float(x), float(y), **params)
