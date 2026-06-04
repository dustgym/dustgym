"""adaptive_planner.py: deploy the self-learned slip energy model to price real missions.

Trains the `inflation(slope)` model once from controlled drives (terrain_authority.self_optimizing) and
uses it to RE-PRICE a planned mission's legs -- so the planner predicts the TRUE slip-inflated energy
(correct provisioning) instead of the naive flat estimate. The model is trained on controlled grades and
GENERALIZES to the mission's actual leg slopes; this is the self-learning loop closing on the real planner.
"""
from __future__ import annotations

from terrain_authority import self_optimizing as SO

_MODEL = None


def learned_model():
    """The inflation(slope) model, trained once from controlled drives over a spread of grades (cached)."""
    global _MODEL
    if _MODEL is None:
        _, _MODEL, _ = SO.run_self_optimizing([2, 5, 8, 12, 16, 20, 24, 28], [10], seed=0)
    return _MODEL


def price_mission(legs, model=None) -> dict:
    """Price a closed-loop mission's legs three ways: naive (flat nominal, slip-blind), learned
    (slope-corrected by the model), and actual (the executed slip-inflated truth). The learned price
    tracks the actual; the naive under-prices the sloped legs."""
    naive = sum(L["nominal_J"] for L in legs)
    # the slip model inflates only the DRIVE portion (nominal - dig); dig energy is mass-fixed, slip-blind.
    learned = sum(L.get("dig_e", 0.0)
                  + (L["nominal_J"] - L.get("dig_e", 0.0)) * (model.predict(L["slope_deg"]) if model is not None else 1.0)
                  for L in legs)
    actual = sum(L["true_J"] for L in legs)
    return {"naive_J": float(naive), "learned_J": float(learned), "actual_J": float(actual)}
