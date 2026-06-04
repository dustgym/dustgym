"""self_optimizing.py: a self-learning loop that learns the slip energy model from execution.

The planner's naive energy model is FLAT (135 J/m drive, slip = 0). The TRUE per-leg energy, from the
conserved slip dynamics (drive_step, with per-cell material), is inflated by slope: slip robs forward
progress (energy ~ 1/(1-slip)) and the rover climbs (m g sin). This pipeline EXECUTES drives over varied
slopes, OBSERVES the (slope -> true/flat energy inflation), LEARNS a generalizing inflation(slope) model
online, and shows the HELD-OUT prediction error SHRINKING -- the system getting better at predicting its
own behaviour from execution, then handing the planner an accurate per-leg energy (so plans are correctly
provisioned and can route around expensive grades).

Grounded: the dynamics are the conserved `drive_step` + slip ladder (+ Material), the energy is the IPEx
model; the ONLY learned thing is the inflation(slope) regression fit from execution. No synthetic data.
"""
from __future__ import annotations

import numpy as np

from . import constants as K
from . import drive
from . import ipex_specs as S
from .column_state import ColumnState

J_PER_M = S.drive_power_w() / S.DRIVE_SPEED_MS


def _tilted_cs(slope_deg: float, density: float, *, grid: int = 12, cell_m: float = 0.5) -> ColumnState:
    cols = np.arange(grid)[None, :] * np.ones((grid, 1))
    rise = np.tan(np.deg2rad(slope_deg)) * cell_m
    datum = (cols * rise).astype(float)
    return ColumnState(grid, grid, cell_m, mass_areal=np.full((grid, grid), 50.0),
                       density=np.full((grid, grid), float(density)),
                       state_label=np.zeros((grid, grid), np.uint8),
                       disturbance=np.zeros((grid, grid)), datum=datum)


def execute_leg_energy(slope_deg: float, density: float = K.RHO_SURFACE, *,
                       dist_m: float = 2.0, g: float = K.g) -> tuple[float, float]:
    """Drive a `dist_m` leg up a `slope_deg` patch (conserved slip dynamics, with Material); return
    (flat_J, true_J). flat_J is the planner's naive estimate; true_J adds slip loss + the gravity climb."""
    cs = _tilted_cs(slope_deg, density)
    mid = cs.mass_areal.shape[0] // 2
    _, _, telem = drive.drive_step(cs, (mid, mid), 0.0, 0.2, 0.0, material=True, g=g)
    s = min(float(telem["slip"]), 0.95)
    flat_J = dist_m * J_PER_M
    true_J = flat_J / (1.0 - s) + K.ROVER_MASS_DRY_KG * g * dist_m * np.sin(np.deg2rad(slope_deg))
    return float(flat_J), float(true_J)


class InflationModel:
    """Learned energy inflation(slope) = true/flat multiplier (quadratic; slip + climb both grow with
    slope). Fit online from observed (slope, inflation) by least squares; init = the naive flat plan (1.0)."""

    def __init__(self):
        self.X: list[float] = []
        self.Y: list[float] = []
        self.coef = np.array([0.0, 0.0, 1.0])              # polyval order: a*s^2 + b*s + c, init flat

    def observe(self, slope_deg: float, inflation: float) -> None:
        self.X.append(float(slope_deg))
        self.Y.append(float(inflation))

    def refit(self) -> None:
        if len(set(self.X)) >= 3:                          # need 3 distinct slopes for a quadratic fit
            self.coef = np.polyfit(self.X, self.Y, 2)

    def predict(self, slope_deg: float) -> float:
        return float(np.polyval(self.coef, slope_deg)) if len(set(self.X)) >= 3 else 1.0

    @property
    def n(self) -> int:
        return len(self.X)


def route_energy(slopes, model: "InflationModel | None" = None, *, dist_m: float = 2.0) -> float:
    """Predicted energy [J] of a route (per-leg slopes). model=None is the planner's naive flat estimate
    (slope-blind, so a flat and a steep route of equal distance look equal); a learned model corrects each
    leg by inflation(slope), so the planner can route AROUND expensive grades -- the optimization the
    self-learning unlocks."""
    return float(sum(dist_m * J_PER_M * (model.predict(s) if model is not None else 1.0) for s in slopes))


def run_self_optimizing(train_slopes, test_slopes, *, density: float = K.RHO_SURFACE, seed: int = 0):
    """Iterate over training slopes (shuffled): execute, observe, refit; after each, measure the HELD-OUT
    prediction error on test_slopes (distinct). Returns (history, model, test_truth). The held-out error
    shrinks as the model self-learns the inflation curve from execution."""
    rng = np.random.default_rng(seed)
    model = InflationModel()
    test_truth = {}
    for ts in test_slopes:                                 # the held-out ground truth, executed once
        fj, tj = execute_leg_energy(ts, density)
        test_truth[ts] = tj / fj
    order = list(train_slopes)
    rng.shuffle(order)
    history = []
    for i, s in enumerate(order):
        fj, tj = execute_leg_energy(s, density)
        model.observe(s, tj / fj)
        model.refit()
        errs = [abs(model.predict(ts) - test_truth[ts]) / test_truth[ts] for ts in test_slopes]
        history.append({"iter": i + 1, "n_obs": model.n, "slope_deg": float(s),
                        "held_out_mape": float(np.mean(errs))})
    return history, model, test_truth
