"""rassor_mass_model.py -- drum regolith-mass inference + arm-lift energy, grounded in real ICE-RASSOR data.

Source [ICE-RASSOR-MASS]:
  N. A. Janmohamed, J. M. Cloud, K. W. Leucht, E. A. Bell, B. C. Buckles, M. A. DuPuis,
  "Mass Inferencing Model Creation and Deployment to the RASSOR Lunar Excavation Robot",
  AIAA ASCEND 2021 (presented 17 Nov 2021). NTRS 20210022781. NASA KSC Intelligent Capabilities
  Enhanced RASSOR (ICE-RASSOR), IR&TD program. Work of the U.S. Government, not subject to copyright
  (public domain). https://ntrs.nasa.gov/citations/20210022781
Companion overview [ICE-RASSOR]: NTRS 20210021455 (excavate / scoop / haul / dump / process actions).

The 2020/2021 RASSOR had NO load cell: drum regolith mass was INFERRED from existing motor telemetry
(arm/drum motor position, velocity, current, voltage, and robot pose) -- no added hardware, hence no new
failure modes. Three model STRUCTURES were developed; the Arm-Raise (AR) and Free-spinning Drum Current
(FDC) models were validated on hardware against a load-cell measurement hoist (Fig 3):

  * Arm-Raise (AR): drum mass is LINEAR in the integrated arm-motor power during an arm raise/lower --
    lifting the loaded drum is gravity work. Front R^2 = 0.996, rear R^2 = 0.974 (Fig 6).
  * Free-spinning Drum Current (FDC): at constant drum velocity, the steady (non-digging) average drum
    current rises with the carried mass; mass is LINEAR in that average current. Front R^2 = 0.989,
    rear R^2 = 0.985 (Fig 8). A neural-net augmentation maps (drum velocity, average current) -> mass to
    remove the velocity dependence (Fig 10) and was integrated in the flight-ready system. The LINEAR
    FDC model had the best hardware accuracy: mean percent error 7.40% over the mass range (11.84% with
    two outlier dig cycles), and 2.56% when the drum is more than half full (> ~20 kg).
  * Excavation Drum Current (EDC): mass excavated per dig cycle is linear in the aggregated drum current
    over the cycle; lower fidelity, R^2 = 0.76 (Fig 12).

WHAT WE TAKE, honestly: the model STRUCTURE (linear), the validated QUALITY (R^2, mean percent error),
and the realistic drum-fill KNOWLEDGE UNCERTAINTY a motor-current estimate carries -- so the autonomy
layer plans against imperfect drum-fill knowledge (e.g. "go offload at the processing plant when the
drums read full") instead of assuming exact mass. The exact fit COEFFICIENTS are NOT published (figures
only) and are RASSOR-hardware / 1-g-simulant specific -- the paper's own Future Work calls for lunar /
low-g recalibration -- so this module does NOT hard-code fabricated slopes/intercepts: LinearMassModel
must be calibrated from data (.fit) before it predicts. The arm-raise term is grounded from first
principles (gravity work, gravity-aware via bodies.py g), which is physically what the AR model's
R^2 = 0.996 linearity reflects.
"""
from __future__ import annotations

import dataclasses
import random

from . import ipex_specs as _ipex

# ---- published quality metrics (verbatim from NTRS 20210022781) -------------------------------
AR_LINEAR_R2 = (0.996, 0.974)      # (front, rear), Fig 6
FDC_LINEAR_R2 = (0.989, 0.985)     # (front, rear), Fig 8 (window length 4)
EDC_R2 = 0.7601                    # rear, Fig 12
FDC_MPE_ALL = 0.07403              # linear FDC mean percent error over the mass range (excl. 2 outliers)
FDC_MPE_WITH_OUTLIERS = 0.11842    # incl. dig cycles 1 and 3
FDC_MPE_HALF_FULL = 0.02558        # when drum mass > ~half full (> 20 kg)
HALF_FULL_KG = 20.0                # the paper's ">20 kg" low-error threshold (drum cycle capacity ~30 kg)
REGOLITH_PER_CYCLE_KG = _ipex.REGOLITH_PER_CYCLE_KG   # ~30 kg/cycle drum capacity (single source: ipex_specs)
EARTH_G = 9.81                     # [m/s^2] reference gravity for the 1-g RASSOR calibration

# Free-spinning drum-current forward model = the sensing observable. [CALIB], approximate, read off-plot
# from NTRS 20210022781 Fig 7 (empty/pre-dig baseline ~1.75 A) + Fig 8 (mass-vs-current slope ~1 A / ~31 kg)
# at 1-g in lunar simulant. The STRUCTURE is grounded (current rises linearly with carried mass); the
# magnitudes are figure-read estimates, NOT published coefficients -> honest [CALIB] defaults.
FDC_BASELINE_A = 1.70              # [CALIB] empty-drum free-spinning current
FDC_SLOPE_A_PER_KG = 0.032         # [CALIB] current rise per kg carried (1-g)

# ---- arm-lift geometry [CALIB] (the AR model is gravity work; height/efficiency are not published) ----
ARM_LIFT_HEIGHT_M = 0.5            # [CALIB] approx arm-raise lift height of the loaded drum (RASSOR-class)
ARM_LIFT_EFFICIENCY = 0.5          # [CALIB] arm drivetrain (electrical-in -> useful-lift) efficiency


def drum_mass_uncertainty_frac(mass_kg, *, include_outliers=False):
    """Realistic relative error of a 2020/2021 motor-current drum-mass estimate at a given fill, from the
    linear FDC hardware result (NTRS 20210022781): ~2.56% when the drum is more than half full
    (> HALF_FULL_KG), else the over-range mean percent error (7.40%, or 11.84% with the two outlier dig
    cycles). Lets the autonomy layer reason about IMPERFECT drum-fill knowledge, not exact mass."""
    if mass_kg > HALF_FULL_KG:
        return FDC_MPE_HALF_FULL
    return FDC_MPE_WITH_OUTLIERS if include_outliers else FDC_MPE_ALL


def arm_raise_lift_energy_j(mass_kg, g, *, lift_height_m=ARM_LIFT_HEIGHT_M, efficiency=ARM_LIFT_EFFICIENCY):
    """Energy to RAISE a loaded drum -- lift the excavated regolith against gravity: W = m * g * h / eff.
    First-principles gravity work, GRAVITY-AWARE (pass the body g from bodies.py / constants.g). The AR
    model's near-perfect linearity in mass (R^2 = 0.996) is physically this term. Small next to the
    excavation energy (dig ~4151 J/kg, ipex_specs), but real, and it scales with g so it matters across
    bodies. [CALIB] lift_height_m / efficiency are arm-geometry estimates, not published."""
    if mass_kg < 0:
        raise ValueError("mass_kg must be >= 0")
    return mass_kg * g * lift_height_m / efficiency


def freespin_drum_current_a(mass_kg, *, g=None, g_ref=EARTH_G,
                            baseline_a=FDC_BASELINE_A, slope_a_per_kg=FDC_SLOPE_A_PER_KG):
    """The Free-spinning Drum Current OBSERVABLE: the steady non-digging drum-motor current a drum carrying
    ``mass_kg`` of regolith draws at constant velocity. ``I = baseline + slope * mass`` (the FDC linear
    structure, NTRS 20210022781 Fig 8). This is the forward/synthesis direction: our conserved authority
    knows the true drum mass, this emits the current a real rover would read, and a calibrated
    :class:`LinearMassModel` inverts it back to a mass ESTIMATE (carry the band from
    :func:`drum_mass_uncertainty_frac`).

    ``g`` is opt-in: by default the measured 1-g slope is used as-is. Passing a body ``g`` rescales the
    mass-dependent current rise by ``g / g_ref`` -- a FLAGGED assumption that the rise is gravity-dominated
    (the paper is 1-g only, cannot separate gravity from shear/inertia, and calls for lunar recalibration)."""
    if mass_kg < 0:
        raise ValueError("mass_kg must be >= 0")
    slope = slope_a_per_kg * (g / g_ref if g is not None else 1.0)
    return baseline_a + slope * mass_kg


@dataclasses.dataclass
class LinearMassModel:
    """The linear drum-mass inference structure the AR / FDC / EDC models share:

        mass_kg = slope * feature + intercept

    where ``feature`` is the model's input -- integrated arm-motor power (AR), average free-spinning drum
    current (FDC), or aggregated excavation drum current (EDC). Coefficients are NOT published, so build
    one via :meth:`fit` on real paired telemetry (RASSOR data, or our own conserved-sim drum signal) --
    never fabricated."""
    slope: float
    intercept: float
    r2: float = float("nan")
    source: str = ""

    def predict(self, feature):
        """Infer drum mass [kg] from the model's feature (e.g. average free-spinning drum current)."""
        return self.slope * feature + self.intercept

    def invert(self, mass_kg):
        """Forward direction: the feature (e.g. motor current) a given drum mass would produce."""
        if self.slope == 0:
            raise ValueError("degenerate model (slope 0) cannot be inverted")
        return (mass_kg - self.intercept) / self.slope

    @classmethod
    def fit(cls, features, masses, *, source=""):
        """Least-squares calibrate (slope, intercept, R^2) from paired (feature, mass) telemetry."""
        features = list(features)
        masses = list(masses)
        n = len(features)
        if n < 2 or n != len(masses):
            raise ValueError("need >= 2 paired (feature, mass) samples of equal length")
        mx = sum(features) / n
        my = sum(masses) / n
        sxx = sum((x - mx) ** 2 for x in features)
        if sxx == 0:
            raise ValueError("features are constant; cannot fit a slope")
        sxy = sum((x - mx) * (y - my) for x, y in zip(features, masses))
        slope = sxy / sxx
        intercept = my - slope * mx
        ss_tot = sum((y - my) ** 2 for y in masses)
        ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(features, masses))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        return cls(slope=slope, intercept=intercept, r2=r2, source=source)


@dataclasses.dataclass
class OffloadDecision:
    """Result of the offload autonomy trigger: whether to haul-to-process, plus the bounds it reasoned on."""
    offload: bool
    inferred_kg: float
    lower_kg: float
    upper_kg: float
    uncertainty_frac: float
    capacity_kg: float
    reason: str


def should_offload(inferred_mass_kg, capacity_kg=REGOLITH_PER_CYCLE_KG, *,
                   conservative=True, include_outliers=False):
    """Autonomy trigger: should the rover stop digging and haul to the processing plant to offload?

    The drum fill is only KNOWN through the motor-current estimate, so this reasons on its realistic
    uncertainty (:func:`drum_mass_uncertainty_frac`) as a safety margin. ``conservative`` (default) fires
    when the UPPER confidence bound reaches capacity, so the drum will not overflow even if the true fill
    is at the high end of the estimate; otherwise it fires on the point estimate. Fill-knowledge is
    tightest exactly where the decision is made (> half full = 2.56% error), so the conservative margin
    near capacity is small. ``capacity_kg`` defaults to the IPEx/RASSOR ~30 kg/cycle drum capacity."""
    if capacity_kg <= 0:
        raise ValueError("capacity_kg must be > 0")
    unc = drum_mass_uncertainty_frac(inferred_mass_kg, include_outliers=include_outliers)
    lower = inferred_mass_kg * (1.0 - unc)
    upper = inferred_mass_kg * (1.0 + unc)
    bound = upper if conservative else inferred_mass_kg
    offload = bound >= capacity_kg
    mode = "upper-bound" if conservative else "point"
    reason = (f"{mode} {bound:.1f} kg {'>=' if offload else '<'} capacity {capacity_kg:.1f} kg "
              f"(inferred {inferred_mass_kg:.1f} +/- {unc * 100:.1f}%)")
    return OffloadDecision(offload=offload, inferred_kg=inferred_mass_kg, lower_kg=lower, upper_kg=upper,
                           uncertainty_frac=unc, capacity_kg=capacity_kg, reason=reason)


@dataclasses.dataclass
class DrumSensor:
    """Drum-fill sensing as one object: synthesize the motor-current observable from the conserved true
    drum mass, infer mass back (the calibrated FDC :class:`LinearMassModel`), and decide offload -- with
    OPTIONAL SEEDED noise that can be turned off at will. ``noise_frac == 0`` (default) is fully
    deterministic and faithful; ``noise_frac > 0`` adds a seeded Gaussian to the current reading whose
    std is ``noise_frac * (slope * capacity)`` (constant in absolute terms, so the RELATIVE mass error
    grows as the drum empties -- matching the paper's 7.40% over-range / 2.56%-when-full structure at
    ``noise_frac ~ FDC_MPE_HALF_FULL``). Build via :meth:`calibrated` to fit the inverse from a set of
    true masses, or pass a fitted ``model``."""
    model: LinearMassModel
    baseline_a: float = FDC_BASELINE_A
    slope_a_per_kg: float = FDC_SLOPE_A_PER_KG
    g: float = None
    capacity_kg: float = REGOLITH_PER_CYCLE_KG
    noise_frac: float = 0.0            # 0 = noise OFF (deterministic); >0 = seeded current noise
    seed: int = 0

    def __post_init__(self):
        self._rng = random.Random(self.seed)

    def reset_noise(self, seed=None):
        """Re-seed the noise stream (so a noisy sensor is reproducible across episodes/runs)."""
        self._rng = random.Random(self.seed if seed is None else seed)

    def current(self, true_mass_kg):
        """The drum-motor-current OBSERVABLE for a given true drum mass; adds seeded noise iff enabled."""
        i = freespin_drum_current_a(true_mass_kg, g=self.g, baseline_a=self.baseline_a,
                                    slope_a_per_kg=self.slope_a_per_kg)
        if self.noise_frac > 0.0:
            i += self._rng.gauss(0.0, self.noise_frac * self.slope_a_per_kg * self.capacity_kg)
        return i

    def infer(self, current_a):
        """Inferred drum mass [kg] from a current reading (the calibrated inverse model)."""
        return self.model.predict(current_a)

    def observe(self, true_mass_kg):
        """End-to-end sensing the rover gets: true mass -> (noisy) current -> inferred mass [kg]."""
        return self.infer(self.current(true_mass_kg))

    def offload(self, inferred_mass_kg, *, conservative=True, include_outliers=False):
        """Offload-autonomy decision on this sensor's capacity (see :func:`should_offload`)."""
        return should_offload(inferred_mass_kg, self.capacity_kg,
                              conservative=conservative, include_outliers=include_outliers)

    @classmethod
    def calibrated(cls, masses, *, g=None, noise_frac=0.0, seed=0, baseline_a=FDC_BASELINE_A,
                   slope_a_per_kg=FDC_SLOPE_A_PER_KG, capacity_kg=REGOLITH_PER_CYCLE_KG):
        """Fit the inverse FDC model from a set of true masses (currents synthesized by the forward
        model), then return a ready sensor. Mirrors calibrating against telemetry on real hardware."""
        currents = [freespin_drum_current_a(m, g=g, baseline_a=baseline_a, slope_a_per_kg=slope_a_per_kg)
                    for m in masses]
        model = LinearMassModel.fit(currents, masses, source="DrumSensor.calibrated")
        return cls(model=model, baseline_a=baseline_a, slope_a_per_kg=slope_a_per_kg, g=g,
                   capacity_kg=capacity_kg, noise_frac=noise_frac, seed=seed)
