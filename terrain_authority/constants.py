"""Physical constants and calibration parameters for the Tier-2 lunar regolith surrogate.

ALL VALUES ARE SI. The spec (§5) quotes densities in g/cm^3 and several moduli in
kN units; everything here is converted to base SI (kg/m^3, Pa, m, rad) so that the
on-disk contract (INTERFACE.md §4 "SI everywhere to kill unit ambiguity") holds with
zero conversion at the consumer.

Sources are cited inline by spec table (§5.1 fixed / §5.2 calibration) AND, where a
paper anchors the number, by papers/ filename + the specific claim. We deliberately do
NOT open the PDFs; citations name the claim a reviewer can check.

CALIBRATION STATUS legend (used in comments):
    [FIXED]   well-constrained physical constant.
    [CALIB]   calibration target — Earth/Apollo-era fit, to be re-fit against a
              Chrono::GPU DEM oracle (spec §10) and corrected 1g -> 1/6 g.
    [UNKNOWN] genuine wide-envelope unknown (esp. polar/PSR), flagged for the reviewer.
"""

import numpy as np

# ---------------------------------------------------------------------------
# §5.1 Fixed global constants
# ---------------------------------------------------------------------------

#: Surface gravity [m/s^2]. [FIXED] spec §5.1. 1/6 Earth; drives all bearing/sinkage.
g = 1.62

#: Grain specific gravity [dimensionless]. [FIXED] spec §5.1 (G_s 3.0-3.32, ~3.1).
#: Sets the solid (zero-void) grain density below.
G_s = 3.1

#: Density of liquid water [kg/m^3] — reference for specific gravity -> density.
RHO_WATER = 1000.0

#: Solid grain density [kg/m^3] = G_s * rho_water. The absolute ceiling on bulk density
#: (bulk density at zero void fraction). geosciences-15-00207-v3.pdf / FULLTEXT01.pdf:
#: lunar grains are anorthositic/agglutinate, ~3.1 g/cm^3.
RHO_GRAIN = G_s * RHO_WATER  # 3100 kg/m^3

#: Solar irradiance [W/m^2]. [FIXED] spec §5.1 (~1361). Thermal/optics only; unused in
#: the mass-balance authority but kept for downstream sensor-model scenes.
S_solar = 1361.0

#: Polar sun-elevation band [deg]. [FIXED] spec §5.1 (0-7 deg). Used by the hillshade
#: preview (grazing sun -> extreme shadows, the IPEx perception challenge, spec §8).
SUN_ELEVATION_DEG_POLAR = 7.0

# ---------------------------------------------------------------------------
# §5.2 Bulk density profile (loose-over-dense). g/cm^3 -> kg/m^3.
# ---------------------------------------------------------------------------

#: Surface (loose top-layer) bulk density [kg/m^3]. [CALIB] spec §5.2 (1.1-1.5, ~1.30
#: g/cm^3 -> 1300). geosciences-15-00207-v3.pdf / FULLTEXT01.pdf: loose fluffy fines at
#: the immediate surface.
RHO_SURFACE = 1300.0  # 1.30 g/cm^3

#: Deep (compacted) bulk density [kg/m^3]. [CALIB] spec §5.2 (1.8-2.0, ~1.92 g/cm^3 ->
#: 1920) below ~100 cm. FULLTEXT01.pdf: density rises with depth as voids close.
RHO_DEEP = 1920.0  # 1.92 g/cm^3

#: Density transition depth [m]. [CALIB] spec §5.2 (z_t 10-15 cm -> 0.12 m). Sets the
#: self-limiting (fast/shallow) static sinkage scale; loose-over-dense is "the hinge for
#: the three terrain states and multi-pass paving" (spec §9).
Z_T = 0.12  # 12 cm

# ---------------------------------------------------------------------------
# §5.2 Strength parameters
# ---------------------------------------------------------------------------

#: Cohesion [Pa]. [CALIB] spec §5.2 (c 0.1-1.0 kPa, ~0.17 kPa -> 170 Pa). Interlocking-
#: driven ("like Velcro", spec §9); spec notes c DECREASES in low-g — NOT applied here.
#: Earth/Apollo-era value; see lyasko2010.pdf (reduced-gravity Bekker corrections).
COHESION = 170.0  # 0.17 kPa

#: Internal friction angle [rad]. [CALIB] spec §5.2 (phi 30-50 deg, ->55 at depth).
#: ~g-independent (spec §5.2). 37 deg is a mid-range loose-surface value.
PHI = np.deg2rad(37.0)

# ---------------------------------------------------------------------------
# §5.2 Bekker / Wong-Reece pressure-sinkage moduli.
#   Bekker: p = (k_c / b + k_phi) * z^n   with k_c [kN/m^(n+1)], k_phi [kN/m^(n+2)].
#   At n=1.0 (our nominal) units reduce to: k_c [kN/m^2 = kPa], k_phi [kN/m^3].
#   Converted to SI Pa-based (x1000).
#
#   *** PAPERED OVER: these are classic Apollo-era (Mitchell/Costes) Earth-fit values
#   (spec §5.2 "calibration starting points, not ground truth"). No 1g -> 1/6 g
#   correction is applied. lyasko2010.pdf shows lowering gravity decreases n, k_phi and
#   c while k_c and phi change little, and sinkage INCREASES under the same load — so
#   these UNDER-predict lunar sinkage. The static-sinkage helper below is a geometric
#   stand-in only; the headline authority is mass conservation, not force accuracy
#   (spec §9 "forces engineered small ... must be geometry- and state-accurate"). ***
# ---------------------------------------------------------------------------

#: Sinkage exponent n [dimensionless]. [CALIB] spec §5.2 (0.8-1.0, ~1.0). Rises with
#: density, DROPS in low-g (lyasko2010.pdf) — low-g drop NOT applied.
N_SINKAGE = 1.0

#: Bekker cohesive modulus k_c [Pa/m^(n-1)]. [CALIB] spec §5.2 (~1.4 kN/m^(n+1)).
#: ~g-independent (lyasko2010.pdf). 1.4 kN -> 1400 (SI, at n=1).
K_C = 1400.0

#: Bekker frictional modulus k_phi [Pa/m^n]. [CALIB] spec §5.2 (~800-820 kN/m^(n+2),
#: wide uncertainty). DROPS in low-g (lyasko2010.pdf) — drop NOT applied. 820 kN ->
#: 820000 (SI, at n=1).
K_PHI = 820_000.0

#: Shear deformation modulus K [m]. [CALIB] spec §5.2 (1.0-1.8 cm, ~1.8 -> 0.018 m).
#: Janosi-Hanamoto shear; unused by the geometry-only rover pass but kept for fidelity.
K_SHEAR = 0.018  # 1.8 cm

#: Slip-sinkage coefficients (theta_m = (c1 + c2*s)*theta_f). [UNKNOWN] spec §5.2
#: (c1~0.4, c2~0.3, "genuine unknowns"). Drives the runaway-entrapment failure mode
#: (spec §6 "Spirit-rover failure"); not exercised by the single-pass geometry rover.
SLIP_C1 = 0.4
SLIP_C2 = 0.3

# ---------------------------------------------------------------------------
# §5.2 / §7 Granular flow: repose angle and bulking.
# ---------------------------------------------------------------------------

#: Nominal angle of repose / critical angle [rad]. [UNKNOWN] spec §5.2 (theta_r 30-47
#: deg, "wide envelope"; finer -> steeper; highland steeper than mare; STEEPER in low-g
#: via relative cohesion — reduced-g effect "genuinely unsettled", spec §7). 35 deg
#: nominal; the sandpile CA accepts a per-call override across the envelope.
THETA_R = np.deg2rad(35.0)
THETA_R_MIN = np.deg2rad(30.0)
THETA_R_MAX = np.deg2rad(47.0)

#: Bulking / swell factor SF [dimensionless]. [CALIB] spec §5.2 (1.1-1.3). In-situ ->
#: loose density drop on excavation; "closes the cut/fill loop" (spec §7 bulking). We
#: define spoil (dumped, loose) density = RHO_DEEP-cut / SF -> looser, taller per kg.
SWELL_FACTOR = 1.2

#: Loose spoil density [kg/m^3] — what freshly dumped material settles to (SPOIL state).
#: Derived so a dense in-situ cut bulks to a lower density when redeposited (spec §7
#: "a bucket deposits more volume than the hole it left"). Kept at/near RHO_SURFACE.
RHO_SPOIL = RHO_SURFACE / 1.0  # 1300; loose like the surface layer (spec §7 ~1.3 g/cm^3)

# ---------------------------------------------------------------------------
# §5.2 Grain size & volatiles
# ---------------------------------------------------------------------------

#: Median grain size D50 [m]. [CALIB] spec §5.2 (40-130 um, ~70 -> 7e-5 m). Fine silty
#: sand, poorly sorted, angular (spec §9). Optics/dust scale; not in mass balance.
D50 = 70e-6  # 70 microns

#: Max ice / volatile mass fraction [dimensionless]. [UNKNOWN] spec §5.2 (0 dry - 5.6 +-
#: 2.9 % PSR, LCROSS-derived). geosciences-15-00207-v3.pdf / FULLTEXT01.pdf (volatiles).
#: Gates GRANULAR vs CEMENTED regime; kept OUT of the conservation invariant (spec §8).
W_ICE_MAX = 0.056  # 5.6 %

#: PSR cold-trap temperature threshold [K]. [FIXED-ish] spec §5.1/§5.2 (<110 K).
T_PSR_K = 110.0

# ---------------------------------------------------------------------------
# §5.2 Rock size-frequency (Golombek). Cumulative FRACTIONAL AREA model.
# ---------------------------------------------------------------------------
#: Golombek exponent law q(k) = 1.79 + 0.152/k  [1/m], with k the total fractional area
#: covered by rocks. F_k(D) = k * exp(-q(k) * D). rock-size-freq_abstract.txt (Golombek
#: et al. 2003, LPSC XXXIV): "Fk(D) = k exp[-q(k) D] ... q(k) = 1.79 + 0.152/k". Family
#: of non-crossing curves, total rock abundance 5-40%.


def golombek_q(k: float) -> float:
    """Golombek SFD exponent q(k) = 1.79 + 0.152/k [1/m].

    rock-size-freq_abstract.txt: governs how abruptly area-covered falls with diameter.
    """
    return 1.79 + 0.152 / k


# ---------------------------------------------------------------------------
# Crater geometry (Pike-class fresh simple crater).
# ---------------------------------------------------------------------------
#: Depth/diameter ratio for a fresh simple (Pike-class) lunar crater [dimensionless].
#: ~0.2 (spec task / Pike 1977 fresh-simple morphometry). Degrades toward shallower with
#: age — NOT modelled (single fresh profile only).
CRATER_DEPTH_DIAMETER_RATIO = 0.2

#: Rim height as a fraction of crater depth [dimensionless]. Pike-class fresh rim ~0.04
#: of diameter ~ 0.2 of depth. Geometric approximation.
CRATER_RIM_HEIGHT_FRAC = 0.2

#: Ejecta blanket radial extent as a multiple of crater RADIUS [dimensionless].
#: Continuous ejecta ~1 crater radius beyond rim (~2 radii from center). Approximation.
CRATER_EJECTA_EXTENT_RADII = 2.0

# ---------------------------------------------------------------------------
# State-label enum (INTERFACE.md §4, spec §6). Mirrored in column_state.StateLabel.
# ---------------------------------------------------------------------------
STATE_VIRGIN = 0
STATE_TREAD = 1
STATE_EXCAVATED = 2
STATE_SPOIL = 3
STATE_COMPACTED_BERM = 4
STATE_NAMES = ["VIRGIN", "TREAD", "EXCAVATED", "SPOIL", "COMPACTED_BERM"]
