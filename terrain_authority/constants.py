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
#: Janosi-Hanamoto shear; READ by slip.developed_thrust (the load-bearing slip ladder, Phases 2-3).
K_SHEAR = 0.018  # 1.8 cm

#: Slip-sinkage coefficients (theta_m = (c1 + c2*s)*theta_f). [UNKNOWN] spec §5.2
#: (c1~0.4, c2~0.3, "genuine unknowns"). Drives the runaway-entrapment failure mode
#: (spec §6 "Spirit-rover failure"); EXERCISED via slip_sinkage_multiplier on the physical=True
#: drive path (four_wheel_pass <- drive.closed_loop_drive / worksite.compact_over). Magnitudes [UNKNOWN].
SLIP_C1 = 0.4
SLIP_C2 = 0.3

# ---------------------------------------------------------------------------
# Rover mass / weight-on-wheels (for load-bearing sinkage).
#   Sourced from the IPEx TRL-5 design overview (papers/ascend24-ipex-trl-5-
#   design-overview.pdf; NTRS 20240008162, p.2): "The IPEx project is developing
#   a 30 kg-class excavator." Low mass is the DESIGN THESIS — counter-rotating
#   bucket drums cancel horizontal dig reaction, so IPEx does NOT rely on high
#   mass for tractive force (p.2). Consequence: very low weight-on-wheels, which
#   is precisely why slip-sinkage (not static bearing) is the dominant failure.
# ---------------------------------------------------------------------------

#: Rover dry mass [kg]. [CALIB] (ascend24 TRL-5, "30 kg-class").
ROVER_MASS_DRY_KG = 30.0

#: Max drum payload per excavation cycle [kg]. [CALIB] (ascend24: up to 30 kg/cycle,
#: 15 kg minimum success threshold). Laden weight-on-wheels rises with payload — a
#: path-dependent dynamic (excavating loads the drums -> more sinkage -> more slip).
DRUM_PAYLOAD_MAX_KG = 30.0

#: Number of ground wheels (IPEx 4-wheel layout; rover.py wheel_contact_points).
N_WHEELS = 4

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
STATE_SINTERED = 5                 # solar/microwave-fused hard surface (pads/roads/walls)
STATE_NAMES = ["VIRGIN", "TREAD", "EXCAVATED", "SPOIL", "COMPACTED_BERM", "SINTERED"]

# ---------------------------------------------------------------------------
# Sintering (the lunar concrete/asphalt analog): fuse loose regolith into a hard solid surface
# with solar/microwave/laser. Mass-conserving densification (porosity collapses -> denser, thinner).
# ---------------------------------------------------------------------------
#: Sintered/fused regolith bulk density [kg/m^3]. [SOURCED] measured sintered lunar-simulant density:
#: microwave-sintered ~2.23-2.34 g/cm^3 (Lin et al., J. Eur. Ceram. Soc. 2024, domestic-microwave study;
#: KLS-1 ~2.11), spark-plasma-sintered up to 2.90 g/cm^3 (Zhang et al., J. Eur. Ceram. Soc. 2020); 2300 is
#: the mid microwave value (between RHO_DEEP 1920 and grain 3100).
RHO_SINTERED = 2300.0
#: Energy to sinter regolith [J/kg]. This value is the THERMODYNAMIC FLOOR (sensible heat to sinter temp):
#: c_p ~0.8-1.0 J/g/K (lunar soil, Hemingway et al. 1973, Apollo 14/15/16; rises with T) x dT ~1075 K to a
#: ~1100 C sinter temp (Tsubaki et al., ACS Omega 2024, microwave ~900-1000 C; furnace 1050-1125 C) ~= 0.9-1.1
#: MJ/kg. [SOURCED-FLOOR] -- the MEASURED domestic-microwave PROCESS energy is 69-98 MJ/kg (Lin et al. 2024),
#: ~50-100x this floor (small-scale/coupling inefficiency); an engineered system lies between and is
#: method-dependent. The floor is the defensible lower bound; do NOT read it as a full process budget.
SINTER_ENERGY_J_PER_KG = 920_000.0
#: Measured microwave PROCESS energy [J/kg] (Lin et al. 2024) -- documented for honest planning; the floor
#: above is the ideal minimum, this is a real (inefficient, small-scale) upper reference.
SINTER_PROCESS_ENERGY_J_PER_KG_MEASURED = 69_000_000.0
#: Sinter feasibility gate (the SINGLE source of truth, read by WorkSite.sinter and mission_planner).
#: Sinter is a real, conserved, tested authority primitive (column_state.sinter), and RHO_SINTERED +
#: SINTER_ENERGY_J_PER_KG are now LITERATURE-SOURCED (above). It stays GATED OFF for the IPEx baseline for
#: two SOURCED physical reasons, not for missing data: (1) IPEx is a RASSOR-lineage drum EXCAVATOR with no
#: sintering tool (no microwave/solar/laser head on the modeled platform); (2) sintering is energetically
#: incompatible with the IPEx power system -- even the thermodynamic floor (0.92 MJ/kg) is ~0.2x the 4.79 MJ
#: pack per kg, and the MEASURED process energy (69-98 MJ/kg) is ~14-20x the whole pack PER KILOGRAM. Flip to
#: True only for a deliberately sinter-EQUIPPED, externally-powered variant (not the IPEx baseline).
SINTER_ENABLED = False

# ===========================================================================
# DEM-TERRAIN THRUST — sourced procgen parameters (Lane B, ADDITIVE block).
# ===========================================================================
# For the real-DEM 10 km terrain work (docs/dem_terrain_contract.md
# §6, docs/lunar_dem_10km_eval.md §6 + papers/CITATIONS.md "Lunar DEM / terrain-
# statistics references"). NOTHING above this line is modified — every name here is
# NEW, so existing scenes/tests are byte-for-byte unaffected (HARD BACKWARD-COMPAT).
#
# HONESTY-TAG legend (extends the file header; the binding "every parameter sourced,
# never eyeballed" rule means the tag carries onto the PARAMETER, not just prose):
#     [FIXED]               well-constrained physical constant / model with a real
#                           primary citation, used in its valid regime.
#     [CALIB]               a calibration CHOICE — a fit (often Earth/Apollo/mare era)
#                           or a transcribed-from-secondary value; not ground truth.
#     [prior-applied-to-pole] a global / equatorial / mare value used AT THE POLE
#                           because no polar in-situ measurement exists.
#     [ASSUMPTION]          an engineering bound stated explicitly so it is auditable.
#     [UNKNOWN]             a genuine wide-envelope unknown with no numeric source.
# A parameter may NOT be called "sourced" without one of these tags + a citation.

# ---------------------------------------------------------------------------
# Crater PRODUCTION — Neukum production-function coefficient vector. [CALIB]
# ---------------------------------------------------------------------------
#: Neukum/Ivanov/Hartmann (2001) crater-production polynomial coefficients a0..a11.
#: Model: log10 N_cum(>=D) = sum_{j=0..11} a_j * (log10 D_km)^j  [craters km^-2 Gyr^-1
#: at the 1 Gyr reference], with D in KILOMETRES. Valid for ~10 m .. ~1000 km;
#: DO NOT extrapolate below ~10 m (the sub-DEM band is governed by the equilibrium
#: cap eq_sfd(D) below, NOT by extrapolating this polynomial).
#:
#: [CALIB] — the PRIMARY table (Ivanov/Neukum/Hartmann 2001, Space Sci. Rev. 96:55) is
#: NOT directly verified here (it is absent from papers/): this vector is TRANSCRIBED from
#: the MintonGroup/cratermaker encoding of the Neukum PF and cross-checked numerically.
#: The three numbers that have caused confusion in earlier notes are DISTINCT quantities,
#: NOT one "normalization":
#:   * a0 = -3.0876  ->  10**a0 = 8.173e-4 craters km^-2 Gyr^-1 at D=1 km. This IS the
#:     canonical Neukum-2001 production-function constant term.
#:   * 8.38e-4 is the LINEAR-in-time coefficient of the CHRONOLOGY N(1,t) (see
#:     neukum_chronology() below) — a DIFFERENT function, not a0's normalization.
#:   * 8.25e-4 is the a10 polynomial SHAPE coefficient (it multiplies (log10 D_km)^10),
#:     not a normalization at all.
#: There IS one small, real INTERNAL mismatch: the production poly anchors the 1-km/1-Gyr
#: level at 8.173e-4 while neukum_chronology(1) gives 8.380e-4 (~2.47%). It is within model
#: uncertainty and does NOT move the sub-10 m band the sim actually uses — that band is
#: governed by the eq_sfd equilibrium cap below, not by extrapolating this polynomial — so
#: it is left as-is and documented rather than force-reconciled.
#: LICENSE: cratermaker is GPL-3.0 (github.com/MintonGroup/cratermaker).
#: GPL-3.0 is copyleft, so NO cratermaker CODE may be copied into this CC0 repo; only the
#: numeric coefficients are reused — uncopyrightable scientific facts, cited to Neukum/
#: Ivanov/Hartmann 2001 by author/year. No cratermaker code is vendored or copied.
NEUKUM_SFD_COEF = (
    -3.0876,    # a0  (10**a0 = 8.173e-4 craters km^-2 Gyr^-1 at D=1 km = production-poly
                #      constant term; this is NOT the chronology's 8.38e-4 — see note above)
    -3.557528,  # a1
    0.781027,   # a2
    1.021521,   # a3
    -0.156012,  # a4
    -0.444058,  # a5
    0.019977,   # a6
    0.086850,   # a7
    -0.005874,  # a8
    -0.006809,  # a9
    8.25e-4,    # a10  (a polynomial SHAPE coefficient on (log10 D_km)^10 — NOT a normalization)
    5.54e-5,    # a11
)

#: Surface age committed for the production function [Gyr]. [CALIB] — a model age
#: choice (the 10 km Haworth tile is ancient highland terrain; 3.5 Gyr is the eval's
#: committed value, docs/lunar_dem_10km_eval.md §6). Production scales ~linearly with
#: this for ages < ~3 Gyr via the Neukum chronology; see neukum_chronology().
NEUKUM_SURFACE_AGE_GYR = 3.5


def neukum_chronology(age_gyr: float) -> float:
    """Neukum (2001) lunar cratering CHRONOLOGY: cumulative-density scale factor vs age.

    Returns the multiplier on the 1-Gyr-reference production density for a surface of
    the given model age [Gyr]:

        N(1 km, t) = 5.44e-14 * (exp(6.93 * t) - 1) + 8.38e-4 * t

    (Neukum/Ivanov/Hartmann 2001, Space Sci. Rev. 96:55, eq. for N(1) vs t). The
    production polynomial NEUKUM_SFD_COEF gives the SHAPE (relative SFD); this anchors
    its absolute level at age t. [CALIB] — the chronology constants are the published
    Neukum values; transcribed, primary table unverified (see NEUKUM_SFD_COEF note).
    """
    return 5.44e-14 * (np.exp(6.93 * age_gyr) - 1.0) + 8.38e-4 * age_gyr


def neukum_production_cumulative(diameter_m: float | np.ndarray,
                                 age_gyr: float = NEUKUM_SURFACE_AGE_GYR,
                                 ) -> np.ndarray:
    """Cumulative crater production N_cum(>= D) per m^2 for a surface of age `age_gyr`.

    Evaluates the NEUKUM_SFD_COEF polynomial (giving the relative SFD shape, anchored
    at 1 Gyr) and rescales the absolute level to `age_gyr` via neukum_chronology().
    Input D in METRES; output is craters per SQUARE METRE (km^-2 -> m^-2 = *1e-6).

    [CALIB] — see NEUKUM_SFD_COEF. Valid ~10 m..1000 km; the caller (procgen_csfd) is
    responsible for NOT extrapolating below ~10 m and for capping at eq_sfd(D).
    """
    d_km = np.asarray(diameter_m, dtype=np.float64) / 1000.0
    x = np.log10(d_km)
    # Horner evaluation of sum a_j x^j.
    acc = np.zeros_like(x)
    for a in reversed(NEUKUM_SFD_COEF):
        acc = acc * x + a
    n_per_km2_1gyr = 10.0 ** acc
    # Rescale the 1-Gyr reference to the committed age, then km^-2 -> m^-2.
    scale = neukum_chronology(age_gyr) / neukum_chronology(1.0)
    return n_per_km2_1gyr * scale * 1e-6


# ---------------------------------------------------------------------------
# Small-crater EQUILIBRIUM cap. [CALIB]
# ---------------------------------------------------------------------------
#: Xiao & Werner (2015, JGR 120, doi:10.1002/2015JE004860) report small craters reach
#: an empirical equilibrium ("steady state") at ~1-10% of GEOMETRIC SATURATION. We take
#: the CENTRAL value of that band (~5.5% of saturation) as the cap coefficient below.
#: The geometric-saturation cumulative density scales as ~D^-2; equilibrium is a
#: fraction of it -> n_eq(>=D) = EQ_SFD_COEF * D^-2  [craters per m^2], D in METRES.
#:
#: [CALIB] — band central value is a calibration CHOICE. Minton et al. 2019 (Icarus,
#: arXiv:1902.07746) fit an Apollo-15 MARE equilibrium n_eq(>=D) ~ 0.0336 * D^-2 (D in m)
#: which, applied to this HIGHLAND/polar surface, is a LOWER BOUND (EQ_SFD_COEF_MARE_LB).
EQ_SFD_COEF = 0.084            # central ~5.5%-of-saturation highland/polar cap [m^0]
EQ_SFD_COEF_MARE_LB = 0.0336   # Minton 2019 Apollo-15 mare fit -> lower bound on highland


def eq_sfd(diameter_m: float | np.ndarray) -> np.ndarray:
    """Equilibrium (steady-state) cumulative crater density n_eq(>= D) per m^2.

        n_eq(>= D) = EQ_SFD_COEF * D^-2        (D in metres)

    The cap in the sub-DEM band: actual emplaced density is min(production, eq_sfd)
    because a surface in equilibrium has erased as many small craters as it accrues
    (Xiao & Werner 2015). [CALIB] — EQ_SFD_COEF is the central value of the X&W 1-10%
    band; EQ_SFD_COEF_MARE_LB (Minton 2019 mare fit) is the lower bound on a highland.
    """
    d = np.asarray(diameter_m, dtype=np.float64)
    return EQ_SFD_COEF * d ** (-2.0)


# ---------------------------------------------------------------------------
# fbm spectral fidelity — Hurst -> amplitude gain. [CALIB]
# ---------------------------------------------------------------------------
#: Self-affine surfaces have an fbm amplitude gain per octave of lacunarity**(-H), where
#: H is the Hurst exponent (PSD slope beta = 2H+1 in 2-D). The repo fbm default gain=0.5
#: at lacunarity=2 implies H=1.0 (too smooth/correlated). South-pole highland-like
#: terrain in the DEM-resolved band (>= ~30 m) measures H ~ 0.95 (Rosenburg et al. 2011,
#: JGR doi:10.1029/2010JE003716; Barker et al. 2025, PSJ doi:10.3847/PSJ/adbc9d), NOT the
#: maria 0.76. [CALIB] — a calibration choice anchored to those measurements.
HURST_RESOLVED_BAND = 0.95     # >= ~30 m, south-pole highland-like [CALIB]

#: cm / rover-band Hurst. [prior-applied-to-pole] — H ~ 0.5-0.7 at mm-cm scale from
#: Helfenstein & Shepard 1999 (Icarus 141), which is APOLLO CLOSE-UP / EQUATORIAL; no
#: polar in-situ cm-scale measurement exists. H is SCALE-DEPENDENT, so a single fixed
#: gain is wrong at one end -> H must ramp between HURST_RESOLVED_BAND and this.
HURST_CM_BAND = 0.6            # 0.5-0.7 envelope, central [prior-applied-to-pole]
HURST_CM_BAND_MIN = 0.5
HURST_CM_BAND_MAX = 0.7

#: Terminal RMS slope at the 2 cm sim cell [rad]. [prior-applied-to-pole] — ~20 deg
#: (envelope 15-35 deg) at mm-cm scale, Helfenstein & Shepard 1999 (equatorial) +
#: Bandfield et al. 2015 (Diviner, GLOBAL). Used to bound the synthesized roughness at
#: the finest scale; no polar in-situ cm-scale slope measurement exists.
TERMINAL_RMS_SLOPE_RAD = np.deg2rad(20.0)
TERMINAL_RMS_SLOPE_MIN_RAD = np.deg2rad(15.0)
TERMINAL_RMS_SLOPE_MAX_RAD = np.deg2rad(35.0)


def hurst_to_fbm_gain(H: float, lacunarity: float = 2.0) -> float:
    """fbm per-octave amplitude gain for a target Hurst exponent: gain = lacunarity**(-H).

    H=1.0 -> gain=0.5 at lacunarity 2 (the repo default, very smooth); H=0.95 -> ~0.518;
    H=0.6 -> ~0.660 (rougher, more high-frequency energy).

    IMPORTANT CAVEAT (docs/lunar_dem_10km_eval.md §6 "fbm spectral fidelity"): fixing the
    gain is NECESSARY BUT NOT SUFFICIENT. The repo fbm's min-max-to-[0,1] renorm
    (procgen.py:74-77) is a realization-dependent NONLINEAR rescale that DESTROYS the PSD
    slope this gain is meant to set. Correct spectral fidelity requires BOTH this gain AND
    a variance/deviogram-anchored normalization (scale to a target RMS from Product-90
    LDRM_RMSD) instead of the [0,1] renorm. Gain alone, with the min-max renorm still in
    place, does not deliver the intended Hurst slope. See procgen.fbm(normalize="variance").
    """
    return float(lacunarity ** (-H))


# ---------------------------------------------------------------------------
# Crater SYNTHESIS cutoff (de-confliction with what the DEM already resolves). [CALIB]
# ---------------------------------------------------------------------------
#: Only synthesize craters BELOW the DEM's effective resolution; D_min = m * eff_px with
#: m a Nyquist-style multiplier (~2-3). The DEM effective resolution per pixel is the
#: SOURCED input (PGDA Product 90 LDEM_EFFRES per-pixel layer, Barker et al. 2023); the
#: 2-3x multiplier is an engineering heuristic. [CALIB].
LDEM_EFFRES_NYQUIST_MULT = 2.5     # m in D_min = m * eff_px [CALIB] (2-3 band, central)


# ---------------------------------------------------------------------------
# Crater depth/diameter — size-dependent (Stopar 2017). [FIXED]>400m / [CALIB] below
# ---------------------------------------------------------------------------
#: Fresh simple-crater depth/diameter is NOT a single value across all sizes:
#:   d/D ~ 0.196 for D >= 400 m  ([FIXED]; Pike 1977, the existing repo value 0.2 is this
#:                                regime, valid >400 m; Stoffler 2006 RiMG 60),
#:   d/D drops to a 0.11-0.17 band BELOW 400 m, ~0.13 at 20-50 m  ([CALIB]; Stopar 2017,
#:                                Icarus) — the small craters procgen actually adds.
#: The existing CRATER_DEPTH_DIAMETER_RATIO=0.2 (constants.py:178) is the >400 m regime
#: and is LEFT UNCHANGED; this is the additive size-dependent helper for the sub-DEM band.
CRATER_DD_LARGE = 0.196            # D >= 400 m [FIXED] (Pike 1977 / Stoffler 2006)
CRATER_DD_SMALL_MIN = 0.11         # D < 400 m band lower [CALIB] (Stopar 2017)
CRATER_DD_SMALL_MAX = 0.17         # D < 400 m band upper [CALIB] (Stopar 2017)
CRATER_DD_SMALL_NOMINAL = 0.13     # ~0.13 at 20-50 m [CALIB] (Stopar 2017, central)
CRATER_DD_TRANSITION_M = 400.0     # the >400 m / <400 m morphometric break [FIXED]


def crater_depth_ratio(diameter_m: float) -> float:
    """Size-dependent fresh-crater depth/diameter ratio d/D.

    D >= 400 m  -> CRATER_DD_LARGE (0.196) [FIXED] (Pike 1977 / Stoffler 2006).
    D <  400 m  -> CRATER_DD_SMALL_NOMINAL (~0.13) [CALIB] (Stopar 2017), within the
                   0.11-0.17 band. (A flat 0.2 is too DEEP for the sub-400 m craters the
                   sub-DEM generator adds.) Constant within each regime here — a single
                   sourced break, not a fitted curve.
    """
    return CRATER_DD_LARGE if diameter_m >= CRATER_DD_TRANSITION_M else CRATER_DD_SMALL_NOMINAL


# ---------------------------------------------------------------------------
# Crater EJECTA — McGetchin radial decay + corrected continuous extent. [FIXED]
# ---------------------------------------------------------------------------
#: Ejecta radial thickness ~ (r/R)^-3 (McGetchin et al. 1973, EPSL 20; Settle & Head
#: 1977; Melosh 1989). The existing carve_crater ejecta uses a quadratic ramp keyed to
#: the outer edge (thickest at the rim, thinning outward — the CORRECT direction, not
#: "backwards"); the sourced refinement is the empirical (r/R)^-3 power law. [FIXED].
CRATER_EJECTA_DECAY_EXP = -3.0     # radial thickness power-law exponent [FIXED]

#: Continuous-ejecta radial extent as a multiple of crater RADIUS. Observed 2.3-2.7 R
#: (McGetchin 1973 / Settle & Head 1977 / Melosh 1989). The existing
#: CRATER_EJECTA_EXTENT_RADII=2.0 (constants.py:186) sits at the LOW edge of this band
#: and is LEFT UNCHANGED; this additive central value is the sourced correction. [FIXED].
CRATER_EJECTA_EXTENT_RADII_MIN = 2.3
CRATER_EJECTA_EXTENT_RADII_MAX = 2.7
CRATER_EJECTA_EXTENT_RADII_SOURCED = 2.5   # central of 2.3-2.7 R [FIXED]

#: Fresh-crater rim height as a fraction of DIAMETER. ~0.036 D -> rim/depth ~ 0.18
#: (Stoffler 2006, RiMG 60). The existing CRATER_RIM_HEIGHT_FRAC=0.2 is rim-AS-FRACTION-
#: OF-DEPTH (a different ratio) and is LEFT UNCHANGED; this is the sourced rim/diameter.
CRATER_RIM_HEIGHT_DIAM_FRAC = 0.036   # h_rim/D [FIXED] (Stoffler 2006)


# ---------------------------------------------------------------------------
# Spatial boulder abundance k (Golombek SFD total fractional area). [CALIB]
# ---------------------------------------------------------------------------
#: The Golombek SFD model golombek_q(k)/sample_boulders is correct AS-IS (Golombek &
#: Rapp 1997, doi:10.1029/96JE03319; Golombek 2003). What is sourced/refined here is
#: making the total-fractional-area k SPATIAL: a sparse polar BACKGROUND (Bandfield 2011
#: Diviner rock abundance <1% over most terrain) ramping UP only in fresh ejecta / rims.
#: [CALIB] — the spatial k field is a calibration choice anchored to those abundances;
#: the per-region areal densities (Bernhardt/Boazman 2022, Watkins 2019, Bickel & Kring
#: 2020) are SECONDARY-SOURCED (primary PDFs unverified) and cross-checked vs USGS LROC
#: NAC Boulder DB v1.
BOULDER_K_BACKGROUND = 0.005       # 0.001-0.01 sparse polar background [CALIB] (~<1%)
BOULDER_K_BACKGROUND_MIN = 0.001
BOULDER_K_BACKGROUND_MAX = 0.01
BOULDER_K_EJECTA = 0.20            # 0.05-0.40 fresh-ejecta/rim ramp [CALIB]
BOULDER_K_EJECTA_MIN = 0.05
BOULDER_K_EJECTA_MAX = 0.40

#: Boulder buried-fraction distribution. [UNKNOWN] — kept at the repo's U(0.1, 0.7)
#: (procgen.py:246). Ruesch & Woehler 2021 (arXiv:2109.00052) give only a QUALITATIVE
#: age-monotonic direction (older boulders more buried); NO numeric distribution exists.
#: This stays a genuine wide-envelope unknown, tagged on the parameter.
BOULDER_BURIED_FRAC_MIN = 0.1      # [UNKNOWN] (repo value; no numeric source)
BOULDER_BURIED_FRAC_MAX = 0.7      # [UNKNOWN]


# ---------------------------------------------------------------------------
# POLAR regolith density profile (ChaSTE, Chandrayaan-3). [CALIB]
# ---------------------------------------------------------------------------
#: ChaSTE in-situ two-layer polar density profile from Chandrayaan-3 (Durga Prasad et
#: al. 2026, ApJ doi:10.3847/1538-4357/ae5228; Mathew et al. 2025, Sci. Rep.
#: doi:10.1038/s41598-025-91866-4), measured at 69.4 deg S (~20 deg FROM the pole):
#:   ~750 kg/m^3   over 0-3 cm   (very loose top fines),
#:   ~1300 kg/m^3  over 3-6.5 cm,
#:   ~1940 kg/m^3  bulk average over 0-10 cm.
#: [CALIB] — a calibration choice; sub-polar (not AT the pole), and only ~10 cm deep.
#:
#: These are NEW polar-tagged SIBLINGS. They DO NOT overwrite the existing equatorial/
#: Apollo profile RHO_SURFACE=1300 / RHO_DEEP=1920 / Z_T=0.12. CRITICAL CAVEAT: ChaSTE's
#: ~1940 kg/m^3 @ 0-10 cm does NOT "confirm" the repo RHO_DEEP=1920 @ ~100 cm — they are
#: DIFFERENT DEPTHS (10 cm vs ~1 m). Different measurements; do not conflate.
RHO_SURFACE_POLAR = 750.0          # 0-3 cm [CALIB] (ChaSTE, Durga Prasad 2026)
RHO_MID_POLAR = 1300.0             # 3-6.5 cm [CALIB] (ChaSTE)
RHO_BULK_POLAR_10CM = 1940.0       # 0-10 cm bulk avg [CALIB] (ChaSTE); != RHO_DEEP@~1m
Z_POLAR_TOP_M = 0.03               # 0-3 cm top-layer base [CALIB] (ChaSTE)
Z_POLAR_MID_M = 0.065              # 3-6.5 cm mid-layer base [CALIB] (ChaSTE)


def polar_density_profile(depth_m: float | np.ndarray) -> np.ndarray:
    """ChaSTE two-layer polar bulk density [kg/m^3] vs depth below surface [m].

    Piecewise: RHO_SURFACE_POLAR over [0, 3cm), RHO_MID_POLAR over [3, 6.5cm), and
    RHO_BULK_POLAR_10CM at/below 6.5 cm (the 0-10 cm bulk average stands in for the
    deeper-than-measured column — ChaSTE only reached ~10 cm). [CALIB], sub-polar
    (69.4 deg S), do NOT conflate with the repo equatorial profile (see constants note).
    """
    z = np.asarray(depth_m, dtype=np.float64)
    return np.where(z < Z_POLAR_TOP_M, RHO_SURFACE_POLAR,
                    np.where(z < Z_POLAR_MID_M, RHO_MID_POLAR, RHO_BULK_POLAR_10CM))


# ---------------------------------------------------------------------------
# Regolith column thickness (the m-scale column, distinct from Z_T). [ASSUMPTION]
# ---------------------------------------------------------------------------
#: Highland regolith column ~10-15 m thick (Bart/Fa crater-morphology methods; a site-
#: specific bound is read from PGDA Product 90). This is the M-SCALE regolith column,
#: explicitly DISTINCT from Z_T=0.12 m (the CM-SCALE loose-over-dense transition the
#: bearing/sinkage model uses). [ASSUMPTION] — an engineering bound stated for audit; the
#: DEM bridge (dem_to_base) injects the cm-scale loose mantle ~Z_T, the datum carries the
#: rest of this column.
REGOLITH_THICKNESS_M = 12.0        # 10-15 m highland column, central [ASSUMPTION]
REGOLITH_THICKNESS_MIN_M = 10.0
REGOLITH_THICKNESS_MAX_M = 15.0
