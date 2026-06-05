"""Per-column terrain state — the Tier-2 data model (spec §5.3, §6; INTERFACE.md §4).

The grid is a "stacked heightfield" (spec §4): a 2.5D surface with a depth-density
profile beneath each column. We store the SI fields the contract freezes, and a tiny
amount of bookkeeping (drum inventory) so the cut/dump/relax cycle conserves mass
exactly (spec §10 invariant 1).

THE conserved invariant is ``mass_areal`` [kg/m^2]. Height is DERIVED, never authored:

    height = datum + mass_areal / density        (INTERFACE.md §4, spec §5.3, §6)

  areal mass [kg/m^2] / bulk density [kg/m^3] = column thickness [m].

This is load-bearing for berm building via bulking (spec §7): cut dense in-situ regolith
(high density) into the drum, dump it at loose spoil density (lower density) and the SAME
mass occupies MORE height — "a bucket deposits more volume than the hole it left." Book in
height and cut/fill never reconciles; book in mass with density mediating height and it
closes exactly.
"""

from __future__ import annotations

import contextlib
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

from . import constants as K


class StateLabel(IntEnum):
    """Discrete terrain class over continuous state (spec §6; INTERFACE.md §4)."""

    VIRGIN = K.STATE_VIRGIN
    TREAD = K.STATE_TREAD
    EXCAVATED = K.STATE_EXCAVATED
    SPOIL = K.STATE_SPOIL
    COMPACTED_BERM = K.STATE_COMPACTED_BERM
    SINTERED = K.STATE_SINTERED


@dataclass
class ColumnState:
    """A uniform fine grid of per-column regolith state (spec §4 solve grid).

    Arrays are row-major C order (INTERFACE.md §2): index[row, col] -> world
    x = col*cell_m, z = row*cell_m. ``width`` = number of columns (x), ``height`` =
    number of rows (z).

    Fields (all the REQUIRED contract rasters plus the datum/inventory bookkeeping):
        mass_areal  [kg/m^2]  THE invariant. shape (height, width) float64 internally.
        density     [kg/m^3]  current bulk density.
        state_label uint8 enum {0..4}.
        disturbance [0,1]     normalized "how worked is this cell".
        ice         [0,~0.06] OPTIONAL volatile mass fraction (None if dry).
        datum       [m]       elevation that mass/density thickness is added to.
        drum_inventory [kg]   absolute mass currently held in the excavator drums.

    We keep mass_areal/density/disturbance in float64 for accumulation accuracy and
    downcast to '<f4' only at save time (io_fields), so conservation tests are not
    polluted by float32 rounding.
    """

    width: int
    height: int
    cell_m: float

    mass_areal: np.ndarray = field(default=None)   # (height, width) kg/m^2
    density: np.ndarray = field(default=None)       # (height, width) kg/m^3
    state_label: np.ndarray = field(default=None)   # (height, width) uint8
    disturbance: np.ndarray = field(default=None)   # (height, width) [0,1]
    ice: np.ndarray = field(default=None)           # (height, width) [0,~0.06] or None
    datum: np.ndarray = field(default=None)         # (height, width) m

    drum_inventory: float = 0.0  # kg held in drums (not on the grid)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:                                  # N14: validate public ctor
            raise ValueError(
                f"ColumnState grid must be positive (got width={self.width}, height={self.height})")
        if self.cell_m <= 0:
            raise ValueError(f"ColumnState cell_m (cell size in m) must be > 0 (got {self.cell_m})")
        shape = (self.height, self.width)
        if self.mass_areal is None:
            # Default: a uniform loose layer ~ Z_T thick at surface density.
            thickness0 = K.Z_T
            self.mass_areal = np.full(shape, K.RHO_SURFACE * thickness0, dtype=np.float64)
        if self.density is None:
            self.density = np.full(shape, K.RHO_SURFACE, dtype=np.float64)
        if self.state_label is None:
            self.state_label = np.full(shape, StateLabel.VIRGIN, dtype=np.uint8)
        if self.disturbance is None:
            self.disturbance = np.zeros(shape, dtype=np.float64)
        if self.datum is None:
            self.datum = np.zeros(shape, dtype=np.float64)

    # -- geometry ----------------------------------------------------------

    @property
    def cell_area(self) -> float:
        """Plan area of one column [m^2]."""
        return self.cell_m * self.cell_m

    def derive_height(self) -> np.ndarray:
        """Heightmap [m] = datum + mass_areal/density (INTERFACE.md §4; spec §5.3, §6).

        Height is NEVER stored independently — always recomputed. The conservation test
        (spec §10 invariant 2) asserts the saved heightmap equals this after every op.
        """
        return self.datum + self.mass_areal / self.density

    # -- mass accounting (spec §10 invariant 1) ----------------------------

    def grid_mass(self) -> float:
        """Total mass on the grid [kg] = Σ(mass_areal · cell_area)."""
        return float(self.mass_areal.sum()) * self.cell_area

    def total_mass(self) -> float:
        """Conserved total [kg] = grid mass + drum inventory (spec §10 invariant 1).

        Sublimation sink (spec §8) is intentionally excluded — volatiles are an optics
        effect, not a bulk mass sink, so they stay out of the invariant.
        """
        return self.grid_mass() + self.drum_inventory

    def check_invariants(self) -> None:
        """N14: runtime guard for the conserved-state invariants (spec §10), raising on violation. Unlike
        the test-only assertions, this is callable in production / CI: mass finite and non-negative, density
        positive (so height is well-defined), derived height finite, drum inventory non-negative."""
        if not np.all(np.isfinite(self.mass_areal)):
            raise ValueError("ColumnState invariant violated: mass_areal has non-finite values")
        if np.any(self.mass_areal < 0.0):
            raise ValueError("ColumnState invariant violated: negative mass_areal")
        if np.any(self.density <= 0.0):
            raise ValueError("ColumnState invariant violated: non-positive density (height undefined)")
        if not np.all(np.isfinite(self.derive_height())):
            raise ValueError("ColumnState invariant violated: derived height is non-finite")
        if self.drum_inventory < 0.0:
            raise ValueError("ColumnState invariant violated: negative drum_inventory")

    @contextlib.contextmanager
    def conserves_mass(self, *, rtol: float = 1e-9):
        """N14: guard a mutating block that must not create or destroy mass -- total_mass() is asserted
        unchanged (within rtol) on exit. Use as ``with cs.conserves_mass(): cs.<op>()``."""
        before = self.total_mass()
        yield
        after = self.total_mass()
        ref = max(abs(before), 1.0)
        if abs(after - before) > rtol * ref:
            raise ValueError(
                f"mass not conserved across block: {before:.6g} -> {after:.6g} kg "
                f"(rel drift {abs(after - before) / ref:.2e} > rtol {rtol:.0e})")

    # -- column-thickness helpers (used by carving / rover / sandpile) -----

    def thickness(self) -> np.ndarray:
        """Loose-column thickness [m] = mass_areal / density (height above datum)."""
        return self.mass_areal / self.density

    def set_height_via_mass(self, target_height: np.ndarray) -> None:
        """Set mass_areal so derive_height() == target_height at current density/datum.

        Used by procedural generators that author a desired SURFACE first, then back it
        out to the conserved mass field. After this, mass_areal is the source of truth
        and height is re-derived (never re-stored). mass_areal stays >= 0.
        """
        thick = np.maximum(target_height - self.datum, 0.0)
        self.mass_areal = thick * self.density

    # -- drum inventory transfers (cut / dump) -----------------------------

    def cut_to_inventory(self, mask: np.ndarray, mass_per_cell: np.ndarray | float) -> float:
        """Remove ``mass_per_cell`` (areal kg/m^2) from masked cells into the drum.

        Conserves: mass leaving the grid is added to drum_inventory (in absolute kg).
        Returns the absolute kg moved. Clamps so mass_areal stays >= 0.
        """
        m = np.zeros_like(self.mass_areal)
        m[mask] = np.minimum(self.mass_areal[mask], np.broadcast_to(mass_per_cell, self.mass_areal.shape)[mask])
        self.mass_areal[mask] -= m[mask]
        moved_kg = float(m.sum()) * self.cell_area
        self.drum_inventory += moved_kg
        return moved_kg

    def dump_from_inventory(self, mask: np.ndarray, total_kg: float, spoil_density: float = K.RHO_SPOIL) -> float:
        """Deposit up to ``total_kg`` (absolute) from the drum onto masked cells as SPOIL.

        Dumped material lands at loose spoil density (bulking, spec §7) so the same mass
        occupies more height than it did in-situ. Spread evenly over the masked cells.
        Returns the absolute kg actually deposited (limited by drum inventory).
        """
        n = int(mask.sum())
        if n == 0:
            return 0.0
        place_kg = min(total_kg, self.drum_inventory)
        per_cell_areal = (place_kg / n) / self.cell_area  # kg/m^2 per masked cell
        # Blend density toward spoil density for the deposited fraction.
        new_areal = self.mass_areal.copy()
        new_areal[mask] += per_cell_areal
        # Where we deposited onto existing material, mix densities by mass; on bare cells
        # (areal ~0) the result is just spoil_density.
        with np.errstate(invalid="ignore", divide="ignore"):
            old = self.mass_areal[mask]
            add = per_cell_areal
            mixed_rho = (old + add) / (old / self.density[mask] + add / spoil_density)
        self.density[mask] = np.where(np.isfinite(mixed_rho), mixed_rho, spoil_density)
        self.mass_areal = new_areal
        self.state_label[mask] = StateLabel.SPOIL
        self.disturbance[mask] = np.clip(self.disturbance[mask] + 0.3, 0.0, 1.0)
        self.drum_inventory -= place_kg
        return place_kg

    def deposit_field(self, mask: np.ndarray, mass_per_cell: np.ndarray | float,
                      spoil_density: float = K.RHO_SPOIL) -> float:
        """Per-cell counterpart of ``cut_to_inventory``: deposit a PER-CELL areal mass field
        [kg/m^2] from the drum onto masked cells as SPOIL.

        Unlike ``dump_from_inventory`` (which spreads a scalar kg *evenly* and so overshoots
        cells already near target on an uneven deficit), this places exactly what each cell's
        field entry asks for, so a build/fill never overshoots. Deposited material lands at loose
        ``spoil_density`` (bulking, spec §7); the density mix is volume-preserving, so a cell's
        height rises by exactly ``deposited_areal / spoil_density`` regardless of the material
        already there. Conserves mass: the field is scaled down to fit available drum inventory
        if it would exceed it. Returns the absolute kg placed.
        """
        field = np.zeros_like(self.mass_areal)
        field[mask] = np.maximum(np.broadcast_to(mass_per_cell, self.mass_areal.shape)[mask], 0.0)
        want_kg = float(field.sum()) * self.cell_area
        if want_kg <= 0.0 or self.drum_inventory <= 0.0:
            return 0.0
        if want_kg > self.drum_inventory:                  # scale to available inventory -> conserved
            field *= self.drum_inventory / want_kg
            want_kg = self.drum_inventory
        placed = field > 0.0
        old = self.mass_areal.copy()
        self.mass_areal = old + field
        with np.errstate(invalid="ignore", divide="ignore"):
            mixed = (old + field) / (old / self.density + field / spoil_density)
        self.density[placed] = np.where(np.isfinite(mixed[placed]), mixed[placed], spoil_density)
        self.state_label[placed] = StateLabel.SPOIL
        self.disturbance[placed] = np.clip(self.disturbance[placed] + 0.3, 0.0, 1.0)
        self.drum_inventory -= want_kg
        return want_kg

    def fill_toward(self, mask: np.ndarray, target_height: np.ndarray | float,
                    max_lift_m: float | None = None, spoil_density: float = K.RHO_SPOIL) -> float:
        """Build/fill convenience: raise masked cells TOWARD ``target_height`` (never above it)
        by depositing drum material via :meth:`deposit_field`.

        Per-cell lift is the height deficit, optionally capped at ``max_lift_m`` (one macro step).
        Because the deposit is volume-preserving, the areal mass needed to raise a cell by ``dh``
        is ``dh * spoil_density``. If the drum can't supply the full field, every cell is filled
        proportionally less (still no overshoot). Returns the absolute kg placed.
        """
        h = self.derive_height()
        d = np.maximum(target_height - h, 0.0)
        if max_lift_m is not None:
            d = np.minimum(d, max_lift_m)
        deficit = np.zeros_like(self.mass_areal)
        deficit[mask] = np.broadcast_to(d, self.mass_areal.shape)[mask]
        return self.deposit_field(mask, deficit * spoil_density, spoil_density=spoil_density)

    def sinter(self, mask: np.ndarray, sintered_density: float = K.RHO_SINTERED) -> float:
        """Fuse masked cells into a hard SINTERED surface (the lunar concrete/asphalt analog).

        Solar/microwave/laser sintering collapses porosity: density rises to ``sintered_density``,
        so MASS is conserved and the column thins (height re-derives lower) -- a mass-conserving
        densification + a phase/label change, never adding or removing mass. State -> SINTERED and
        disturbance is cleared (a bonded crust, not loose dust). Returns the kg fused, for the sinter
        energy cost (constants.SINTER_ENERGY_J_PER_KG); the authority does NOT model that energy.
        """
        sintered_kg = float(self.mass_areal[mask].sum()) * self.cell_area
        self.density[mask] = np.maximum(self.density[mask], float(sintered_density))
        self.state_label[mask] = np.uint8(StateLabel.SINTERED)
        self.disturbance[mask] = 0.0
        return sintered_kg

    # -- field bundle for io_fields ---------------------------------------

    def fields_dict(self) -> dict[str, np.ndarray]:
        """The REQUIRED contract rasters (INTERFACE.md §1), height re-derived live."""
        out = {
            "heightmap": self.derive_height(),
            "mass_areal": self.mass_areal,
            "density": self.density,
            "disturbance": self.disturbance,
            "state_label": self.state_label,
        }
        if self.ice is not None:
            out["ice"] = self.ice
        return out


def loose_mask(cs: ColumnState) -> np.ndarray:
    """Cells eligible for sandpile relaxation (spec §5.3 "Disturbed flag", §7).

    A cell is loose only if it is BOTH unpaved (not a compacted rut/berm) AND below the
    mid-density: VIRGIN/EXCAVATED/SPOIL fresh spoil relaxes. TREAD/COMPACTED_BERM hold
    their slope (compacted by construction, regardless of density), SINTERED holds (dense),
    and any CEMENTED (high-ice) cell holds. (Using OR floated a fresh single rut and even a
    dense sintered cell into "loose", contradicting the spec.)
    """
    label = cs.state_label
    not_paved = (label != StateLabel.TREAD) & (label != StateLabel.COMPACTED_BERM)
    soft = cs.density < (0.5 * (K.RHO_SURFACE + K.RHO_DEEP))  # below the mid-density
    mask = not_paved & soft
    if cs.ice is not None:
        cemented = cs.ice > 0.5 * K.W_ICE_MAX  # CEMENTED disables relaxation (spec §8)
        mask &= ~cemented
    return mask
