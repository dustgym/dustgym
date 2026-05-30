"""Mass-exact variable-resolution operators + tile extraction (spec §2, NORMATIVE §2.4;
INTERFACE.md §5.3).

These back **Mode B — corridor refinement** (spec §2.1): keep a coarse base over the whole
field, store/render fine TILES only where the rover interacted (the quadtree active/touched
set, §5.1). Cost scales with path length, not area (eval §8). This module supplies the two
resolution operators that move ``ColumnState`` data between a base ``cell_m`` and a finer
``fine_cell_m``, and the tile extractor that packages base-cell-aligned blocks as
fine-resolution bundles for the §5.3 ``tiles[]`` sidecar.

THE invariant is mass (INTERFACE.md §4, spec §10): ``mass_areal`` [kg/m^2] is conserved and
height is DERIVED, never authored:

    height = datum + mass_areal / density        (INTERFACE.md §4; spec §2.4, §5.3)

Both operators are mass-exact and preserve that identity. Design choices (all NORMATIVE,
spec §2.4 / INTERFACE.md §5.3 "CONSERVATION INVARIANT"):

  * ``mass_areal`` is INTENSIVE (kg/m^2) so REFINE COPIES it to each child (does NOT divide).
    Total mass ``Σ mass_areal·cell_area`` is unchanged because each child cell_area is
    ``1/k^2`` of the parent and there are ``k^2`` children.
  * COARSEN uses the *mass-weighted HARMONIC mean* of child densities,
    ``density_coarse = mass_areal_coarse / mean(thickness_fine)`` — chosen so the coarse
    height equals the AREA-MEAN of child heights. It is emphatically NOT ``Σ(mass·ρ)/Σmass``
    (that breaks the height identity). See spec §2.4 for the algebra.
  * ``state_label`` coarsens by a TOTAL priority order EXCAVATED > SPOIL > COMPACTED_BERM >
    TREAD > VIRGIN ("most-worked / most-salient wins") — deterministic, tie-free, and
    associative across multi-level coarsening (NOT statistical mode, which is ill-defined on
    ties and can erase excavation evidence).

By construction ``coarsen(refine(x)) == x`` BIT-EXACT in float64 (drift 0), and for any base
cell overlapped by a tile the base value equals ``coarsen()`` of that tile's fine cells
(base<->tile consistency, INTERFACE.md §5.3 normative invariant).

Pure NumPy, deterministic, dependency-free. Working precision is float64 (as ColumnState
already keeps; spec §2.4 implementation note: only down-cast to '<f4' at save).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import constants as K
from .column_state import ColumnState, StateLabel

# ---------------------------------------------------------------------------
# State-label coarsening priority (spec §2.4 / INTERFACE.md §5.3, NORMATIVE).
# A TOTAL order over the 5 StateLabel values: "most-worked / most-salient wins".
#   EXCAVATED > SPOIL > COMPACTED_BERM > TREAD > VIRGIN.
# Built structures and excavation evidence outrank a plain rut; a total order makes the
# block reduction deterministic, tie-free, and ASSOCIATIVE so multi-level coarsening agrees
# with single-level (verified by brute force over all 5^4 child combinations in tests).
# Implemented as a rank LUT indexed by the uint8 label value -> priority; we then reduce a
# block by argmax over the rank and map back, so the operation is a pure array op (no mode).
# ---------------------------------------------------------------------------
_STATE_PRIORITY: dict[int, int] = {
    int(StateLabel.EXCAVATED): 4,
    int(StateLabel.SPOIL): 3,
    int(StateLabel.COMPACTED_BERM): 2,
    int(StateLabel.TREAD): 1,
    int(StateLabel.VIRGIN): 0,
}

# rank LUT: label value (0..4) -> priority. Built so _STATE_RANK[label] is vectorizable.
_STATE_RANK = np.zeros(max(_STATE_PRIORITY) + 1, dtype=np.int64)
for _lbl, _pri in _STATE_PRIORITY.items():
    _STATE_RANK[_lbl] = _pri
# inverse: priority -> label value, for mapping the winning rank back to a label.
_RANK_TO_LABEL = np.zeros(max(_STATE_PRIORITY.values()) + 1, dtype=np.uint8)
for _lbl, _pri in _STATE_PRIORITY.items():
    _RANK_TO_LABEL[_pri] = _lbl

# Fields that refine/coarsen operate over (the 5 REQUIRED rasters' source data + datum).
# ``heightmap`` is DERIVED (column_state.derive_height), never carried here.
_FIELD_NAMES = ("mass_areal", "density", "datum", "state_label", "disturbance")

# Tolerance for accepting base_cell_m/fine_cell_m as an integer k (spec §6.2d: a
# non-integer ratio must be REJECTED, not silently truncated). Tight relative tolerance
# absorbs float division noise (e.g. 0.02/0.01 = 1.9999999999999998) while still rejecting
# genuine non-integers like 0.02/0.012 = 1.666...
_K_REL_TOL = 1e-9


# ---------------------------------------------------------------------------
# 1. k_factor — refine ratio validation (spec §2.4 / §6.2d, NORMATIVE).
# ---------------------------------------------------------------------------

def k_factor(base_cell_m: float, fine_cell_m: float) -> int:
    """Refine factor ``k = base_cell_m / fine_cell_m`` as a positive integer.

    The refinement factor MUST be a positive integer so a fine tile is a base-cell-aligned
    k x k block (spec §2.4; INTERFACE.md §5.3). A non-integer ratio is REJECTED with
    ``ValueError`` (spec §6.2d: "must be rejected, not silently truncated"), as is a
    non-positive ratio.

    The check rounds to the nearest integer and accepts only if the ratio sits within a
    tight relative tolerance of it (``_K_REL_TOL``), which absorbs IEEE-754 division noise
    (0.02/0.01 lands at 1.999...8) without admitting a genuine non-integer (0.02/0.012).
    """
    if base_cell_m <= 0.0 or fine_cell_m <= 0.0:
        raise ValueError(
            f"k_factor: cell sizes must be positive (base={base_cell_m}, fine={fine_cell_m})"
        )
    ratio = base_cell_m / fine_cell_m
    k_round = int(round(ratio))
    if k_round < 1:
        raise ValueError(
            f"k_factor: base_cell_m/fine_cell_m = {ratio} < 1 (fine must be <= base)"
        )
    if abs(ratio - k_round) > _K_REL_TOL * k_round:
        raise ValueError(
            f"k_factor: base_cell_m/fine_cell_m = {ratio} is not a positive integer "
            f"(nearest {k_round}); a tile must be a base-cell-aligned k x k block "
            f"(spec §2.4 / §6.2d)"
        )
    return k_round


# ---------------------------------------------------------------------------
# Field-bundle helpers — operate on raw numpy dicts so tests can call directly.
# ---------------------------------------------------------------------------

def _as_field_dict(arrays: ColumnState | dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Normalize a ColumnState or a fields dict to a float64 working-array dict.

    Returns the 5 carried fields (mass_areal, density, datum, state_label, disturbance) as
    new float64 arrays (state_label round-trips through float64 only as a carrier and is
    returned uint8). Accepting either type lets tests pass raw arrays while producers pass a
    ColumnState (the "ColumnState-typed convenience" of the task).
    """
    if isinstance(arrays, ColumnState):
        src = {
            "mass_areal": arrays.mass_areal,
            "density": arrays.density,
            "datum": arrays.datum,
            "state_label": arrays.state_label,
            "disturbance": arrays.disturbance,
        }
    else:
        src = arrays
    out: dict[str, np.ndarray] = {}
    for name in _FIELD_NAMES:
        if name not in src:
            raise ValueError(f"refinement: missing field '{name}' (need {_FIELD_NAMES})")
        out[name] = np.asarray(src[name])
    return out


# ---------------------------------------------------------------------------
# 2. refine_field — coarse cell -> k x k fine cells (spec §2.4, NORMATIVE).
# ---------------------------------------------------------------------------

def refine_field(base_arrays: ColumnState | dict[str, np.ndarray], k: int,
                 ) -> dict[str, np.ndarray]:
    """Refine a (H, W) base bundle to (H*k, W*k) by piecewise-constant copy (spec §2.4).

    Each k x k child of a parent cell is a VERBATIM copy of the parent's value for every
    carried field. ``mass_areal`` is INTENSIVE (kg/m^2) so it is COPIED, not divided: total
    mass ``Σ mass_areal·cell_area`` is identical because each child cell_area is ``1/k^2`` of
    the parent's and there are ``k^2`` children. ``height = datum + mass_areal/density`` is
    unchanged everywhere. Drift = 0 (bit-exact; verified in tests).

    Implemented as a 2-axis block expansion (``np.repeat`` on axis 0 then axis 1, the
    Kronecker-with-ones-block expansion) so the parent at (r, c) maps to children
    [r*k:(r+1)*k, c*k:(c+1)*k]. Working arrays are float64; state_label stays uint8.

    Parameters
    ----------
    base_arrays : ColumnState | dict
        Source bundle (5 carried fields). A ColumnState is accepted for convenience.
    k : int
        Positive integer refine factor (use ``k_factor`` to derive/validate it).

    Returns
    -------
    dict[str, np.ndarray]
        Fine bundle with each field at (H*k, W*k). mass_areal/density/datum/disturbance are
        float64; state_label is uint8.
    """
    if not isinstance(k, (int, np.integer)) or k < 1:
        raise ValueError(f"refine_field: k must be a positive integer, got {k!r}")
    k = int(k)
    src = _as_field_dict(base_arrays)

    fine: dict[str, np.ndarray] = {}
    for name in _FIELD_NAMES:
        a = src[name]
        if a.ndim != 2:
            raise ValueError(f"refine_field: field '{name}' must be 2-D, got shape {a.shape}")
        # Block-expand: repeat each element k times on both axes (piecewise-constant).
        expanded = np.repeat(np.repeat(a, k, axis=0), k, axis=1)
        if name == "state_label":
            fine[name] = expanded.astype(np.uint8)
        else:
            fine[name] = expanded.astype(np.float64, copy=False)
    return fine


# ---------------------------------------------------------------------------
# 3. coarsen_field — k x k fine cells -> one coarse cell (spec §2.4, NORMATIVE).
# ---------------------------------------------------------------------------

def coarsen_field(fine_arrays: ColumnState | dict[str, np.ndarray], k: int,
                  ) -> dict[str, np.ndarray]:
    """Coarsen a (H*k, W*k) fine bundle to (H, W) by the NORMATIVE §2.4 block rules.

    Each k x k equal-area block reduces by (reshaping to (H, k, W, k) and reducing the two
    k-axes):

      * ``mass_areal_coarse = mean(mass_areal_fine)`` over the block.
        Conserves total mass exactly (equal-area children ⇒ area-weighted mean = simple
        mean, and child count k^2 × child area 1/k^2 = parent area).
      * ``thickness_fine = mass_areal_fine / density_fine``;
        ``density_coarse = mass_areal_coarse / mean(thickness_fine)``.
        This is the MASS-WEIGHTED HARMONIC mean of the child densities, chosen precisely so
        ``mass_areal_coarse/density_coarse = mean(thickness_fine)`` ⇒ coarse height =
        area-mean of child heights. NOT ``Σ(mass·ρ)/Σmass`` (that breaks the identity).
      * ZERO-MASS BRANCH: where ``mean(thickness_fine)==0`` (all children empty ⇒
        ``mass_areal_coarse==0``), set ``density_coarse = mean(density_fine)`` to avoid 0/0;
        then ``height_coarse = datum_coarse``. Precondition: ``density_fine > 0`` everywhere
        (VIRGIN cells carry the regolith bulk density, never 0 — column_state default).
      * ``datum_coarse = mean(datum_fine)`` — required so area-mean-height holds for
        non-uniform datum.
      * ``state_label_coarse`` = highest-priority child label by the TOTAL order
        EXCAVATED > SPOIL > COMPACTED_BERM > TREAD > VIRGIN (see ``_STATE_PRIORITY``).
      * ``disturbance_coarse = mean(disturbance_fine)`` — intentional areal average.

    Guarantees ``coarsen(refine(x)) == x`` BIT-EXACT in float64 (the refine-copy makes every
    block uniform, so every mean is an identity) and preserves the height identity.

    Parameters
    ----------
    fine_arrays : ColumnState | dict
        Fine bundle whose dims are (H*k, W*k) (divisible by k on both axes).
    k : int
        Positive integer coarsen factor.

    Returns
    -------
    dict[str, np.ndarray]
        Coarse bundle at (H, W). mass_areal/density/datum/disturbance float64; state_label
        uint8.
    """
    if not isinstance(k, (int, np.integer)) or k < 1:
        raise ValueError(f"coarsen_field: k must be a positive integer, got {k!r}")
    k = int(k)
    src = _as_field_dict(fine_arrays)

    hf, wf = src["mass_areal"].shape
    if hf % k != 0 or wf % k != 0:
        raise ValueError(
            f"coarsen_field: fine dims {(hf, wf)} not divisible by k={k} "
            "(a tile must be an integer block of base cells)"
        )
    h, w = hf // k, wf // k

    def _block(name: str) -> np.ndarray:
        """Reshape a fine field to (H, k, W, k) for block reduction over the two k-axes."""
        return src[name].astype(np.float64, copy=False).reshape(h, k, w, k)

    def _uniform_aware_mean(block: np.ndarray) -> np.ndarray:
        """Area-mean over each k x k block, but copy the shared value VERBATIM where the block
        is homogeneous (bit-exactness, spec §2.4 "Drift = 0 ... bit-exact").

        Mathematically this is the mean (mean of k^2 identical values == that value), but
        ``np.mean`` of k^2 equal float64s is only bit-exact when the k^2 sum/divide round-trips
        (k=2, 4, ...); a plain mean drifts ~1e-13 at e.g. k=8 — and k=8 is the spec's own
        mission config (§2.5/§8 "base 8 cm + 1 cm touched band"). The refine-copy makes every
        block uniform, so copying the shared value where ``min==max`` is what keeps
        ``coarsen(refine(x)) == x`` drift-0 for ALL integer k (the density branch below already
        does the same for ρ). A genuinely heterogeneous block takes the true area-mean.
        """
        lo = block.min(axis=(1, 3))
        hi = block.max(axis=(1, 3))
        out = block.mean(axis=(1, 3))            # (H, W) area-mean (equal-area children)
        uniform = lo == hi
        out[uniform] = lo[uniform]               # homogeneous block -> exact shared value
        return out

    mass_b = _block("mass_areal")
    rho_b = _block("density")
    datum_b = _block("datum")
    dist_b = _block("disturbance")

    # Reduce over the two k-axes (1 and 3) -> (H, W). Equal-area children, so the area-weighted
    # mean is the simple mean; the uniform-aware variant copies homogeneous blocks verbatim so
    # the round-trip is bit-exact for every k (see _uniform_aware_mean).
    mass_coarse = _uniform_aware_mean(mass_b)

    # thickness = mass_areal/density (column thickness [m]). Precondition density_fine>0, so
    # this is finite everywhere; the only zero comes from mass_areal==0 (an empty cell).
    thickness_fine = mass_b / rho_b
    thickness_mean = thickness_fine.mean(axis=(1, 3))

    datum_coarse = _uniform_aware_mean(datum_b)
    dist_coarse = _uniform_aware_mean(dist_b)

    # density_coarse = mass_areal_coarse / mean(thickness_fine), with two exact special
    # cases layered on the general formula:
    #
    #   ZERO-MASS BRANCH (spec §2.4): mean(thickness_fine)==0 ⇔ every child empty ⇔
    #     mass_coarse==0; the division would be 0/0. Substitute density_coarse =
    #     mean(density_fine) (finite, >0) so height_coarse = datum_coarse + 0/ρ =
    #     datum_coarse (no NaN/inf).
    #
    #   UNIFORM-DENSITY BRANCH (bit-exactness, spec §2.4 "Drift = 0 ... bit-exact"): when
    #     every child in a block shares ONE density (the case ``coarsen(refine(x))`` always
    #     produces), the mass-weighted harmonic mean of identical densities IS that density
    #     exactly. Evaluating the general formula ``mass_coarse / mean(mass/ρ)`` instead
    #     round-trips through ``mass/ρ`` and back, which is NOT bit-exact in float64
    #     (~1e-13 drift). So where the block density is homogeneous we copy that density
    #     verbatim — mathematically the SAME §2.4 result, just the exact evaluation. This is
    #     what makes ``coarsen(refine(x)) == x`` BIT-EXACT and the base<->tile invariant
    #     (INTERFACE.md §5.3) hold to drift 0. Heterogeneous blocks (a genuine coarsen of
    #     mixed data) take the general harmonic-mean formula.
    rho_min = rho_b.min(axis=(1, 3))
    rho_max = rho_b.max(axis=(1, 3))
    uniform_rho = rho_min == rho_max
    empty = thickness_mean == 0.0
    rho_mean = rho_b.mean(axis=(1, 3))
    density_coarse = np.empty((h, w), dtype=np.float64)
    # general harmonic-mean reconstruction (default); overwritten by the exact branches.
    general = ~empty & ~uniform_rho
    density_coarse[general] = mass_coarse[general] / thickness_mean[general]
    # uniform density (incl. uniform-and-empty) -> copy the shared density exactly. This is
    # also the correct value in the empty-AND-uniform case (== mean(density_fine)).
    density_coarse[uniform_rho] = rho_min[uniform_rho]
    # remaining empty-but-heterogeneous blocks -> mean(density_fine) (zero-mass branch).
    empty_het = empty & ~uniform_rho
    density_coarse[empty_het] = rho_mean[empty_het]

    # state_label_coarse: highest-priority child label by the TOTAL order (NOT mode). Map
    # each child label to its priority rank, take the max rank over the block, map back to a
    # label. Pure array ops; deterministic and associative across multi-level coarsening.
    label_b = src["state_label"].astype(np.int64, copy=False).reshape(h, k, w, k)
    rank_b = _STATE_RANK[label_b]                       # (H, k, W, k) priorities
    rank_coarse = rank_b.max(axis=(1, 3))               # (H, W) winning priority
    state_coarse = _RANK_TO_LABEL[rank_coarse].astype(np.uint8)

    return {
        "mass_areal": mass_coarse,
        "density": density_coarse,
        "datum": datum_coarse,
        "state_label": state_coarse,
        "disturbance": dist_coarse,
    }


# ---------------------------------------------------------------------------
# Convenience: build a fine ColumnState from a base ColumnState block.
# ---------------------------------------------------------------------------

def _slice_block(cs: ColumnState, r0: int, c0: int, r1: int, c1: int,
                 ) -> dict[str, np.ndarray]:
    """Extract the (r1-r0) x (c1-c0) base-cell block of a ColumnState as a field dict."""
    return {
        "mass_areal": cs.mass_areal[r0:r1, c0:c1],
        "density": cs.density[r0:r1, c0:c1],
        "datum": cs.datum[r0:r1, c0:c1],
        "state_label": cs.state_label[r0:r1, c0:c1],
        "disturbance": cs.disturbance[r0:r1, c0:c1],
    }


def _column_state_from_fields(fields: dict[str, np.ndarray], cell_m: float,
                              ice: np.ndarray | None = None) -> ColumnState:
    """Wrap a refined field bundle as a ColumnState at ``cell_m`` (height stays derived).

    The fine bundle's dims set width/height; mass_areal remains the source of truth so
    ``derive_height()`` is unchanged from the base block (refine copies, height invariant).
    """
    mass = np.ascontiguousarray(fields["mass_areal"], dtype=np.float64)
    h, w = mass.shape
    return ColumnState(
        width=w,
        height=h,
        cell_m=cell_m,
        mass_areal=mass,
        density=np.ascontiguousarray(fields["density"], dtype=np.float64),
        state_label=np.ascontiguousarray(fields["state_label"], dtype=np.uint8),
        disturbance=np.ascontiguousarray(fields["disturbance"], dtype=np.float64),
        datum=np.ascontiguousarray(fields["datum"], dtype=np.float64),
        ice=ice,
    )


# ---------------------------------------------------------------------------
# 4. extract_tiles — package base-cell-aligned blocks as fine tiles (INTERFACE.md §5.3).
# ---------------------------------------------------------------------------

@dataclass
class Tile:
    """One fine-resolution refinement tile (INTERFACE.md §5.3 ``tiles[]`` descriptor).

    Carries the on-disk-sidecar fields the §5.3 contract freezes plus the live fine
    ColumnState. By construction ``coarsen(this tile)`` equals the base block it came from
    (base<->tile consistency, §5.3 normative), because refine copies and coarsen(refine)==id.

    Attributes
    ----------
    id : int
        Unique within a frame (assigned by extract_tiles in scan order).
    region_rc : list[int]
        ``[r0, c0, r1, c1]`` half-open, in BASE cells; base-cell-aligned by construction.
    cell_m : float
        The tile (fine) cell size = ``fine_cell_m``.
    cs : ColumnState
        The fine-resolution ColumnState; dims == (r1-r0)*k x (c1-c0)*k.
    """

    id: int
    region_rc: list[int]
    cell_m: float
    cs: ColumnState

    @property
    def fields(self) -> dict[str, np.ndarray]:
        """The fine bundle as a raw field dict (mass_areal/density/datum/state_label/disturbance)."""
        return {
            "mass_areal": self.cs.mass_areal,
            "density": self.cs.density,
            "datum": self.cs.datum,
            "state_label": self.cs.state_label,
            "disturbance": self.cs.disturbance,
        }

    def descriptor(self, dir_: str | None = None) -> dict:
        """JSON-friendly §5.3 ``tiles[]`` entry (id, region_rc, cell_m, optional dir)."""
        d = {"id": int(self.id), "region_rc": [int(v) for v in self.region_rc],
             "cell_m": float(self.cell_m)}
        if dir_ is not None:
            d["dir"] = dir_
        return d


def merge_leaf_boxes_to_aligned_regions(leaf_boxes, base_cell_m: float, fine_cell_m: float,
                                        ) -> list[list[int]]:
    """Consolidate raw leaf boxes into base-cell-aligned, de-duplicated regions (optional).

    The §5.1 quadtree leaf boxes are already base-cell-aligned (they tile the base grid), so
    this is light: it validates k is an integer, drops empty/degenerate boxes, de-duplicates
    identical boxes, and returns them sorted by (r0, c0) for deterministic tile ordering. It
    does NOT geometrically union overlapping boxes — overlap is a producer error per §5.3
    ("Producers MUST NOT emit overlapping tiles"); ``extract_tiles`` asserts disjointness.

    Returns a list of ``[r0, c0, r1, c1]`` half-open base-cell regions.
    """
    k_factor(base_cell_m, fine_cell_m)  # validate integer k (raises if not)
    seen: set[tuple[int, int, int, int]] = set()
    out: list[list[int]] = []
    for box in leaf_boxes:
        r0, c0, r1, c1 = (int(box[0]), int(box[1]), int(box[2]), int(box[3]))
        if r1 <= r0 or c1 <= c0:
            continue  # degenerate / empty box
        key = (r0, c0, r1, c1)
        if key in seen:
            continue
        seen.add(key)
        out.append([r0, c0, r1, c1])
    out.sort(key=lambda b: (b[0], b[1]))
    return out


def extract_tiles(cs: ColumnState, leaf_boxes, fine_cell_m: float) -> list[Tile]:
    """Build fine-resolution tiles for base-cell-aligned leaf boxes (INTERFACE.md §5.3).

    For each half-open box ``[r0, c0, r1, c1]`` (BASE cells) in ``leaf_boxes``, refine that
    base block to ``fine_cell_m`` (via ``refine_field``) and wrap it as a ``Tile`` at the
    fine resolution. Tiles are assigned unique ascending ``id``s in scan order (sorted by
    (r0, c0)) for determinism.

    Guarantees / preconditions (all §5.3 normative):
      * ``k = base_cell_m / fine_cell_m`` is a positive integer (else ValueError; base
        ``cell_m`` comes from ``cs.cell_m``).
      * each ``region_rc`` is base-cell-aligned (it is an integer block by construction) and
        inside the base grid.
      * the regions are pairwise DISJOINT (no base cell under more than one tile) — asserted
        here via a coverage check (overlap is a producer error, §5.3).
      * the fine raster dims equal ``(r1-r0)*k x (c1-c0)*k``.
      * by construction ``coarsen(tile, k) == base block`` (base<->tile consistency), since
        refine copies and ``coarsen(refine(x)) == x``.

    Parameters
    ----------
    cs : ColumnState
        The BASE-resolution scene (its ``cell_m`` is ``base_cell_m``).
    leaf_boxes : iterable of [r0, c0, r1, c1]
        Base-cell-aligned half-open boxes (e.g. quadtree active/touched leaves, §5.1).
    fine_cell_m : float
        Target fine resolution; ``base_cell_m/fine_cell_m`` must be a positive integer.

    Returns
    -------
    list[Tile]
        One Tile per input box, ordered by (r0, c0), with ascending ``id`` 0..n-1.
    """
    base_cell_m = float(cs.cell_m)
    k = k_factor(base_cell_m, fine_cell_m)

    regions = merge_leaf_boxes_to_aligned_regions(leaf_boxes, base_cell_m, fine_cell_m)

    # Disjointness + bounds check over the base grid (§5.3: regions pairwise disjoint).
    cover = np.zeros((cs.height, cs.width), dtype=np.int32)
    for (r0, c0, r1, c1) in regions:
        if r0 < 0 or c0 < 0 or r1 > cs.height or c1 > cs.width:
            raise ValueError(
                f"extract_tiles: region [{r0},{c0},{r1},{c1}] outside base grid "
                f"({cs.height}x{cs.width})"
            )
        cover[r0:r1, c0:c1] += 1
    if np.any(cover > 1):
        raise ValueError(
            "extract_tiles: leaf_boxes overlap (a base cell is under >1 tile); "
            "producers MUST NOT emit overlapping tiles (INTERFACE.md §5.3)"
        )

    tiles: list[Tile] = []
    for tile_id, (r0, c0, r1, c1) in enumerate(regions):
        block = _slice_block(cs, r0, c0, r1, c1)
        fine = refine_field(block, k)
        fine_cs = _column_state_from_fields(fine, fine_cell_m)
        # Dimension relation (§5.3): fine dims == (r1-r0)*k x (c1-c0)*k.
        assert fine_cs.height == (r1 - r0) * k and fine_cs.width == (c1 - c0) * k, (
            "extract_tiles: fine dims do not match (r1-r0)*k x (c1-c0)*k"
        )
        tiles.append(Tile(id=tile_id, region_rc=[r0, c0, r1, c1],
                          cell_m=float(fine_cell_m), cs=fine_cs))
    return tiles
