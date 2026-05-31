"""Coordinate-hashed seed + global-lattice fbm — the load-bearing determinism primitive
(L0 contract §3; eval §5 step 6; spec §10).

WHY this exists. The repo's ``procgen.fbm`` seeds a single scalar
``np.random.default_rng(seed)`` (``procgen.py:63``) and min-max-renorms the result to [0, 1]
(``procgen.py:74-77``). Both choices are FATAL for a demand-driven 10 km corridor:

  * a scalar seed makes the noise a function of *render order / tile index*, not of the world
    point — so two adjacent corridor tiles disagree at their shared seam, and the SAME world
    point sampled at a 5 m base vs a 1 m base disagrees. The contract's headline ("the rover
    may explore ANY part of the 10 km and re-visiting a patch yields byte-identical terrain")
    cannot hold.
  * the [0, 1] min-max renorm is a realization-dependent nonlinear rescale that destroys the
    PSD slope the Hurst derivation assumes (eval §6 "fbm spectral fidelity").

THE FIX (this module). Seed and sample in the GLOBAL frame:

  * ``coord_seed(gx, gy, octave, base_cell_class)`` hashes the *quantized global coordinate*
    (plus octave + a resolution class + a world seed) into a stable 64-bit integer. The same
    world point yields the same seed regardless of which tile renders it or what base_cell_m
    the caller is on (the resolution class is deliberately NOT the cell size — see below).
  * ``fbm_global(world_x0, world_y0, n, cell_m, ...)`` evaluates value-noise/fbm on the GLOBAL
    integer lattice (lattice node = global coordinate / feature-wavelength), each lattice node
    drawing its value from ``coord_seed`` of THAT node. Two windows that overlap therefore read
    the SAME lattice nodes on the overlap and agree BIT-EXACT; a coarse and a fine window over
    the same span read the same nodes and agree at the shared nodes. Amplitudes are anchored to
    a deviogram target ``nu0`` (NOT min-max renormed), so variance is physical and stable.

Pure NumPy + the stdlib ``hashlib`` (BLAKE2b, a stable cross-process hash — Python's builtin
``hash()`` is salted per-process and MUST NOT be used for reproducible terrain). Deterministic,
dependency-free, no global state.
"""

from __future__ import annotations

import hashlib
import struct
from typing import Callable

import numpy as np

# ---------------------------------------------------------------------------
# Quantization of the global coordinate before hashing.
#
# A continuous global metre coordinate cannot be hashed directly (float jitter ->
# different bytes). We quantize to an integer number of QUANTUM_M-sized steps. The quantum
# must be FINE ENOUGH that distinct lattice nodes never collide, yet COARSE ENOUGH that the
# same intended node computed via slightly different float paths (5 m base vs 1 m base
# arithmetic) lands on the same integer. The fbm lattice spacing is always a clean multiple
# of the feature wavelength (>= 1 m here) and we quantize to 1 mm, so adjacent lattice nodes
# (>= 1 m apart) are 1000 quanta apart — zero collision risk — while float noise (~1e-9 m at
# these magnitudes, eval validation addendum: float32 resolves ~0.3 mm; we work in float64,
# ~1e-9 m over 10 km) rounds away cleanly.
# ---------------------------------------------------------------------------
QUANTUM_M = 1.0e-3  # 1 mm coordinate quantum for the seed hash


def _q(coord_m: float) -> int:
    """Quantize a global metre coordinate to a signed integer count of QUANTUM_M steps.

    ``round`` (banker's-rounding-free here because inputs are clean multiples of the lattice
    spacing) maps a near-integer-mm coordinate to the same integer regardless of the float
    path that produced it. Negative coordinates (the Haworth tiepoint X0=-52900 is negative)
    quantize symmetrically.
    """
    return int(round(float(coord_m) / QUANTUM_M))


# ---------------------------------------------------------------------------
# 1. coord_seed — stable 64-bit hash of a quantized global point (L0 §3).
# ---------------------------------------------------------------------------

def coord_seed(global_x_m: float, global_y_m: float, octave: int,
               base_cell_class: int, *, world_seed: int = 0) -> int:
    """Stable 64-bit seed from a QUANTIZED global coordinate + octave + resolution class.

    The seed is a pure function of the world point, NOT of tile/render order or ``base_cell_m``
    (L0 §3; eval §5 step 6). Two calls for the same ``(global_x_m, global_y_m, octave,
    base_cell_class, world_seed)`` return the same int across processes and runs (BLAKE2b, not
    the per-process-salted builtin ``hash``).

    Parameters
    ----------
    global_x_m, global_y_m : float
        The world point in the GLOBAL stereographic frame (metres). May be negative.
    octave : int
        fbm octave index (each octave samples a different lattice, so it must enter the hash).
    base_cell_class : int
        A RESOLUTION CLASS, not the cell size in metres. It exists so callers that *intend*
        different noise families (e.g. the DEM-overlay residual vs an independent decorative
        layer) get independent draws, while a single layer sampled at 5 m vs 1 m base uses the
        SAME class and therefore agrees. Pass 0 for the default/overlay layer. (Contract wording
        "5 m vs 1 m base agrees" => the cell SIZE must NOT be folded in here; the class is.)
    world_seed : int
        Global scenario seed (0 = default). Lets a whole world be re-rolled deterministically.

    Returns
    -------
    int
        Unsigned 64-bit integer in [0, 2**64).
    """
    qx = _q(global_x_m)
    qy = _q(global_y_m)
    # Pack the 5 integer fields as fixed-width signed/unsigned little-endian so the byte
    # string is canonical (no str() ambiguity, no locale, stable across platforms). qx/qy are
    # signed 64-bit; octave/class/world_seed unsigned 64-bit (masked to 64 bits).
    payload = struct.pack(
        "<qqQQQ",
        qx, qy,
        int(octave) & 0xFFFFFFFFFFFFFFFF,
        int(base_cell_class) & 0xFFFFFFFFFFFFFFFF,
        int(world_seed) & 0xFFFFFFFFFFFFFFFF,
    )
    digest = hashlib.blake2b(payload, digest_size=8).digest()
    return int.from_bytes(digest, "little", signed=False)


def _seed_to_unit(seed: int) -> float:
    """Map a 64-bit seed to a float64 in [0, 1) deterministically (top 53 bits / 2**53)."""
    return (seed >> 11) / float(1 << 53)


def _lattice_values(ix: np.ndarray, iy: np.ndarray, octave: int,
                    base_cell_class: int, world_seed: int) -> np.ndarray:
    """Per-lattice-node random values in [0,1), each from ``coord_seed`` of that GLOBAL node.

    ``ix``/``iy`` are integer lattice indices already expressed in the GLOBAL frame (i.e. the
    same physical node has the same (ix, iy) no matter which window asks). We hash each node
    independently so overlapping windows read identical values on shared nodes — the bit-exact
    seam property. Vectorized over the (ny, nx) node grid.
    """
    ny, nx = ix.shape
    out = np.empty((ny, nx), dtype=np.float64)
    # Hash node-by-node. Node counts are small (feature lattices: a few * span/wavelength),
    # so this stays cheap; the lattice is shared across a whole tile's fine grid.
    for j in range(ny):
        for i in range(nx):
            # Reconstruct the node's GLOBAL coordinate from its integer index * spacing handled
            # by the caller via coord_seed taking metres; here ix/iy ARE already global node
            # metres (the caller scales). We pass them straight through coord_seed.
            s = coord_seed(float(ix[j, i]), float(iy[j, i]), octave,
                           base_cell_class, world_seed=world_seed)
            out[j, i] = _seed_to_unit(s)
    return out


def _smoothstep(t: np.ndarray) -> np.ndarray:
    """Perlin smoothstep fade 3t^2 - 2t^3 (matches procgen._value_noise's fade)."""
    return t * t * (3.0 - 2.0 * t)


def _value_noise_global(world_x0: float, world_y0: float, n: int, cell_m: float,
                        wavelength_m: float, octave: int, base_cell_class: int,
                        world_seed: int) -> np.ndarray:
    """One octave of value noise on the GLOBAL lattice, sampled over an (n x n) window.

    The window's fine grid spans ``[world_x0, world_x0 + n*cell_m)`` (and y likewise), with
    sample centers at ``world_*0 + (i + 0.5)*cell_m`` — cell-CENTER registration so refine/
    coarsen blocks share the same continuous field. Lattice nodes sit at integer multiples of
    ``wavelength_m`` in the GLOBAL frame; each node draws its value from ``coord_seed`` of that
    node's global coordinate. Bilinear interpolation with a smoothstep fade. Two windows that
    overlap read the SAME global nodes on the overlap -> bit-exact agreement.
    """
    # Global sample coordinates of every fine cell center (1-D, then broadcast).
    xs = world_x0 + (np.arange(n, dtype=np.float64) + 0.5) * cell_m   # (nx,)
    ys = world_y0 + (np.arange(n, dtype=np.float64) + 0.5) * cell_m   # (ny,)

    # SNAP each global center to the QUANTUM_M grid (same quantization coord_seed uses). WHY:
    # the same physical center reached via two different (origin, index) splits can differ by
    # one ULP (e.g. 2.0+0.5*0.02 == 2.01 but 0.0+100.5*0.02 == 2.0100000000000002). Snapping to
    # integer-mm and back makes the lattice coordinate (and thus the interpolation weights)
    # PATH-INDEPENDENT, so two overlapping windows agree BIT-EXACT on the overlap regardless of
    # which origin they were computed from. (1 mm << wavelength >= 1 m, so this never shifts a
    # sample to a different lattice cell.)
    xs = np.round(xs / QUANTUM_M) * QUANTUM_M
    ys = np.round(ys / QUANTUM_M) * QUANTUM_M

    # Lattice coordinate (in node units) of each sample.
    gx = xs / wavelength_m
    gy = ys / wavelength_m
    x0 = np.floor(gx).astype(np.int64)
    y0 = np.floor(gy).astype(np.int64)
    tx = (gx - x0)[None, :]
    ty = (gy - y0)[:, None]

    # Unique node indices spanned by this window (so we hash each global node once).
    ux = np.arange(x0.min(), x0.max() + 2, dtype=np.int64)   # +2: need x0 and x0+1
    uy = np.arange(y0.min(), y0.max() + 2, dtype=np.int64)
    # Global metre coordinate of each node = node_index * wavelength_m. Build the (ny,nx) grids.
    node_ix = (ux * wavelength_m)[None, :].repeat(uy.size, axis=0)
    node_iy = (uy * wavelength_m)[:, None].repeat(ux.size, axis=1)
    lat = _lattice_values(node_ix, node_iy, octave, base_cell_class, world_seed)  # (uy, ux)

    # Map each sample's x0/y0 into the local lattice block index.
    lx0 = (x0 - ux[0]).astype(np.int64)   # (nx,)
    ly0 = (y0 - uy[0]).astype(np.int64)   # (ny,)
    lx1 = lx0 + 1
    ly1 = ly0 + 1

    v00 = lat[np.ix_(ly0, lx0)]
    v01 = lat[np.ix_(ly0, lx1)]
    v10 = lat[np.ix_(ly1, lx0)]
    v11 = lat[np.ix_(ly1, lx1)]

    fx = _smoothstep(tx)
    fy = _smoothstep(ty)
    top = v00 * (1.0 - fx) + v01 * fx
    bot = v10 * (1.0 - fx) + v11 * fx
    return top * (1.0 - fy) + bot * fy


# ---------------------------------------------------------------------------
# 2. fbm_global — variance/deviogram-anchored fbm on the GLOBAL lattice (L0 §3).
# ---------------------------------------------------------------------------

def fbm_global(world_x0: float, world_y0: float, n: int, cell_m: float, *,
               H: float | Callable[[float], float] = 0.95, nu0: float = 1.0,
               world_seed: int = 0, octaves: int = 6,
               base_wavelength_m: float = 8.0, lacunarity: float = 2.0,
               base_cell_class: int = 0) -> np.ndarray:
    """Variance-anchored fractional Brownian motion on the GLOBAL lattice (L0 §3; eval §5/§6).

    Sums ``octaves`` of global value-noise (``_value_noise_global``) whose feature wavelength
    halves each octave (``lacunarity``) and whose amplitude follows the Hurst gain
    ``gain = lacunarity**(-H)`` (eval §6: a single fixed gain implies one H; H may be a
    scale-ramp callable of the octave wavelength to model scale-dependent roughness, eval §6
    "H is scale-dependent"). The output is mean-removed and scaled so its sample STANDARD
    DEVIATION equals ``sqrt(nu0)`` — the deviogram/variance anchor — INSTEAD of the
    realization-dependent [0,1] min-max renorm the repo's ``fbm`` uses (``procgen.py:74-77``),
    which destroys the PSD slope (eval §6 "fbm spectral fidelity").

    DETERMINISM / SEAM CONTINUITY (the load-bearing property):
      * every lattice node is seeded by ``coord_seed`` of its GLOBAL coordinate, so two windows
        that overlap agree BIT-EXACT on the overlap (proven in tests);
      * the SAME world point sampled by a coarse window (large cell_m) and a fine window (small
        cell_m) reads the SAME global lattice nodes, so the continuous field agrees at every
        shared node (cross-resolution stability).
    NOTE: this returns the RAW zero-mean residual field. The DEM overlay (dem_overlay.py) is
    responsible for the per-base-cell zero-mean projection that preserves coarsen(fine)==base;
    fbm_global itself only guarantees global determinism + variance anchoring.

    Parameters
    ----------
    world_x0, world_y0 : float
        Global metre coordinate of the window's origin (the (0,0) fine-cell's lower corner).
    n : int
        Window side in fine cells (square window, (n, n) output).
    cell_m : float
        Fine cell size [m] of THIS window.
    H : float | callable
        Hurst exponent (eval §6 ~0.95 resolved band; ~0.5-0.7 cm band). If callable, it is
        evaluated as ``H(wavelength_m)`` per octave for a scale ramp.
    nu0 : float
        Target VARIANCE (deviogram anchor); the field is scaled to std == sqrt(nu0). 0 -> a
        zero field.
    world_seed : int
        Global scenario seed (forwarded to coord_seed).
    octaves : int
        Number of fbm octaves.
    base_wavelength_m : float
        Feature wavelength [m] of octave 0 (octave o uses base/lacunarity**o).
    lacunarity : float
        Wavelength shrink factor per octave (>1).
    base_cell_class : int
        Resolution class forwarded to coord_seed (0 = default/overlay layer; see coord_seed).

    Returns
    -------
    np.ndarray
        (n, n) float64 zero-mean field with sample std == sqrt(nu0) (0 if nu0==0 or n==0).
    """
    if n <= 0:
        return np.zeros((max(n, 0), max(n, 0)), dtype=np.float64)
    if nu0 <= 0.0:
        return np.zeros((n, n), dtype=np.float64)

    total = np.zeros((n, n), dtype=np.float64)
    amp = 1.0
    wavelength = float(base_wavelength_m)
    for o in range(int(octaves)):
        if callable(H):
            h_o = float(H(wavelength))
        else:
            h_o = float(H)
        gain = lacunarity ** (-h_o)
        noise = _value_noise_global(world_x0, world_y0, n, cell_m, wavelength,
                                    octave=o, base_cell_class=base_cell_class,
                                    world_seed=world_seed)
        # Center each octave's value noise (value noise is in [0,1); subtract its node-fair
        # midpoint 0.5 so octaves are zero-mean before the variance anchor) -> mean-stable.
        total += amp * (noise - 0.5)
        amp *= gain
        wavelength /= lacunarity

    # Variance anchor (NOT min-max renorm): remove the sample mean and scale to target std.
    total -= total.mean()
    std = float(total.std())
    if std > 0.0:
        total *= (np.sqrt(nu0) / std)
    return total
