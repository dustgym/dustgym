"""I11: as-built acceptance / siting gates the WHOLE footprint on the real DEM, not just the centre cell.

A pad whose centre sits on flat ground but whose edge straddles a steep rim must still be rejected; the
check reports the worst slope over the footprint and the fraction of footprint cells over the threshold.
"""
import math
import os

import numpy as np
import pytest

from planet_browser import mission_planner as MP


def _cut(x, y, footprint_m2):
    return MP.Mission(name="t", body="moon",
                      orders=[MP.BuildOrder("pad", "cut", x, y, footprint_m2, 0.05)])


def test_whole_footprint_gate_catches_an_edge_the_centre_misses():
    # geometric fixture for the gate LOGIC: flat for col<50, a ~26.6deg ramp for col>=50 (rise 0.5 m/cell).
    cell = 1.0
    Z = np.zeros((100, 100), dtype=np.float64)
    Z[:, 50:] = (np.arange(50) * 0.5)[None, :]
    dem = (Z, cell)
    # centre at the flat/steep boundary; a big footprint straddles the steep edge -> rejected
    big = MP.validate_plan(_cut(50.0, 50.0, 400.0), dem=dem, dem_origin=(0.0, 0.0), max_slope_deg=15.0)
    assert big["slope_violations"], "the whole-footprint gate should flag the steep edge"
    sv = big["slope_violations"][0]
    assert sv["slope_deg"] > 15.0 and 0.0 < sv["frac_over"] <= 1.0
    # a small footprint fully inside the flat region -> accepted (centre and footprint both flat)
    small = MP.validate_plan(_cut(20.0, 50.0, 9.0), dem=dem, dem_origin=(0.0, 0.0), max_slope_deg=15.0)
    assert not small["slope_violations"]


BUNDLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "samples", "lunar_dem", "haworth_10km_5m")


@pytest.mark.skipif(not os.path.isdir(BUNDLE), reason="Haworth bundle absent")
def test_whole_footprint_gate_flags_a_real_steep_cell():
    dem = MP.load_haworth_dem()
    Z, cell = dem
    smap = MP.slope_deg_map(Z, cell)
    ri, ci = np.unravel_index(int(np.argmax(smap)), smap.shape)   # the steepest REAL cell (a crater wall)
    assert float(smap[ri, ci]) > 15.0
    foot = (3 * cell) ** 2                                        # ~3-cell footprint centred on it
    v = MP.validate_plan(_cut(0.0, 0.0, foot), dem=dem, dem_origin=(float(ci * cell), float(ri * cell)),
                         max_slope_deg=15.0)
    assert v["slope_violations"] and v["slope_violations"][0]["slope_deg"] > 15.0
    assert math.isfinite(v["slope_violations"][0]["frac_over"])
