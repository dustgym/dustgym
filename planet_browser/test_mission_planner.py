"""P1 round-trip tests (TDD): build-order queue -> mission_planner adapter -> mission-control report,
and the local /plan server that wires the browser to it.

planet_browser/ is standalone; these run under the runtime venv (numpy + matplotlib). They use the REAL
bodies.json + the real grounded planner (no synthetic constants) and a small REAL mission fixture
(two orders on the Moon). Host-runnable via pytest:

    cd planet_browser && PYTHONPATH=. <venv>/bin/python -m pytest test_mission_planner.py -q
"""
from __future__ import annotations

import json
import math
import os
import threading
import urllib.error
import urllib.request

import pytest

import mission_planner as MP
import server as SRV

HERE = os.path.dirname(os.path.abspath(__file__))


def _payload(orders=None, body="moon", name="Test Site"):
    if orders is None:
        orders = [
            {"action": "Level pad", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04},
            {"action": "Build berm", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10},
        ]
    return {"name": name, "body": body, "charger": [0, 0], "orders": orders}


# ---- mission_from_dict (the browser-queue -> Mission adapter) ------------------------------------
def test_mission_from_dict_roundtrips():
    m = MP.mission_from_dict(_payload())
    assert isinstance(m, MP.Mission)
    assert m.body == "moon" and m.name == "Test Site" and m.charger == (0.0, 0.0)
    assert [o.kind for o in m.orders] == ["cut", "fill"]
    assert m.orders[0].footprint_m2 == 36.0 and m.orders[0].depth_m == 0.04


def test_mission_from_dict_rejects_unknown_body():
    with pytest.raises(ValueError, match="body"):
        MP.mission_from_dict(_payload(body="pluto"))


def test_mission_from_dict_rejects_bad_kind():
    with pytest.raises(ValueError, match="kind"):
        MP.mission_from_dict(_payload(orders=[
            {"action": "x", "kind": "teleport", "x": 1, "y": 1, "footprint_m2": 1, "depth_m": 0.1}]))


def test_mission_from_dict_requires_fields():
    with pytest.raises(ValueError, match="missing"):
        MP.mission_from_dict(_payload(orders=[{"action": "x", "kind": "cut", "x": 1, "y": 1}]))


def test_mission_from_dict_requires_orders():
    with pytest.raises(ValueError, match="orders"):
        MP.mission_from_dict({"name": "x", "body": "moon", "orders": []})


# ---- run() on a queued mission writes a REAL pdf + md, balanced ----------------------------------
def test_queued_mission_balances_and_writes_pdf():
    m = MP.mission_from_dict(_payload())
    pdf, md, totals = MP.run(m, stem="pytest-roundtrip")
    assert os.path.isfile(pdf) and os.path.isfile(md)
    with open(pdf, "rb") as f:
        assert f.read(5) == b"%PDF-"                       # a real PDF, not an empty/placeholder file
    assert totals["cut_kg"] > 0 and totals["fill_kg"] > 0  # both order classes present
    assert abs(totals["cut_kg"] - totals["fill_kg"]) < totals["cut_kg"]  # fill drawn from cut, balanced-ish


def test_run_unique_stem_no_overwrite():
    p1, _, _ = MP.run(MP.mission_from_dict(_payload(name="A")), stem="pytest-A")
    p2, _, _ = MP.run(MP.mission_from_dict(_payload(name="B")), stem="pytest-B")
    assert p1 != p2 and os.path.isfile(p1) and os.path.isfile(p2)


# ---- sinter stays gated through the adapter ------------------------------------------------------
def test_sinter_order_still_refused():
    m = MP.mission_from_dict(_payload(orders=[
        {"action": "Sinter apron", "kind": "sinter", "x": 10, "y": 10, "footprint_m2": 9, "depth_m": 0.01}]))
    with pytest.raises(RuntimeError, match="GATED OFF"):
        MP.plan_and_simulate(m)


# ---- I8: validate the plan on the conserved authority (column_state) ----------------------------
def test_validate_plan_conserves_mass_and_is_feasible():
    import structures as ST
    m = MP.mission_from_dict({"name": "Pad", "body": "moon", "charger": [0, 0],
                              "orders": ST.decompose("landing_pad", 40.0, 30.0)})
    v = MP.validate_plan(m)
    assert v["mass_conserved"] is True and v["feasible"] is True
    # the conserved authority actually moves ~ the planned bank cut and loose fill (material available)
    assert math.isclose(v["executed_cut_kg"], v["planned_cut_kg"], rel_tol=0.05)
    assert math.isclose(v["executed_fill_kg"], v["planned_fill_kg"], rel_tol=0.05)


def test_validate_plan_small_footprint_not_falsely_infeasible():
    # a small fill (6 m^2) the drum can easily supply must read feasible; feasibility means material-
    # limited, NOT that a sub-meter footprint under-covers the 0.5 m grid. (Regression: I8 compared an
    # analytic planned mass to a rasterized executed mass, so small footprints tripped the gate falsely.)
    m = MP.mission_from_dict({"name": "small", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 0.0, "y": 0.0, "footprint_m2": 16.0, "depth_m": 0.04},
        {"action": "fill", "kind": "fill", "x": 10.0, "y": 10.0, "footprint_m2": 6.0, "depth_m": 0.05}]})
    v = MP.validate_plan(m)                       # flat scene, no DEM -> only the mass/feasibility check
    assert v["mass_conserved"] is True and v["feasible"] is True


def test_validate_plan_flags_drum_undersupply():
    # the gate must still fire for a GENUINE under-supply: a big fill drawing on a tiny cut empties the
    # drum far short of the plan -> infeasible (distinct from the small-footprint discretization case).
    m = MP.mission_from_dict({"name": "starved", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 0.0, "y": 0.0, "footprint_m2": 4.0, "depth_m": 0.04},
        {"action": "fill", "kind": "fill", "x": 20.0, "y": 20.0, "footprint_m2": 100.0, "depth_m": 0.30}]})
    v = MP.validate_plan(m)
    assert v["feasible"] is False and v["drum_remaining_kg"] < 1.0      # drum ran dry, fill far short


def test_validate_plan_flags_infeasible_deep_cut():
    # a cut deeper than the regolith mantle floors at the datum -> the authority cannot move it -> infeasible
    m = MP.mission_from_dict({"name": "deep", "body": "moon", "charger": [0, 0],
                              "orders": [{"action": "too deep", "kind": "cut", "x": 0.0, "y": 0.0,
                                          "footprint_m2": 9.0, "depth_m": 50.0}]})
    v = MP.validate_plan(m, regolith_depth_m=10.0)
    assert v["feasible"] is False


def test_validate_plan_slope_gate_on_real_haworth_dem():
    # I6: against the REAL Haworth DEM, a pad on a flat spot is feasible; on a steep crater wall it is
    # rejected for slope. Uses the rare-flattest and steepest real cells (no synthetic terrain).
    import numpy as np
    dem = MP.load_haworth_dem()                       # (Z, cell_m) — real LOLA Haworth 5 m
    Z, cell = dem
    smap = MP.slope_deg_map(Z, cell)
    flat_rc = np.unravel_index(int(np.argmin(smap)), smap.shape)
    steep_rc = np.unravel_index(int(np.argmax(smap)), smap.shape)

    def pad_at(rc):
        x, y = rc[1] * cell, rc[0] * cell             # col -> x, row -> y (DEM meters)
        return MP.mission_from_dict({"name": "p", "body": "moon", "charger": [0, 0],
            "orders": [{"action": "pad", "kind": "cut", "x": x, "y": y, "footprint_m2": 36, "depth_m": 0.05}]})

    vf = MP.validate_plan(pad_at(flat_rc), dem=dem, max_slope_deg=15.0)
    vs = MP.validate_plan(pad_at(steep_rc), dem=dem, max_slope_deg=15.0)
    assert vf["feasible"] is True and not vf["slope_violations"]
    assert vs["feasible"] is False and vs["slope_violations"]


def test_flattest_anchor_finds_a_buildable_spot():
    # M11: auto-find a flat buildable region on the (mostly-steep) Haworth DEM
    import numpy as np
    dem = MP.load_haworth_dem()
    Z, cell = dem
    ax, ay = MP.flattest_anchor(dem, window_m=20.0)
    smap = MP.slope_deg_map(Z, cell)
    col, row = int(ax / cell), int(ay / cell)
    assert smap[row, col] < float(np.median(smap))    # anchor is flatter than typical Haworth


def test_validate_plan_anchored_to_dem_origin():
    # M11: the order LOCAL frame is anchored to a DEM site via dem_origin -> the slope gate fires on the
    # actual anchored terrain. Same pad: feasible at the flat anchor, rejected at the steepest cell.
    import numpy as np
    dem = MP.load_haworth_dem()
    Z, cell = dem
    pad = MP.mission_from_dict({"name": "p", "body": "moon", "charger": [0, 0],
        "orders": [{"action": "pad", "kind": "cut", "x": 0.0, "y": 0.0, "footprint_m2": 16.0, "depth_m": 0.04}]})
    flat = MP.flattest_anchor(dem, window_m=20.0)
    vf = MP.validate_plan(pad, dem=dem, dem_origin=flat, max_slope_deg=15.0)
    sr = np.unravel_index(int(np.argmax(MP.slope_deg_map(Z, cell))), Z.shape)
    vs = MP.validate_plan(pad, dem=dem, dem_origin=(sr[1] * cell, sr[0] * cell), max_slope_deg=15.0)
    assert vf["feasible"] is True
    assert vs["feasible"] is False and vs["slope_violations"]


# ---- AL2: infeasible precedence fails loud, not a silent 0-trip "success" -------------------------
def test_precedence_feasibility_unit():
    assert MP._precedence_is_feasible(3, [(0, 1), (1, 2)]) is True       # a chain: feasible
    assert MP._precedence_is_feasible(2, [(0, 1), (1, 0)]) is False      # a 2-cycle: infeasible
    assert MP._precedence_is_feasible(1, []) is True                     # trivial


def test_cyclic_precedence_fails_loud_not_silent():
    # two separated cut->fill builds whose cross-precedence forms a cycle (A before B and B before A)
    m = MP.mission_from_dict({"name": "cyc", "body": "moon", "charger": [0, 0],
        "orders": [
            {"action": "cutA", "kind": "cut", "x": 5, "y": 5, "footprint_m2": 40, "depth_m": 0.3},
            {"action": "fillA", "kind": "fill", "x": 5, "y": 5, "footprint_m2": 40, "depth_m": 0.3},
            {"action": "cutB", "kind": "cut", "x": 60, "y": 60, "footprint_m2": 40, "depth_m": 0.3},
            {"action": "fillB", "kind": "fill", "x": 60, "y": 60, "footprint_m2": 40, "depth_m": 0.3}],
        "precedence": [["fillA", "cutB"], ["fillB", "cutA"]]})
    with pytest.raises(RuntimeError, match="precedence"):
        MP.plan_and_simulate(m)


def test_optimality_flag_reported_and_exact_for_small_plan():
    # AL1: a small plan is solved exactly (brute) and the optimality is reported, not silent
    m = MP.mission_from_dict({"name": "small", "body": "moon", "charger": [0, 0],
        "orders": [{"action": "cut_pad", "kind": "cut", "x": 5, "y": 5, "footprint_m2": 40, "depth_m": 0.3},
                   {"action": "fill_low", "kind": "fill", "x": 12, "y": 8, "footprint_m2": 40, "depth_m": 0.3}]})
    _, _, _, _, totals = MP.plan_and_simulate(m, algorithm="auto")
    assert totals["optimality"] == "exact"                  # 2 trips <= BRUTE_MAX_TRIPS -> brute = exact
    assert totals["resolved_algorithm"] == "brute"
    # and the default heuristic path is honestly labelled, not silently "exact"
    _, _, _, _, t_nn = MP.plan_and_simulate(m, algorithm="nearest")
    assert t_nn["optimality"] == "heuristic"


# ---- P4: non-polar DEM ingest (reproject a cylindrical lat/lon product to the local metric grid) -
def test_ingest_nonpolar_cylindrical_dem_relief_round_trips(tmp_path):
    import os
    import numpy as np
    import dem_import as di
    _fx = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures")   # CWD-independent
    heights, geom = di.load_cylindrical_fixture(
        os.path.join(_fx, "ldem4_equator_dn.npy"), os.path.join(_fx, "ldem4_equator.json"))
    assert abs(0.5 * (geom["lat_top_deg"] + geom["lat_bottom_deg"])) < 60.0      # genuinely NON-polar
    Z, cell = di.reproject_cylindrical(
        heights, lat_top=geom["lat_top_deg"], lat_bottom=geom["lat_bottom_deg"],
        lon_left=geom["lon_left_deg"], lon_right=geom["lon_right_deg"], radius_m=geom["radius_m"],
        target_cell_m=2000.0)                                                     # fine metric grid (proves fidelity)
    assert cell == 2000.0                                                         # a metric grid (m)
    rin, rout = float(heights.max() - heights.min()), float(Z.max() - Z.min())
    assert 0.95 * rin <= rout <= rin * 1.001         # relief round-trips at fine sampling (bilinear ≤ source range)
    # and it round-trips through the sim bundle format -> read_dem_window reads the ingested non-polar map
    out = str(tmp_path / "ldem4_bundle")
    di.ingest_to_bundle(Z, cell, out, source=geom["source"])
    win, wcell = MP.read_dem_window(0, 0, Z.shape[0], Z.shape[1], bundle_dir=out)
    assert wcell == cell and np.allclose(win, Z, atol=1.0)                        # rf32 (float32) round-trip


# ---- P4: streaming a km-scale DEM without holding it in RAM -------------------------------------
def test_dem_grid_info_reads_without_loading_data():
    info = MP.dem_grid_info()
    assert info["width"] == 2000 and info["height"] == 2000 and info["cell_m"] == 5.0


def test_read_dem_window_matches_full_load_crop():
    import numpy as np
    Z, cell = MP.load_haworth_dem()
    win, wcell = MP.read_dem_window(800, 1600, 80, 80)
    assert wcell == cell and win.shape == (80, 80)
    assert np.array_equal(win, Z[800:880, 1600:1680])                # bit-exact vs the full-load crop
    # a window at the far corner proves random access (not a prefix read)
    far, _ = MP.read_dem_window(1980, 1980, 20, 20)
    assert np.array_equal(far, Z[1980:2000, 1980:2000])


def test_read_dem_window_holds_a_memory_ceiling():
    # streaming: reading a window allocates ~window bytes, NOT the full 2000^2 (32 MB float64) map
    import tracemalloc
    full_dem_bytes = 2000 * 2000 * 8
    tracemalloc.start()
    win, _ = MP.read_dem_window(500, 500, 128, 128)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    assert peak < full_dem_bytes // 8                                # comfortably under an eighth of the full map


def test_flattest_anchor_streamed_finds_buildable_site_within_memory_ceiling():
    import numpy as np
    import tracemalloc
    Z, cell = MP.load_haworth_dem()
    median_slope = float(np.median(MP.slope_deg_map(Z, cell)))
    tracemalloc.start()
    ax, ay = MP.flattest_anchor_streamed(window_m=20.0, tile=400)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    smap = MP.slope_deg_map(Z, cell)
    col, row = int(round(ax / cell)), int(round(ay / cell))
    assert smap[row, col] < median_slope                            # found a genuinely flat (buildable) site
    assert peak < 2000 * 2000 * 8                                   # never held the full DEM in RAM


# ---- I10: hazard + slope/slip-aware haul routing on a DEM costmap -------------------------------
def _haworth_routing_crop():
    Z, cell = MP.load_haworth_dem()
    return Z[800:880, 1600:1680].copy(), cell      # real window: 75% passable @25 deg, hazards to 61 deg


def test_slope_costmap_marks_steep_cells_impassable():
    import numpy as np
    crop, cell = _haworth_routing_crop()
    cost, passable = MP.slope_costmap(crop, cell, max_slope_deg=25.0)
    smap = MP.slope_deg_map(crop, cell)
    assert passable.any() and (~passable).any()                    # both buildable corridor and hazards
    assert np.array_equal(passable, smap <= 25.0)
    assert np.all(cost >= 1.0) and np.all(np.isfinite(cost[passable]))
    # cost rises monotonically with slope (slip penalty): steepest passable cell costs more than flattest
    pc = cost[passable]
    assert pc.max() > pc.min()


def test_route_least_cost_avoids_hazards_and_is_at_least_straight_line():
    import numpy as np
    crop, cell = _haworth_routing_crop()
    cost, passable = MP.slope_costmap(crop, cell, max_slope_deg=25.0)
    smap = MP.slope_deg_map(crop, cell)
    H, W = smap.shape
    left, right = smap[:, :W // 3], smap[:, 2 * W // 3:]
    sr = np.unravel_index(int(np.argmin(left)), left.shape)
    gr = np.unravel_index(int(np.argmin(right)), right.shape)
    gr = (gr[0], gr[1] + 2 * W // 3)
    path, length_m, reached = MP.route_least_cost(cost, passable, cell, sr, gr)
    assert reached and path[0] == sr and path[-1] == gr
    assert all(passable[r, c] for r, c in path)                    # routed AROUND every hazard cell
    straight = math.hypot((gr[1] - sr[1]) * cell, (gr[0] - sr[0]) * cell)
    assert length_m >= straight - 1e-6                             # detour is never shorter than the line


def test_route_least_cost_blocked_when_endpoint_impassable():
    import numpy as np
    crop, cell = _haworth_routing_crop()
    cost, passable = MP.slope_costmap(crop, cell, max_slope_deg=25.0)
    smap = MP.slope_deg_map(crop, cell)
    steep = np.unravel_index(int(np.argmax(smap)), smap.shape)     # a real hazard cell (>25 deg)
    flat = np.unravel_index(int(np.argmin(smap)), smap.shape)
    path, length_m, reached = MP.route_least_cost(cost, passable, cell, steep, flat)
    assert reached is False and math.isinf(length_m) and path == []


def test_routed_distance_detours_around_a_crater_on_real_haworth():
    # I10 end to end: between two buildable LOCAL sites on opposite sides of a hazardous window, the routed
    # haul distance exceeds the Euclidean line (it goes around the steep ground). Anchored to a DEM origin
    # (M11). Endpoints are the flattest cell in each third, so both are guaranteed passable real terrain.
    import numpy as np
    dem = MP.load_haworth_dem()
    Z, cell = dem
    r0, c0 = 800, 1600
    origin = (c0 * cell, r0 * cell)
    crop = Z[r0:r0 + 80, c0:c0 + 80]
    smap = MP.slope_deg_map(crop, cell)
    W = smap.shape[1]
    sl = np.unravel_index(int(np.argmin(smap[:, :W // 3])), smap[:, :W // 3].shape)
    gl = np.unravel_index(int(np.argmin(smap[:, 2 * W // 3:])), smap[:, 2 * W // 3:].shape)
    a_local = (sl[1] * cell, sl[0] * cell)                          # local meters within the window
    b_local = ((gl[1] + 2 * W // 3) * cell, gl[0] * cell)
    dist, grid_straight, reached = MP.routed_distance(dem, origin, a_local, b_local, max_slope_deg=25.0, margin_m=10.0)
    assert reached
    assert dist >= grid_straight - 1e-6                            # routed >= grid baseline (consistent)
    assert dist > grid_straight + 1.0                              # a real hazard detour, not a straight line


# ---- pluggable sequencer x objective (run different algorithms, sort by any metric) -------------
def _spread_mission():
    # several cuts + fills spread around the charger so the VISIT ORDER actually changes the metrics
    return MP.mission_from_dict({"name": "spread", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut A", "kind": "cut", "x": 120, "y": 0, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "cut B", "kind": "cut", "x": -110, "y": 10, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "fill C", "kind": "fill", "x": 0, "y": 130, "footprint_m2": 16, "depth_m": 0.05},
        {"action": "fill D", "kind": "fill", "x": 10, "y": -120, "footprint_m2": 16, "depth_m": 0.05},
        {"action": "fill E", "kind": "fill", "x": 140, "y": 30, "footprint_m2": 16, "depth_m": 0.05}]})


def test_optimize_sequence_each_algorithm_returns_a_valid_permutation():
    m = _spread_mission()
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    n = len(trips)
    assert n >= 3                                                   # the mission must actually have choices
    for algo in MP.SEQUENCERS:
        order = MP.optimize_sequence(trips, m, algorithm=algo, objective="distance")
        assert sorted(order) == list(range(n))                     # a permutation of all trips


def test_brute_force_is_at_least_as_good_as_nearest():
    m = _spread_mission()
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)

    def dist(order):
        return MP._simulate(m, [trips[i] for i in order])[2]["distance_m"]

    nn = dist(MP.optimize_sequence(trips, m, algorithm="nearest", objective="distance"))
    bf = dist(MP.optimize_sequence(trips, m, algorithm="brute", objective="distance"))
    two = dist(MP.optimize_sequence(trips, m, algorithm="two_opt", objective="distance"))
    assert bf <= nn + 1e-6 and two <= nn + 1e-6                     # exhaustive/local search >= the heuristic


def test_objective_choice_drives_the_optimum():
    # each objective's optimal plan is best in ITS metric -> the objective genuinely steers the optimizer
    m = _spread_mission()
    _, _, _, _, Td = MP.plan_and_simulate(m, algorithm="brute", objective="distance")
    _, _, _, _, Te = MP.plan_and_simulate(m, algorithm="brute", objective="energy")
    assert Td["distance_m"] <= Te["distance_m"] + 1e-6
    assert Te["energy_J"] <= Td["energy_J"] + 1e-6
    assert Td["algorithm"] == "brute" and Td["objective"] == "distance"


def test_compare_algorithms_sorts_by_objective():
    m = _spread_mission()
    res = MP.compare_algorithms(m, objective="distance")
    assert {r["algorithm"] for r in res["rows"]} == {a for a in MP.SEQUENCERS if a != "auto"}
    vals = [r["objective_value"] for r in res["rows"] if "objective_value" in r]
    assert vals == sorted(vals)                                    # ascending for a min objective (best first)
    assert any(r.get("pareto") for r in res["rows"] if "error" not in r)   # a frontier is marked


def test_multivehicle_is_gated_off_by_default():
    m = _spread_mission()
    try:
        MP.plan_and_simulate(m, vehicles=2)
    except RuntimeError as e:
        assert "multi-vehicle" in str(e)
    else:
        raise AssertionError("vehicles > 1 must raise the multi-vehicle gate (single-vehicle default)")


def test_unknown_algorithm_or_objective_raises():
    m = _spread_mission()
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    with pytest.raises(ValueError):
        MP.optimize_sequence(trips, m, algorithm="bogus", objective="time")
    with pytest.raises(ValueError):
        MP.optimize_sequence(trips, m, algorithm="nearest", objective="bogus")


def _pairs_mission(sites, precedence=None):
    # co-located cut+fill pairs -> one trip per site (a pure TSP/SOP over the sites)
    orders = []
    for i, (x, y) in enumerate(sites):
        orders += [{"action": f"cut{i}", "kind": "cut", "x": x, "y": y, "footprint_m2": 40, "depth_m": 0.05},
                   {"action": f"fill{i}", "kind": "fill", "x": x + 1, "y": y + 1, "footprint_m2": 40, "depth_m": 0.05}]
    p = {"name": "p", "body": "moon", "charger": [0, 0], "orders": orders}
    if precedence:
        p["precedence"] = precedence
    return MP.mission_from_dict(p)


def _dist(m, trips, order):
    return MP._simulate(m, [trips[i] for i in order])[2]["distance_m"]


def test_held_karp_finds_the_optimal_driving_tour():
    # Held-Karp's guarantee is the EXACT minimum additive driving tour (charger -> sites -> charger).
    # Verify against a direct min over all permutations of the pure tour length.
    import itertools
    import math as _m
    m = _pairs_mission([(40, 0), (-40, 3), (80, 0), (-80, 3), (0, 120)])    # 5 trips, NN-suboptimal
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    n = len(trips)
    pts = [tuple(m.charger)] + [tuple(t["site"]) for t in trips]
    dd = lambda a, b: _m.hypot(pts[a][0] - pts[b][0], pts[a][1] - pts[b][1])
    tour = lambda o: dd(0, o[0] + 1) + sum(dd(o[k] + 1, o[k + 1] + 1) for k in range(n - 1)) + dd(o[-1] + 1, 0)
    hk = MP.optimize_sequence(trips, m, algorithm="held_karp", objective="distance")
    true_min = min(itertools.permutations(range(n)), key=tour)
    assert abs(tour(hk) - tour(true_min)) < 1e-6                            # exact optimal driving tour


def test_held_karp_scales_past_brute_and_auto_dispatches():
    sites = [(40 * i - 220, (i % 3) * 55) for i in range(10)]               # 10 trips (> brute's 7 cap)
    m = _pairs_mission(sites)
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    assert 8 <= len(trips) <= 16
    hk = MP.optimize_sequence(trips, m, algorithm="held_karp", objective="distance")
    assert sorted(hk) == list(range(len(trips)))
    nn = MP.optimize_sequence(trips, m, algorithm="nearest", objective="distance")
    assert _dist(m, trips, hk) <= _dist(m, trips, nn) + 1e-6
    _, _, _, _, Ta = MP.plan_and_simulate(m, algorithm="auto", objective="distance")
    _, _, _, _, Thk = MP.plan_and_simulate(m, algorithm="held_karp", objective="distance")
    assert Ta["resolved_algorithm"] == "held_karp_lk"                       # 8-16 trips -> HK seed + LK polish
    assert Ta["distance_m"] <= Thk["distance_m"] + 1e-6                     # the LK polish never hurts


def test_or_opt_and_lk_are_valid_and_no_worse_than_nearest():
    m = _pairs_mission([(40, 0), (-40, 3), (80, 0), (-80, 3), (0, 120)])
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    nn = _dist(m, trips, MP.optimize_sequence(trips, m, algorithm="nearest", objective="distance"))
    for a in ("or_opt", "lk"):
        o = MP.optimize_sequence(trips, m, algorithm=a, objective="distance")
        assert sorted(o) == list(range(len(trips))) and _dist(m, trips, o) <= nn + 1e-6


def test_auto_dispatch_picks_brute_for_small():
    _, _, _, _, T = MP.plan_and_simulate(_pairs_mission([(40, 0), (-40, 3), (80, 0)]), algorithm="auto")
    assert T["resolved_algorithm"] == "brute"                               # <= 7 trips -> exact brute


def test_precedence_is_respected_by_every_algorithm():
    # fill3 must precede fill0 (a constraint the unconstrained optimum would violate)
    m = _pairs_mission([(40, 0), (-40, 3), (80, 0), (-80, 3)], precedence=[["fill3", "fill0"]])
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)
    prec = MP.trip_precedence(trips, m)
    assert prec                                                             # the constraint lifted to trips
    pred = MP._prec_masks(len(trips), prec)
    for a in ("nearest", "greedy", "two_opt", "or_opt", "lk", "brute", "held_karp", "auto"):
        order = MP.optimize_sequence(trips, m, algorithm=a, objective="distance", precedence=prec)
        assert sorted(order) == list(range(len(trips)))
        assert MP._respects(order, pred), f"{a} violated precedence"


def test_weighted_objective_parses_and_runs():
    assert MP.parse_objective("time") == {"time": 1.0}
    w = MP.parse_objective("time:3,energy:1")
    assert abs(w["time"] - 0.75) < 1e-9 and abs(w["energy"] - 0.25) < 1e-9
    _, _, _, _, T = MP.plan_and_simulate(_spread_mission(), algorithm="two_opt", objective="time:0.5,energy:0.5")
    assert T["objective"] == "time:0.5,energy:0.5"
    with pytest.raises(ValueError):
        MP.parse_objective("time:1,bogus:1")


def test_compare_with_weighted_objective_marks_pareto():
    res = MP.compare_algorithms(_spread_mission(), objective="time:0.5,distance:0.5")
    vals = [r["objective_value"] for r in res["rows"] if "objective_value" in r]
    assert vals == sorted(vals)                                             # weighted score, best first
    assert any(r.get("pareto") for r in res["rows"] if "error" not in r)


# ---- I10 energy: exact gravity lift for uphill hauls -------------------------------------------
def test_haul_energy_is_slip_adjusted_when_the_haul_climbs():
    # #1 slip-loss: a cut->fill haul over sloped ground costs more than the flat 135 J/m (the wheel travels
    # 1/(1-slip) per metre). Place the cut at a low cell and the fill at a high cell so the haul has slope.
    import numpy as np
    dem = MP.load_haworth_dem()
    Z, cell = dem
    win = Z[0:40, 0:40]
    lo = np.unravel_index(int(np.argmin(win)), win.shape)
    hi = np.unravel_index(int(np.argmax(win)), win.shape)
    m = MP.mission_from_dict({"name": "h", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": lo[1] * cell, "y": lo[0] * cell, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "fill", "kind": "fill", "x": hi[1] * cell, "y": hi[0] * cell, "footprint_m2": 16, "depth_m": 0.05}]})
    trips, _, _, _ = MP._build_trips(m, dem, (0.0, 0.0), 25.0)
    cf = next(t for t in trips if t["kind"] == "cutfill")
    assert cf["haul_e"] > cf["haul_m"] * MP.DRIVE_J_PER_M     # slip raises the haul energy above flat


def test_no_dem_haul_energy_is_flat_135():
    m = MP.mission_from_dict({"name": "f", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04},
        {"action": "fill", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10}]})
    trips, _, _, _ = MP._build_trips(m, None, (0.0, 0.0), 25.0)          # no DEM -> no slope -> flat
    cf = next(t for t in trips if t["kind"] == "cutfill")
    assert math.isclose(cf["haul_e"], cf["haul_m"] * MP.DRIVE_J_PER_M)


def test_dem_plan_energy_at_least_the_flat_plan():
    # integration: the slip+lift+routing-aware DEM plan never costs less than the flat plan
    m = MP.mission_from_dict({"name": "i", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "Borrow pit", "kind": "cut", "x": -120, "y": -90, "footprint_m2": 60, "depth_m": 0.08},
        {"action": "Landing pad", "kind": "fill", "x": 140, "y": 110, "footprint_m2": 40, "depth_m": 0.10}]})
    dem = MP.load_haworth_dem(); o = MP.flattest_anchor(dem)
    _, _, _, _, t_flat = MP.plan_and_simulate(m)
    _, _, _, _, t_dem = MP.plan_and_simulate(m, dem=dem, dem_origin=o)
    assert t_dem["energy_J"] >= t_flat["energy_J"] - 1e-6


def test_uphill_haul_adds_exact_gravity_lift_energy():
    import numpy as np
    dem = MP.load_haworth_dem()
    Z, cell = dem
    win = Z[0:40, 0:40]                                   # a small real-Haworth window with relief
    lo = np.unravel_index(int(np.argmin(win)), win.shape)
    hi = np.unravel_index(int(np.argmax(win)), win.shape)
    dh = float(Z[hi] - Z[lo])
    assert dh > 0.0                                       # the window has real relief
    cut_lo = {"action": "cut", "kind": "cut", "x": lo[1] * cell, "y": lo[0] * cell, "footprint_m2": 40, "depth_m": 0.05}
    fill_hi = {"action": "fill", "kind": "fill", "x": hi[1] * cell, "y": hi[0] * cell, "footprint_m2": 16, "depth_m": 0.05}
    m_up = MP.mission_from_dict({"name": "up", "body": "moon", "charger": [0, 0], "orders": [cut_lo, fill_hi]})
    _, _, _, _, t_up = MP.plan_and_simulate(m_up, dem=dem)        # haul cut(low) -> fill(high) = uphill
    _, _, _, _, t_flat = MP.plan_and_simulate(m_up)               # no DEM -> no lift term
    assert t_flat["lift_energy_J"] == 0.0
    # exact: lift = hauled regolith mass * g * dh
    g = MP.body_gravity("moon")
    flows, _ = MP.balance(m_up)
    hauled = sum(mass for co, fo, mass, d in flows if co is not None)
    assert t_up["lift_energy_J"] > 0.0
    assert abs(t_up["lift_energy_J"] - hauled * g * dh) < 1.0
    # downhill (swap): hauling cut(high) -> fill(low) does no positive lift
    m_dn = MP.mission_from_dict({"name": "dn", "body": "moon", "charger": [0, 0], "orders": [
        {**cut_lo, "x": hi[1] * cell, "y": hi[0] * cell}, {**fill_hi, "x": lo[1] * cell, "y": lo[0] * cell}]})
    _, _, _, _, t_dn = MP.plan_and_simulate(m_dn, dem=dem)
    assert t_dn["lift_energy_J"] == 0.0


# ---- endurance / single-charge range ("true distance before recharge") -------------------------
def _tiny_mission():
    return MP.mission_from_dict({"name": "e", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "c", "kind": "cut", "x": 1, "y": 1, "footprint_m2": 9, "depth_m": 0.02}]})


def test_endurance_flat_range_is_grounded():
    e = MP.endurance(_tiny_mission())
    # ~4.795 MJ / 134.6 J/m -> ~35.6 km full pack, ~32 km to the 10% reserve
    assert 34.0 < e["range_flat_full_km"] < 37.0
    assert 30.0 < e["range_flat_reserve_km"] < 34.0
    assert e["range_flat_reserve_km"] < e["range_flat_full_km"]   # reserve cuts range
    assert e["duration_flat_h"] > 20.0


def test_endurance_slope_slip_and_dem_reach():
    e = MP.endurance(_tiny_mission(), dem=MP.load_haworth_dem(), dem_origin=(5000.0, 5000.0))
    assert e["range_slopeslip_km"] <= e["range_flat_reserve_km"] + 1e-6   # slope+slip never raises range
    assert e["work_area_median_slope_deg"] > 0.0
    r = e["reach"]
    assert r["radius_m"] > 0.0 and r["tile_fully_reachable"] is True       # 32 km range >> 10 km Haworth tile
    assert 0.0 < r["worst_cell_pack_frac"] < 1.0                           # crossing the tile costs < a full pack


def test_power_regime_psr_tower_vs_sunlit_solar():
    # #2 power model: a PSR (e.g. Haworth) has NO sun -> lander/tower power, available anytime, duty 1.0.
    # A sunlit site recharges from solar, duty-limited to the body's daylight fraction (< 1).
    m = _tiny_mission()
    psr = MP.power_regime(m, kind="psr_tower")
    assert psr["duty_frac"] == 1.0 and psr["effective_charge_w"] == psr["charge_power_w"]
    assert "tower" in psr["availability"].lower() or "lander" in psr["availability"].lower()
    sun = MP.power_regime(m, kind="sunlit_solar")
    assert 0.0 < sun["duty_frac"] < 1.0                      # only the daylight fraction of the body-day
    assert sun["effective_charge_w"] < sun["charge_power_w"]
    with pytest.raises(ValueError):
        MP.power_regime(m, kind="bogus")


def test_thermal_derating_cold_cuts_usable_pack():
    assert MP.thermal_derate(None) == 1.0 and MP.thermal_derate(25.0) == 1.0   # warm -> full pack
    assert MP.thermal_derate(-35.0) < 1.0 and MP.thermal_derate(-200.0) >= 0.5  # cold derates, floored
    m = _tiny_mission()
    cold = MP.power_regime(m, kind="psr_tower", temp_c=-35.0)
    assert cold["usable_pack_J"] < MP.BATTERY_J                # cold reduces usable capacity


def test_endurance_carries_power_regime():
    e = MP.endurance(_tiny_mission())                          # default = PSR tower (the Haworth demo site)
    assert e["power"]["kind"] == "psr_tower" and e["power"]["duty_frac"] == 1.0


def test_endurance_conops_reconciliation_drums_dominate():
    c = MP.endurance(_tiny_mission())["conops"]
    assert c["traverse_km"] == 70.0 and c["mission_days"] == 11.0          # SCHULER24 ConOps
    assert 1.5 < c["drive_packs"] < 2.5                                    # 70 km drive ~ 2 packs
    assert c["dig_packs"][0] > c["drive_packs"]                            # digging 5-10 t > driving
    assert c["drums_dominate"] is True


def test_timescale_is_body_dependent():
    # Moon: a 30 h sortie fits easily in the ~9-11 day sun window; Mars: it spans multiple sols
    moon = MP.endurance(_tiny_mission())["timescale"]
    assert moon["day_label"] == "lunar day" and moon["solar_day_h"] > 700
    assert moon["fits_in_window"] is True and moon["sorties_per_window"] > 5
    mars_m = MP.mission_from_dict({"name": "m", "body": "mars", "charger": [0, 0],
                                   "orders": [{"action": "c", "kind": "cut", "x": 1, "y": 1, "footprint_m2": 9, "depth_m": 0.02}]})
    mars = MP.endurance(mars_m)["timescale"]
    assert mars["day_label"] == "sol" and mars["solar_day_h"] < 25
    assert mars["fits_in_window"] is False and mars["spans_days"] > 1.0    # 30 h drive spans >1 sol


def test_single_charge_range_monotone_in_slope_and_slip():
    g = MP.body_gravity("moon")
    flat = MP.single_charge_range_m(g)
    assert MP.single_charge_range_m(g, slope_deg=20.0) < flat              # climbing shortens range
    assert MP.single_charge_range_m(g, slip=0.3) < flat                    # slip shortens range
    assert MP.single_charge_range_m(g, full_pack=True) > flat              # full pack > to-reserve


# ---- P5: execute + watch — animatable timeline -------------------------------------------------
def test_build_timeline_is_animatable():
    m = MP.demo_mission()
    tlj = MP.build_timeline(m)
    fr = tlj["frames"]
    assert tlj["duration_s"] > 0 and tlj["battery_J"] > 0 and len(fr) >= len(m.orders)
    assert (fr[0]["x0"], fr[0]["y0"]) == tuple(m.charger)              # starts parked at the charger
    assert (fr[-1]["x1"], fr[-1]["y1"]) == tuple(m.charger)            # returns to the charger
    assert fr[0]["t0"] == 0.0
    assert all(abs(fr[i]["t1"] - fr[i + 1]["t0"]) < 1e-6 for i in range(len(fr) - 1))   # contiguous time
    assert all(-1e-9 <= f["batt0_frac"] <= 1 + 1e-6 and -1e-9 <= f["batt1_frac"] <= 1 + 1e-6 for f in fr)
    cum = [f["cum_mass_kg"] for f in fr]
    assert all(cum[i] <= cum[i + 1] + 1e-6 for i in range(len(cum) - 1)) and cum[-1] > 0   # mass monotonic
    assert {f["phase"] for f in fr} >= {"drive", "dig"}               # the rover drives and digs
    assert any(f["phase"] == "charge" for f in fr)                    # the demo needs mid-mission recharges


def test_build_timeline_routes_with_dem():
    # P5 + I10: with a DEM the timeline's hauls reflect the routed plan (still a valid animatable timeline)
    m = MP.demo_mission()
    dem = MP.load_haworth_dem()
    tlj = MP.build_timeline(m, dem=dem, dem_origin=MP.flattest_anchor(dem))
    assert tlj["duration_s"] > 0 and tlj["frames"][-1]["t1"] == tlj["frames"][-1]["t1"]   # finite, no NaN
    assert (tlj["frames"][-1]["x1"], tlj["frames"][-1]["y1"]) == tuple(m.charger)


# ---- /plan server endpoint (real socket: drive the app) -----------------------------------------
@pytest.fixture()
def base():
    srv = SRV.make_server(0)                                # ephemeral port
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        yield "http://127.0.0.1:%d" % srv.server_address[1]
    finally:
        srv.shutdown()


def _post(base, route, obj):
    req = urllib.request.Request(base + route, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_plan_endpoint_returns_fetchable_pdf(base):
    code, body = _post(base, "/plan", _payload())
    assert code == 200 and body["ok"] is True
    assert body["pdf"].startswith("/reports/") and body["totals"]["cut_kg"] > 0
    with urllib.request.urlopen(base + body["pdf"], timeout=30) as r:   # the report is actually served back
        assert r.status == 200 and r.read(5) == b"%PDF-"
    assert body["validation"]["mass_conserved"] is True                 # I8: plan validated on the authority


def test_plan_endpoint_includes_autonomy_and_perception(base):
    # the closed-loop autonomy + the AutoNav onboard-estimate (perception) uncertainty, folded into /plan
    code, body = _post(base, "/plan", _payload())
    assert code == 200 and body["ok"] is True
    au = body["autonomy"]
    assert au is not None and au["recharges"] >= 0 and au["replans"] >= 0
    assert 0.0 <= au["final_soc"] <= 1.0 and isinstance(au["completed"], bool)
    pc = body["perception"]
    assert pc is not None and pc["pose_sigma_m"] >= 0.0 and pc["drum_fill_uncertainty_pct"] > 0.0


def test_plan_endpoint_returns_animatable_timeline(base):
    # P5: /plan returns the execute+watch timeline the browser animates (frames + duration)
    code, body = _post(base, "/plan", _payload())
    tl = body["timeline"]
    assert code == 200 and tl["duration_s"] > 0 and len(tl["frames"]) >= 1
    f0 = tl["frames"][0]
    assert {"t0", "t1", "x0", "y0", "x1", "y1", "phase", "batt0_frac", "cum_mass_kg"} <= set(f0)


def test_plan_endpoint_moon_slope_gates_on_real_dem(base):
    # I6/M11 wired live: a Moon plan is validated against the real Haworth DEM, anchored to the
    # auto-selected flattest site -> the validation carries the slope-gate fields (not the flat check).
    code, body = _post(base, "/plan", _payload())
    v = body["validation"]
    assert code == 200
    assert v["max_slope_deg"] == 15.0 and isinstance(v["slope_violations"], list)


def test_plan_endpoint_honors_algorithm_and_objective(base):
    code, body = _post(base, "/plan", _payload(orders=[
        {"action": "cut A", "kind": "cut", "x": 120, "y": 0, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "cut B", "kind": "cut", "x": -110, "y": 10, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "fill C", "kind": "fill", "x": 0, "y": 130, "footprint_m2": 16, "depth_m": 0.05},
        {"action": "fill D", "kind": "fill", "x": 10, "y": -120, "footprint_m2": 16, "depth_m": 0.05}],
        ) | {"algorithm": "brute", "objective": "energy"})
    assert code == 200 and body["ok"] is True
    assert body["totals"]["algorithm"] == "brute" and body["totals"]["objective"] == "energy"


def test_compare_endpoint_returns_sorted_rows(base):
    payload = _payload(orders=[
        {"action": "cut A", "kind": "cut", "x": 120, "y": 0, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "cut B", "kind": "cut", "x": -110, "y": 10, "footprint_m2": 40, "depth_m": 0.05},
        {"action": "fill C", "kind": "fill", "x": 0, "y": 130, "footprint_m2": 16, "depth_m": 0.05},
        {"action": "fill D", "kind": "fill", "x": 10, "y": -120, "footprint_m2": 16, "depth_m": 0.05}])
    code, body = _post(base, "/compare", payload | {"objective": "distance"})
    assert code == 200 and body["ok"] is True and body["objective"] == "distance"
    vals = [r["objective_value"] for r in body["rows"] if "objective_value" in r]
    assert vals == sorted(vals) and len(vals) >= 2


def test_plan_endpoint_rejects_sinter(base):
    code, body = _post(base, "/plan", _payload(orders=[
        {"action": "Sinter", "kind": "sinter", "x": 1, "y": 1, "footprint_m2": 9, "depth_m": 0.01}]))
    assert code == 400 and body["ok"] is False and "GATED OFF" in body["error"]


def test_plan_endpoint_rejects_unknown_body(base):
    code, body = _post(base, "/plan", _payload(body="pluto"))
    assert code == 400 and "body" in body["error"]


def test_plan_endpoint_rejects_bad_json(base):
    req = urllib.request.Request(base + "/plan", data=b"{not json", method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=30)
        assert False, "expected 400"
    except urllib.error.HTTPError as e:
        assert e.code == 400


def test_sense_endpoint_noise_off_and_on(base):
    # noise OFF (default): inference is faithful + deterministic
    code, a = _post(base, "/sense", {"true_mass_kg": 25.0})
    assert code == 200 and a["ok"] is True
    assert abs(a["inferred_kg"] - 25.0) < 0.5 and a["noise_frac"] == 0.0
    assert a["offload"] is False and a["current_a"] > 0
    _, a2 = _post(base, "/sense", {"true_mass_kg": 25.0})
    assert a2["inferred_kg"] == a["inferred_kg"]                          # deterministic when noise off
    # near capacity -> offload fires, and fill is in the best-known (>half full) regime
    _, full = _post(base, "/sense", {"true_mass_kg": 30.0})
    assert full["offload"] is True and full["uncertainty_frac"] < 0.03
    # noise ON: reading is perturbed (seeded server-side)
    _, n = _post(base, "/sense", {"true_mass_kg": 25.0, "noise_frac": 0.15, "seed": 3})
    assert n["inferred_kg"] != a["inferred_kg"]


def test_sense_endpoint_rejects_bad_input(base):
    code, body = _post(base, "/sense", {"nope": 1})
    assert code == 400 and "true_mass_kg" in body["error"]


def test_structure_endpoint_returns_plannable_orders(base):
    code, body = _post(base, "/structure", {"name": "landing_pad", "x": 40, "y": 30})
    assert code == 200 and body["ok"] is True
    assert len(body["orders"]) == 2 and sorted(o["kind"] for o in body["orders"]) == ["cut", "fill"]
    # the structure's orders must plan through /plan unchanged
    code2, plan = _post(base, "/plan", {"name": "Pad", "body": "moon", "orders": body["orders"]})
    assert code2 == 200 and plan["ok"] is True and plan["totals"]["cut_kg"] > 0


def test_structure_endpoint_rejects_unknown(base):
    code, body = _post(base, "/structure", {"name": "death_star", "x": 0, "y": 0})
    assert code == 400 and body["ok"] is False and "death_star" in body["error"]


def test_static_index_and_bodies_served(base):
    with urllib.request.urlopen(base + "/", timeout=30) as r:
        assert r.status == 200 and b"<" in r.read(2048)    # serves some HTML
    with urllib.request.urlopen(base + "/bodies.json", timeout=30) as r:
        d = json.loads(r.read())
        assert "moon" in d and "_ipex" in d                # the py-generated bodies + ipex mirror
