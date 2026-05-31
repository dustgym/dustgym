extends RefCounted
class_name SunSweep
# OWNER LANE: A2-sweep (sun-elevation/azimuth sweep + per-frame boulder manifest).
# NO-OP skeleton landed in the L0 contracts-first pass. The A2-sweep lane fills this
# in; it NEVER edits sidecar.gd (the --sun-sweep flag + dispatch call-site are
# already wired in sidecar.gd by L0).
#
# Contract (FROZEN docs/sun_sweep_manifest.md, sun_sweep/1.0):
#   out/sun_sweep/<scene>/manifest.json + per-frame images, sweeping the lunar-day
#   sun across the grazing polar band (azimuth advances 360deg per synodic period;
#   elevation oscillates in the grazing band per the documented assumption). Each
#   frame records sun {azimuth_deg, elevation_deg} + time_delta_s, and the boulder
#   poses come from SunSweep -> BoulderManifest, reusing SensorsEmit.pose_dict for
#   the Godot-frame poses (REP-103 stays C1's job).
#
# This lane drives the sidecar's existing _sun_elev_deg / _sun_azim_deg per frame
# and re-renders; the per-frame "sun" block emitted into any sensors.json comes from
# the shared SensorsEmit.sun_block(...) so the sweep + the egress agree.

# Stub entry point (signature TBD by the A2-sweep lane). Logs + returns.
static func run_sun_sweep(_sidecar: Node) -> void:
	print("sun_sweep: stub (A2-sweep lane)")
