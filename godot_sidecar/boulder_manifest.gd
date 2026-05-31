extends RefCounted
class_name BoulderManifest
# OWNER LANE: A2-sweep (per-frame boulder manifest; companion to sun_sweep.gd).
# NO-OP skeleton landed in the L0 contracts-first pass. The A2-sweep lane fills this
# in; it NEVER edits sidecar.gd.
#
# Contract (FROZEN docs/sun_sweep_manifest.md, sun_sweep/1.0):
#   boulders:[{id, center_m, radius_m, world_pos:[x,y,z], quaternion_xyzw:[x,y,z,w],
#   buried_frac, shadow:{azimuth_deg,length_m}|null}]. The boulder source-of-truth is
#   the scene metadata.json clasts[]; boulder poses reuse SensorsEmit.pose_dict
#   (Godot frame; the REP-103 conversion stays C1's job). The per-frame shadow
#   azimuth/length follow from the swept sun (sun_sweep.gd).

# Stub entry point (signature TBD by the A2-sweep lane). Logs + returns an empty
# boulder list so a caller can compose a manifest without crashing pre-implementation.
static func build_boulders(_sf) -> Array:
	print("boulder_manifest: stub (A2-sweep lane)")
	return []
