extends RefCounted
class_name BoulderManifest
# OWNER LANE: A2-sweep (per-frame boulder manifest; companion to sun_sweep.gd).
# Fills the L0 NO-OP skeleton. NEVER edits sidecar.gd / sensors_emit.gd / state_fields.gd.
#
# Contract (FROZEN docs/sun_sweep_manifest.md, sun_sweep/1.0):
#   boulders:[{id, center_m, radius_m, world_pos:[x,y,z], quaternion_xyzw:[x,y,z,w],
#   buried_frac, shadow:{azimuth_deg,length_m}|null}]. The boulder source-of-truth is
#   the scene metadata.json clasts[] (loaded into sf.clasts, state_fields.gd:160); the
#   manifest COPIES id/center_m/radius_m/buried_frac VERBATIM — it never recomputes or
#   re-places a clast. Poses go through the FROZEN SensorsEmit.pose_dict (Godot frame;
#   REP-103 conversion stays C1's job, frames.py). The per-frame shadow azimuth/length
#   (§6) follow from the swept sun, so build_boulders takes the frame's sun az/el — the
#   boulder set is constant across frames, only the light (and thus the shadow) moves.

# Shadow-length clamp (manifest §6). As elevation -> 0 the first-order flat-plane shadow
# length h_exposed / tan(elev) diverges; cap it at the scene's ground extent so a grazing
# shadow is "as long as the field" rather than +inf. field_extent_m is the larger of the
# scene's two world spans (passed by sun_sweep.gd from sf.extent_m()); fall back to a
# sane lunar-clast-patch span if it is non-positive.
const SHADOW_LEN_FALLBACK_M := 8.0

# Build the per-frame boulders[] array for the sun-sweep manifest.
#   sf              : the loaded StateFields (sf.clasts is the source-of-truth, §4)
#   pose_dict_fn    : Callable -> FROZEN SensorsEmit.pose_dict (xf -> {position_m, quaternion_xyzw})
#   sun_azimuth_deg : this frame's sun azimuth [deg]   (sun_sweep.gd model, §3)
#   sun_elev_deg    : this frame's sun elevation [deg] (sun_sweep.gd model, §3)
#   field_extent_m  : larger scene world span [m], for the grazing-shadow length clamp (§6)
static func build_boulders(sf, pose_dict_fn: Callable, sun_azimuth_deg: float, sun_elev_deg: float, field_extent_m: float) -> Array:
	if sf == null:
		push_error("boulder_manifest: build_boulders requires a loaded scene (sf == null)")
		return []

	var clasts: Array = sf.clasts            # state_fields.gd:160 (scene metadata clasts[])
	var clamp_len: float = field_extent_m if field_extent_m > 0.0 else SHADOW_LEN_FALLBACK_M

	# Shadow points OPPOSITE the sun azimuth on a flat local plane (§6), constant per frame.
	var shadow_az: float = fmod(sun_azimuth_deg + 180.0, 360.0)
	# A shadow is resolvable only when the sun is above the local horizon (§6 null rule).
	var sun_up: bool = sun_elev_deg > 0.0
	var tan_elev: float = tan(deg_to_rad(sun_elev_deg)) if sun_up else 0.0

	var out: Array = []
	for c in clasts:
		# COPY VERBATIM from the scene metadata — never recompute or re-place (§4).
		var cid: int = int(c.get("id", -1))
		var ctr = c.get("center_m", [0.0, 0.0, 0.0])
		var rad: float = float(c.get("radius_m", 0.0))
		var buried: float = float(c.get("buried_frac", 0.0))

		# Pose (§5): for the current shape:"sphere" clasts, world_pos == center_m and the
		# orientation is identity. Build the boulder transform from center_m and serialize
		# it through the SAME FROZEN pose_dict every other sidecar pose uses, so world_pos /
		# quaternion_xyzw are byte-consistent with the rest of the bridge.
		var center := Vector3(float(ctr[0]), float(ctr[1]), float(ctr[2]))
		var xf := Transform3D(Basis.IDENTITY, center)
		var pose: Dictionary = pose_dict_fn.call(xf)

		# Shadow (§6): null when nothing is exposed (buried_frac >= 1.0) or the sun is at/
		# below the local horizon (elev <= 0). Otherwise a first-order flat-plane estimate:
		#   h_exposed = 2*radius_m*(1 - buried_frac)              (cap of the sphere above grade)
		#   length_m  = h_exposed / tan(elevation_deg)            (clamped at the field extent)
		var shadow = null
		if sun_up and buried < 1.0:
			var h_exposed: float = 2.0 * rad * (1.0 - buried)
			var length_m: float = h_exposed / tan_elev if tan_elev > 0.0 else clamp_len
			length_m = minf(length_m, clamp_len)
			shadow = {
				"azimuth_deg": shadow_az,
				"length_m": length_m,
			}

		out.append({
			"id": cid,
			"center_m": [center.x, center.y, center.z],   # copied (Godot frame, §4)
			"radius_m": rad,
			"world_pos": pose["position_m"],              # §5: == center_m for spheres
			"quaternion_xyzw": pose["quaternion_xyzw"],   # §5: identity for spheres
			"buried_frac": buried,
			"shadow": shadow,                             # §6: per-frame estimate or null
		})

	return out
