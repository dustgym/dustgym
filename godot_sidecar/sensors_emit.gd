extends RefCounted
class_name SensorsEmit
# THE shared schema-assembly sink for the sensor-bridge egress
# (docs/sensor_bridge_contract.md §2; FROZEN CONTRACT v1.1). MOVED here out of
# sidecar.gd in the L0 contracts-first pass so the four Wave-1 feature lanes that
# each extend the schema (M2-egress multi-frame, A2-sweep, M3-tag faces[], M2-slam)
# never collide on sidecar.gd. After L0, sidecar.gd holds ONE delegating call-site
# per render path; the schema assembly + the procedural lander build live HERE and
# are NOT edited by any feature lane (each lane feeds parameters in, never edits the
# sink).
#
# Structured like camera_rig.gd: a RefCounted with `class_name` + STATIC methods,
# preloaded by sidecar.gd. camera_rig.gd is NOT touched (sole owner = M3-cam); this
# module reads the camera list it is handed, generically.
#
# FRAME: every pose emitted is 100% GODOT-frame (contract §3: the REP-103 conversion
# is C1's job in frames.py, NOT here). We do NOT pre-compose any camera->tag truth
# transform (C1 computes it from the exact poses we emit).
#
# BEHAVIOR-PRESERVING for the existing --cameras path: a no-new-flags --cameras
# render produces a sensors.json byte-for-byte identical to the prior v1.0 output
# EXCEPT (a) schema_version '1.0' -> '1.1' and (b) the additive top-level "sun"
# block. Same field order otherwise.

# --- v1.1 frozen constants -------------------------------------------------------
const SCHEMA_VERSION := "sensor_bridge/1.1"
# AprilTag id-0 size (contract §1; the side length of the tag's black-border square).
# Mirrors sidecar.gd::APRILTAG_SIZE_M; kept here so the lander build owns its tag spec.
const APRILTAG_SIZE_M := 0.150
# Lander placement ahead of the rover (contract §1/§5). Mirrors sidecar.gd default.
const LANDER_STANDOFF_M := 2.5

# ---------------------------------------------------------------------------
# Transform3D -> {position_m:[x,y,z], quaternion_xyzw:[x,y,z,w]} (Godot frame).
# Static so the sweep lane (boulder poses) can reuse it directly off this sink.
static func pose_dict(xf: Transform3D) -> Dictionary:
	var p := xf.origin
	var q := xf.basis.get_rotation_quaternion()
	return {
		"position_m": [p.x, p.y, p.z],
		"quaternion_xyzw": [q.x, q.y, q.z, q.w],
	}

# ---------------------------------------------------------------------------
# Build the procedural CC0 lander: a BoxMesh body on 4 cylinder legs (plain grey
# StandardMaterial), with the apriltag id-0 tag quad on its ROVER-FACING vertical
# face. The lander frame ORIGIN coincides with the TAG CENTER (contract §1 M1
# simplification), and the lander +X axis = the tag outward normal (pointing back
# toward the rover). So apriltag.pose_in_lander is identity.
#
# Placement: `standoff` metres ahead of the rover along its forward (+X yawed)
# direction `fwd`, on the local surface, so both front cameras see the tag.
# Returns the lander root (its global_transform == the lander/tag pose we report).
#
# This is the verbatim move of sidecar.gd::_build_lander, parameterized on the
# pieces sidecar previously read off members (standoff, yaw, scripts), so the
# behavior is unchanged. `sf` is the StateFields instance (surface height lookup),
# `parent` is the sidecar root the lander is added under, and `apriltag_gen` is the
# preloaded AprilTagGen script (camera_rig-style script injection keeps this module
# free of res:// preloads the sidecar already owns).
#
# v1.1 faces[] (M3-tag lane) is NOT built here: M3-tag owns lander_bundle.gd and the
# per-face quad orientation (contract §6). This M1 single-face build keeps the front
# face (id 0) at the identity pose_in_lander so the M1 reading stays correct.
static func build_lander(parent: Node, sf, apriltag_gen, rover_pos: Vector3, fwd: Vector3,
		standoff_arg: float, lander_yaw_deg: float) -> Node3D:
	# Ground position ahead of the rover; snap to the surface height there.
	var standoff: float = standoff_arg if standoff_arg > 0.0 else LANDER_STANDOFF_M
	var ground := rover_pos + fwd * standoff
	var u: float = clampf((ground.x - sf.world_min.x) / maxf(sf.extent_m().x, 1e-6), 0.0, 1.0)
	var v: float = clampf((ground.z - sf.world_min.y) / maxf(sf.extent_m().y, 1e-6), 0.0, 1.0)
	var surf_y: float = sf.height_uv(u, v)

	# Lander root frame: ORIGIN = the tag center (per §1), placed at tag height on
	# the rover-facing face. +X (lander) = the tag outward normal = back toward the
	# rover = -fwd. We build a basis whose +X column = -fwd, +Y = up.
	var nx := (-fwd).normalized()                 # lander +X = tag normal (toward rover)
	var ny := Vector3(0, 1, 0)                     # lander +Y = up
	var nz := nx.cross(ny).normalized()            # lander +Z (right-handed)
	ny = nz.cross(nx).normalized()
	var tag_h := surf_y + 0.45                      # [CALIB] tag center height (mast-eye level)
	var lander_basis := Basis(nx, ny, nz)
	# Optional yaw of the whole lander/tag about world +Y so the tag face is OFF-SQUARE
	# to the camera ray -> oblique fiducial views (--lander-yaw; for the angle sweep).
	if absf(lander_yaw_deg) > 1e-3:
		lander_basis = Basis(Vector3(0, 1, 0), deg_to_rad(lander_yaw_deg)) * lander_basis

	var root := Node3D.new()
	root.name = "Lander"
	# The lander ROOT origin == the tag center (§1). The lander body sits BEHIND the
	# tag face (along -X, away from the rover) so the box does not occlude the tag.
	root.transform = Transform3D(lander_basis, Vector3(ground.x, tag_h, ground.z))
	parent.add_child(root)

	var grey := StandardMaterial3D.new()
	grey.albedo_color = Color(0.55, 0.56, 0.58)
	grey.metallic = 0.2
	grey.roughness = 0.7

	# Local-Y reference points (the lander origin / tag center is local y = 0):
	#   surface (foot)   = surf_y - tag_h   (negative; tag sits tag_h above ground)
	#   body base        = a bit below the tag so the tag reads on the lower-front face
	var foot_y := surf_y - tag_h                  # surface plane, local y
	var body_size := Vector3(0.55, 0.6, 0.9)      # [CALIB] x(depth) y(height) z(width)
	# Body sits BEHIND the tag (local -X) so it never occludes it, and is raised so
	# its base clears the surface (legs span the gap). Body center put just above the
	# tag center so the tag is on the lower portion of the rover-facing face.
	var body_center_y := 0.15                     # [CALIB] body center local y (tag at y=0)
	var body := MeshInstance3D.new()
	body.name = "lander_body"
	var box := BoxMesh.new()
	box.size = body_size
	box.material = grey
	body.mesh = box
	# Front face pulled 2 cm BEHIND the tag plane (local x = -0.02) so the matte tag
	# sits proud of the body and never z-fights with it; tag center stays at x=0.
	body.position = Vector3(-body_size.x * 0.5 - 0.02, body_center_y, 0.0)
	root.add_child(body)

	# 4 cylinder legs from the body base corners down to the surface (local frame).
	var body_base_y := body_center_y - body_size.y * 0.5
	var leg_height: float = maxf(body_base_y - foot_y, 0.25)
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			var leg := MeshInstance3D.new()
			var cyl := CylinderMesh.new()
			cyl.top_radius = 0.03
			cyl.bottom_radius = 0.04
			cyl.height = leg_height
			cyl.material = grey
			leg.mesh = cyl
			leg.position = Vector3(
				-body_size.x * 0.5 + sx * body_size.x * 0.25,
				foot_y + leg_height * 0.5,
				sz * body_size.z * 0.4)
			root.add_child(leg)

	# The AprilTag id-0 quad: centred on the lander ORIGIN (== tag center, §1). The
	# QuadMesh faces local +Z by default; rotate it so its +Z points along the
	# lander +X (the tag outward normal, toward the rover). pose_in_lander = identity.
	# Explicit type: apriltag_gen is an injected (untyped) script, so := cannot infer
	# the MeshInstance3D return — annotate it (build_tag_quad -> MeshInstance3D).
	var tag: MeshInstance3D = apriltag_gen.build_tag_quad(APRILTAG_SIZE_M, 32)
	# Map quad-local +Z -> lander-local +X (the tag outward normal, toward the rover):
	# a +90deg yaw about local +Y sends +Z->+X (verified against Godot's convention).
	tag.transform = Transform3D(Basis(Vector3(0, 1, 0), PI / 2.0), Vector3.ZERO)
	root.add_child(tag)

	print("sidecar: --cameras built procedural lander at (%.2f,%.2f,%.2f); tag id0 size_m=%.3f, normal toward rover" % [
		ground.x, tag_h, ground.z, APRILTAG_SIZE_M])
	return root

# ---------------------------------------------------------------------------
# Assemble the sensors.json Dictionary (contract §2.2, FROZEN v1.1; ALL poses
# Godot-frame). The verbatim move of sidecar.gd::_build_sensors_json, parameterized:
#
#   frame_index   : the real monotonic frame index (was hardcoded 0). Single-frame
#                   --cameras passes 0 so the M1 output is unchanged; multi-frame
#                   egress (M2-egress) passes the running index.
#   sun           : the additive v1.1 top-level "sun" block, WIRED NOW. Pass the
#                   live sidecar _sun_elev_deg/_sun_azim_deg (and time_delta_s, 0 by
#                   default). Always emitted now (contract §1).
#   faces         : OPTIONAL v1.1 apriltags[] (M3-tag lane). When null/empty the
#                   lander carries ONLY the v1.0 single apriltag{} (id 0). When the
#                   caller passes faces[] it ADDS apriltags[] (which supersedes
#                   apriltag{} per contract §3); the single apriltag{} stays for
#                   v1.0 back-compat. M1 callers pass null -> inert.
#   stereo_rear   : OPTIONAL v1.1 top-level "stereo_rear":{left,right,baseline_m}
#                   (M3-cam lane). NEVER replaces "stereo" (the front pair, which
#                   the frozen write_bag reads by name). When null -> inert/absent.
#
# `intrinsics_fn`/`fov_x_deg` are injected (camera_rig.gd is the sole owner of the
# intrinsics formula; this sink stays generic over the camera list it is handed).
static func build_sensors_json(
		scene: String,
		frame_index: int,
		view_size: Vector2i,
		rover_root: Node3D,
		lander_root: Node3D,
		cams: Array,
		intrinsics_fn: Callable,
		fov_x_deg: float,
		sun: Dictionary,
		faces = null,
		stereo_rear = null) -> Dictionary:
	var w := view_size.x
	var h := view_size.y

	var rover_pose := pose_dict(rover_root.global_transform)
	var lander_pose := pose_dict(lander_root.global_transform)
	lander_pose["frame_id"] = "lander"
	lander_pose["apriltag"] = {
		"family": "tag36h11",
		"id": 0,
		"size_m": APRILTAG_SIZE_M,
		"pose_in_lander": {"position_m": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]},
	}
	# v1.1 OPTIONAL apriltags[] (M3-tag lane): only present when the caller hands in
	# faces[]; supersedes apriltag{} per contract §3. Front face (id 0) keeps the
	# identity pose_in_lander so M1 stays correct. Inert when faces is null/empty.
	if faces != null and (faces is Array) and not (faces as Array).is_empty():
		lander_pose["apriltags"] = faces
	rover_pose["frame_id"] = "base_link"

	var intr: Dictionary = intrinsics_fn.call(fov_x_deg, w, h)
	var rover_inv: Transform3D = rover_root.global_transform.affine_inverse()
	var cameras: Array = []
	var extr_pos: Dictionary = {}       # name -> extrinsic position (for baseline check)
	for e in cams:
		var cam: Camera3D = e["cam"]
		var world_xf: Transform3D = cam.global_transform
		var extr_xf: Transform3D = rover_inv * world_xf
		extr_pos[String(e["name"])] = extr_xf.origin
		cameras.append({
			"name": String(e["name"]),
			"frame_id": String(e["frame_id"]),
			"image": String(e["image"]),
			"width": w,
			"height": h,
			"intrinsics": intr.duplicate(true),
			"pose_in_world": pose_dict(world_xf),
			"extrinsic_in_base_link": pose_dict(extr_xf),
		})

	# baseline_m = world distance between the two cameras; MUST equal the extrinsic
	# delta magnitude (contract §2.2). We compute from the extrinsics so the two are
	# identical by construction. "stereo" ALWAYS carries the FRONT pair (contract §2):
	# the frozen write_bag reads sensors['stereo']['left'/'right'] BY NAME.
	var baseline := 0.0
	if extr_pos.has("front_left") and extr_pos.has("front_right"):
		baseline = (extr_pos["front_left"] as Vector3).distance_to(extr_pos["front_right"])

	var doc := {
		"schema_version": SCHEMA_VERSION,
		"scene": scene,
		"frame_index": frame_index,
		"frame_convention": "godot",
		"sun": sun,
		"rover": rover_pose,
		"lander": lander_pose,
		"cameras": cameras,
		"stereo": {"left": "front_left", "right": "front_right", "baseline_m": baseline},
	}

	# v1.1 OPTIONAL "stereo_rear" (M3-cam lane): a SEPARATE top-level key, NEVER a
	# replacement for "stereo". Only present when the caller hands in a rear pair
	# {left,right,baseline_m} (or the cam names to resolve). Inert when null.
	if stereo_rear != null and (stereo_rear is Dictionary):
		doc["stereo_rear"] = stereo_rear

	return doc

# v1.1 top-level "sun" block (contract §1). Always emitted now: reads the live
# sidecar _sun_elev_deg / _sun_azim_deg members. time_delta_s defaults 0 (the
# A2-sweep lane passes a real per-frame value off its lunar-day sun model).
static func sun_block(elevation_deg: float, azimuth_deg: float, time_delta_s: float = 0.0) -> Dictionary:
	return {
		"elevation_deg": elevation_deg,
		"azimuth_deg": azimuth_deg,
		"time_delta_s": time_delta_s,
	}
