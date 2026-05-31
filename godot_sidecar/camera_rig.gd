extends RefCounted
class_name CameraRig
# Declarative 8-camera rig for the SLAM/perception egress
# (docs/sensor_bridge_contract.md §2, §4). A small extrinsics TABLE + a builder that
# mounts Camera3Ds (each in its own shared-World3D SubViewport, mirroring
# sidecar.gd::_probe_multicam_capture) on the articulated rover root.
#
# FRAME: all offsets below are in the ROVER ROOT's LOCAL frame, which is the
# §3 Godot convention (right-handed, +Y up, camera looks -Z). The rover's FORWARD
# is local +X (sidecar.gd header: front wheels LF/RF sit at +X; the chase camera
# treats +X as forward). So:
#   * forward  = local +X   (front pair looks this way -> the lander/tag)
#   * back     = local -X   (rear pair looks this way -> second stereo baseline)
#   * up       = local +Y   (mast height is +Y)
#   * lateral  = local +Z / -Z  (the FRONT stereo baseline runs along Z;
#                                the side monos look along +Z / -Z)
#
# M1 was TWO cameras (front_left, front_right). This is the full M3 8-camera rig
# (contract §4 reserved identifiers): the front pair is UNCHANGED (so M1/M2 do not
# regress) and the six reserved cameras are ADDED to the same CAMERAS table, which
# sidecar.gd::_cameras_capture already iterates generically (each new entry is
# captured + emitted automatically, no sidecar.gd edit).
#   front_left, front_right   front stereo (look +X)              [M1, UNCHANGED]
#   rear_left,  rear_right    rear stereo  (look -X, REAR_BASELINE_M, §4 stereo_rear)
#   left_mono,  right_mono    side monoculars (look +Z / -Z)      [§4 cameras[]]
#   drum_front_cam            aims at the FRONT drum-arm joint     [§4 cameras[]]
#   drum_back_cam             aims at the BACK  drum-arm joint     [§4 cameras[]]

# --- rig geometry: SOURCED from the EZ-RASSOR URDF (not [CALIB] guesses) -------
# ezrassor.xacro `camera_front_joint`: base_link -> depth_camera_front at
# xyz="0.3 0 -0.1" (Z-up), rpy 0; horizontal FOV 1.29154 rad (~74deg), 640x480
# (docs/ezrassor_assets.md §sensor stack). The URDF carries ONE mono depth camera;
# we mount the IPEx front-STEREO pair CENTERED on that real mount point (the stereo
# pair is the IPEx addition). Z-up -> Y-up via (x,y,z)_zup -> (x,z,-y)_yup:
#   (0.3, 0, -0.1)_zup -> (0.30 fwd +X, -0.10 up +Y i.e. 0.10 BELOW base_link, 0 lat).
const CAM_FORWARD_M := 0.30            # URDF camera_front X (forward of base_link)
const CAM_VERT_M := -0.10              # URDF camera_front Z(Z-up) -> Y-up: 0.10 m below base_link
# Stereo baseline: ~70 mm — a realistic small-rover stereo separation (John). The pair sits at
# +/- BASELINE_M/2 along the lateral (Z) axis centered on the URDF mount, so the world separation
# == BASELINE_M exactly (sensors.json baseline_m MUST equal |extrinsic_left.pos - right.pos|).
const BASELINE_M := 0.070              # realistic; was a [CALIB] 0.10 m guess
# REAR stereo baseline (contract §4 stereo_rear). A SEPARATE baseline from the front
# pair (the rear pair is its own descriptor, never a replacement for the front
# "stereo"). The rear module mirrors the front mount geometry, so we keep the same
# realistic ~70 mm separation; it is a distinct const so the two can diverge without
# touching the front pair. The rear pair sits at +/- REAR_BASELINE_M/2 along Z,
# centered on the rear mount, so |extrinsic(rear_left).pos - rear_right.pos| == it.
const REAR_BASELINE_M := 0.070         # rear-module stereo separation (§4 stereo_rear)
# Rear camera module mount: mirror the front mount to the BACK of base_link. The rear
# arm joint sits at base_link X = -0.20 (sidecar.gd ARM_BACK_ORIGIN); the rear stereo
# module rides just behind/above it at the symmetric -CAM_FORWARD_M, same height.
const REAR_CAM_BACK_M := -0.30         # symmetric to CAM_FORWARD_M (behind base_link)
# Side-monocular mounts: at the rover's lateral edge (track half-width ~0.285 m, the
# wheel-pivot Z from sidecar.gd WHEEL_ORIGINS), level with the front module, looking
# straight out the side (+Z = left, -Z = right). Forward offset 0 (mid-chassis).
const SIDE_MONO_LAT_M := 0.285         # base_link -> wheel-pivot Z (track half-width)
const SIDE_MONO_VERT_M := -0.05        # a touch above the front module (sees the wheel + ground)
# Drum-inspection camera mounts (contract §4 drum_front_cam / drum_back_cam). These
# cameras AIM AT the drum-arm joints (where the excavation happens) — they sit above
# the chassis, fore/aft, and look down/out at the live (pitched) drum-arm node. The
# mount is on the chassis; the AIM TARGET is the live drum joint node (looked up by
# name on the rover root at build time, so the aim tracks the arm's current pitch).
const DRUM_CAM_VERT_M := 0.18          # above base_link, on the camera mast, sees over the body
const DRUM_FRONT_CAM_FWD_M := 0.10     # just ahead of base_link center, looking fwd+down at front drum
const DRUM_BACK_CAM_FWD_M := -0.10     # just behind base_link center, looking back+down at back drum
# Drum joint node names on the rover root (sidecar.gd::_build_rover: arm pivots named
# "arm_front"/"arm_back", each carrying a "drum_front"/"drum_back" child). We aim at
# the DRUM node (the bucket) when present, else the ARM pivot, else a static fallback.
const DRUM_FRONT_NODE := "arm_front"   # front drum-arm pivot (carries "drum_front")
const DRUM_BACK_NODE := "arm_back"     # back  drum-arm pivot (carries "drum_back")

# Horizontal FOV from the URDF depth camera (1.29154 rad). fov IS the horizontal fov when
# keep_aspect = KEEP_WIDTH (set per cam below); drives intrinsics fx=fy=(w/2)/tan(fov_x/2).
const FOV_X_DEG := 73.99               # URDF 1.29154 rad (was a [CALIB] 70 guess)
const NEAR_M := 0.02
const FAR_M := 100.0

# The declarative extrinsics table. Local offsets (rover/base_link frame). Each entry's
# "look" is the desired OPTICAL FORWARD direction in the rover-local frame (the camera's
# -Z is aimed along it); look_basis() builds the matching camera Basis. frame_id matches
# the contract §2.2 / §4 schema.
#
# FRONT pair (M1, UNCHANGED): left = +Z half-baseline, right = -Z half-baseline, both
# look +X. The actual L/R image handedness is C1's concern after the §3 conversion; what
# matters here is the pair is laterally separated by BASELINE_M, centered on the URDF
# mount, and both look forward. (Their offsets/look reproduce the prior 2-camera table
# byte-for-byte: look_basis(+X) == the old forward_look_basis().)
#
# REAR pair (§4 stereo_rear): mirror of the front module at the back, look -X, separated
# by REAR_BASELINE_M. SIDE monos look straight out their side (+Z / -Z). DRUM cams carry
# an "aim" node name instead of a fixed "look": build() points them at the live drum
# joint node (so the aim follows the arm's pitch); the "look" here is only the fallback
# direction used if that node is absent (chassis-only rover).
const CAMERAS := [
	{
		"name": "front_left",
		"frame_id": "front_left_optical",
		"image": "front_left.png",
		"offset": Vector3(CAM_FORWARD_M, CAM_VERT_M, 0.5 * BASELINE_M),
		"look": Vector3(1, 0, 0),          # forward +X
		"pitchable": true,                  # the front pair honors --cam-pitch (M1 behavior)
	},
	{
		"name": "front_right",
		"frame_id": "front_right_optical",
		"image": "front_right.png",
		"offset": Vector3(CAM_FORWARD_M, CAM_VERT_M, -0.5 * BASELINE_M),
		"look": Vector3(1, 0, 0),          # forward +X
		"pitchable": true,
	},
	# --- M3 reserved cameras (contract §4) ----------------------------------------
	{
		"name": "rear_left",
		"frame_id": "rear_left_optical",
		"image": "rear_left.png",
		# Rear module mirrors the front: when looking BACKWARD (-X), the camera's local
		# right (+X column of the basis) points to -Z, so the "left" image side is -Z.
		# We keep the SAME +Z/-Z assignment as the front (left=+Z, right=-Z) for table
		# regularity; |left.pos - right.pos| == REAR_BASELINE_M regardless of handedness.
		"offset": Vector3(REAR_CAM_BACK_M, CAM_VERT_M, 0.5 * REAR_BASELINE_M),
		"look": Vector3(-1, 0, 0),         # backward -X
	},
	{
		"name": "rear_right",
		"frame_id": "rear_right_optical",
		"image": "rear_right.png",
		"offset": Vector3(REAR_CAM_BACK_M, CAM_VERT_M, -0.5 * REAR_BASELINE_M),
		"look": Vector3(-1, 0, 0),         # backward -X
	},
	{
		"name": "left_mono",
		"frame_id": "left_mono_optical",
		"image": "left_mono.png",
		"offset": Vector3(0.0, SIDE_MONO_VERT_M, SIDE_MONO_LAT_M),
		"look": Vector3(0, 0, 1),          # out the LEFT side (+Z)
	},
	{
		"name": "right_mono",
		"frame_id": "right_mono_optical",
		"image": "right_mono.png",
		"offset": Vector3(0.0, SIDE_MONO_VERT_M, -SIDE_MONO_LAT_M),
		"look": Vector3(0, 0, -1),         # out the RIGHT side (-Z)
	},
	{
		"name": "drum_front_cam",
		"frame_id": "drum_front_cam_optical",
		"image": "drum_front_cam.png",
		"offset": Vector3(DRUM_FRONT_CAM_FWD_M, DRUM_CAM_VERT_M, 0.0),
		"aim": DRUM_FRONT_NODE,            # point at the live FRONT drum-arm joint
		# fallback (fwd+down) if the node is absent; look_basis() normalizes it, so the raw
		# direction is fine (a const expression cannot call .normalized()).
		"look": Vector3(1, -1, 0),
	},
	{
		"name": "drum_back_cam",
		"frame_id": "drum_back_cam_optical",
		"image": "drum_back_cam.png",
		"offset": Vector3(DRUM_BACK_CAM_FWD_M, DRUM_CAM_VERT_M, 0.0),
		"aim": DRUM_BACK_NODE,             # point at the live BACK drum-arm joint
		# fallback (back+down) if the node is absent; look_basis() normalizes it.
		"look": Vector3(-1, -1, 0),
	},
]

# The stereo pairs this rig exposes, in the EXACT shape sensors_emit.build_sensors_json
# consumes for "stereo" / "stereo_rear": {left, right, baseline_m}. "front" is the M1
# pair (sensors_emit hardcodes it as "stereo"); "rear" is the optional §4 rear pair the
# caller hands to build_sensors_json's `stereo_rear` parameter. baseline_m here is the
# DESIGN baseline (the const); rear_pair_descriptor() below recomputes it from the actual
# built extrinsics so the emitted value is identical-by-construction to |Lpos - Rpos|
# (contract §2.2), exactly as sensors_emit does for the front pair.
const STEREO_PAIRS := {
	"front": {"left": "front_left", "right": "front_right", "baseline_m": BASELINE_M},
	"rear": {"left": "rear_left", "right": "rear_right", "baseline_m": REAR_BASELINE_M},
}

# Local basis for a camera whose OPTICAL FORWARD (-Z) points along `fwd_local` (in the
# rover-local frame), with up as close to world-local +Y as possible. A Godot Camera3D
# looks down its local -Z, with +Y up and +X right; the basis COLUMNS are (right, up,
# back) where back = -forward. We build a right-handed orthonormal basis (det +1, a proper
# rotation — never a mirror) from `fwd_local` + the +Y up hint:
#   back  (+Z col) = -forward
#   right (+X col) = up_hint x back      (= forward x up_hint, normalized)
#   up    (+Y col) = back x right        (re-orthogonalized true up)
# If forward is (anti)parallel to +Y (a straight up/down look), we fall back to +Z as the
# up hint so the cross products stay well-conditioned.
static func look_basis(fwd_local: Vector3) -> Basis:
	var fwd := fwd_local.normalized()
	if fwd.length() < 1e-6:
		fwd = Vector3(1, 0, 0)
	var up_hint := Vector3(0, 1, 0)
	if absf(fwd.dot(up_hint)) > 0.999:
		up_hint = Vector3(0, 0, 1)         # degenerate (looking along +/-Y): pick a side up
	var z_axis := -fwd                      # camera back = -forward
	var x_axis := up_hint.cross(z_axis).normalized()    # camera right
	var y_axis := z_axis.cross(x_axis).normalized()      # true camera up
	return Basis(x_axis, y_axis, z_axis)

# Local basis for a camera that LOOKS ALONG ROVER FORWARD (+X) with up = +Y. Kept as the
# named M1 helper (sidecar/tests may reference it); it is exactly look_basis(+X), which
# reproduces the original hand-derived basis: X(right)=(0,0,1), Y(up)=(0,1,0),
# Z(back)=(-1,0,0), det +1. So the front pair is byte-for-byte the pre-lane M1 pose.
static func forward_look_basis() -> Basis:
	return look_basis(Vector3(1, 0, 0))

# Pinhole intrinsics from a horizontal fov + image dims (contract §2.2 rule):
#   fx = fy = (width/2) / tan(fov_x/2),  cx = width/2,  cy = height/2.
# Returns a Dictionary ready to drop into sensors.json intrinsics (distortion OFF).
static func intrinsics(fov_x_deg: float, w: int, h: int) -> Dictionary:
	var fx := (float(w) * 0.5) / tan(deg_to_rad(fov_x_deg) * 0.5)
	return {
		"model": "pinhole",
		"fx": fx,
		"fy": fx,                       # square pixels: fy == fx
		"cx": float(w) * 0.5,
		"cy": float(h) * 0.5,
		"distortion_model": "plumb_bob",
		"D": [0, 0, 0, 0, 0],           # rectified pinhole for M1 (distortion OFF)
	}

# Build the full 8-camera rig as shared-World3D SubViewports parented under `parent`
# (the sidecar root), each carrying a Camera3D positioned at its rig offset RELATIVE TO
# `mount` (the rover root) and oriented per its table entry.
#
# We DO NOT parent the Camera3D under the rover node (a SubViewport renders its OWN child
# cameras), so we compose the world transform explicitly:
#   cam.global_transform = mount.global_transform * local_offset_transform
# This keeps the proven probe mechanism (one SubViewport per view, shared world) while
# still riding the rover's pose/yaw.
#
# Orientation per entry:
#   * "look" entries  -> look_basis(spec.look) in the rover-local frame.
#   * the front pair  -> additionally honors `pitch_deg` (the --cam-pitch downward tilt),
#     applied about the camera's local +X (right) exactly as the M1 path did, so pitch=0
#     reproduces the prior front-stereo pose byte-for-byte.
#   * "aim" entries (drum cams) -> the camera's optical axis is pointed at the LIVE drum
#     joint node (looked up by name under `mount`), computed in WORLD space so it tracks
#     the arm's current pitch. If the node is absent (chassis-only rover) it falls back to
#     look_basis(spec.look).
#
# Returns an Array of Dictionaries: {name, frame_id, image, sv, cam} so the caller can read
# each camera's global_transform for sensors.json and grab the rendered texture per view.
static func build(parent: Node, mount: Node3D, world: World3D,
		view_size: Vector2i, pitch_deg: float = 0.0) -> Array:
	var mount_xf: Transform3D = mount.global_transform
	var out: Array = []
	for spec in CAMERAS:
		var sv := SubViewport.new()
		sv.size = view_size
		sv.world_3d = world                                   # SHARE the built scene
		sv.render_target_update_mode = SubViewport.UPDATE_ALWAYS
		sv.render_target_clear_mode = SubViewport.CLEAR_MODE_ALWAYS
		parent.add_child(sv)

		var cam := Camera3D.new()
		cam.fov = FOV_X_DEG
		cam.keep_aspect = Camera3D.KEEP_WIDTH   # fov IS the horizontal fov -> intrinsics match
		cam.near = NEAR_M
		cam.far = FAR_M
		sv.add_child(cam)

		var offset: Vector3 = spec["offset"]
		var aim_node := String(spec.get("aim", ""))
		var aimed := false
		if aim_node != "":
			# Aim at the live drum-arm joint node (world space) so the optical axis tracks
			# the arm's current pitch. Compose only the camera POSITION from the mount; the
			# ORIENTATION is a world-space look_at toward the joint origin.
			var target = _aim_target_world(mount, aim_node)   # Vector3 or null (untyped)
			if target != null:
				var cam_pos_world: Vector3 = mount_xf * offset
				cam.global_transform = _look_at_xf(cam_pos_world, target as Vector3, Vector3(0, 1, 0))
				aimed = true
		if not aimed:
			# Fixed local look (front/rear/side, or drum fallback when the node is absent).
			var look := look_basis(spec["look"])
			# Optional DOWNWARD pitch for the FRONT pair (--cam-pitch): rotate the look basis
			# about the camera's local +X (right) by -pitch so the optical axis tilts toward
			# the ground. Lets the front stereo aim at the terrain/boulders (dense passive-
			# stereo depth) instead of the mostly-black sky a level gaze sees. pitch_deg=0 ->
			# the original level look (M1 byte-for-byte). Only the front pair is pitchable so
			# the rear/side/drum geometry is deterministic.
			if bool(spec.get("pitchable", false)) and absf(pitch_deg) > 1e-6:
				look = look * Basis(Vector3(1, 0, 0), deg_to_rad(-pitch_deg))
			var local_xf := Transform3D(look, offset)
			cam.global_transform = mount_xf * local_xf
		cam.current = true                       # active cam for THIS subviewport

		out.append({
			"name": String(spec["name"]),
			"frame_id": String(spec["frame_id"]),
			"image": String(spec["image"]),
			"sv": sv,
			"cam": cam,
		})
	return out

# World-space origin of the drum-arm joint node `node_name` under `mount` (recursive), or
# null if it is absent (chassis-only rover). Used to aim the drum-inspection cameras at the
# live (pitched) drum joint so the bucket stays framed regardless of arm pose.
static func _aim_target_world(mount: Node, node_name: String):
	var n := mount.find_child(node_name, true, false)   # recursive, not owner-limited
	if n != null and n is Node3D:
		return (n as Node3D).global_transform.origin
	return null

# A Godot camera global_transform that places the optical origin at `eye` and aims the
# optical axis (-Z) at `target`, with up as close to `up_hint` as possible. Built directly
# (not via Camera3D.look_at) so it works before the node enters the tree and stays a proper
# rotation; if the eye->target ray is (anti)parallel to up_hint we swap the up hint to +Z.
static func _look_at_xf(eye: Vector3, target: Vector3, up_hint: Vector3) -> Transform3D:
	var dir := (target - eye)
	if dir.length() < 1e-6:
		dir = Vector3(1, 0, 0)
	dir = dir.normalized()
	var u := up_hint
	if absf(dir.dot(u.normalized())) > 0.999:
		u = Vector3(0, 0, 1)
	var z_axis := -dir                       # camera back = -look direction
	var x_axis := u.cross(z_axis).normalized()
	var y_axis := z_axis.cross(x_axis).normalized()
	return Transform3D(Basis(x_axis, y_axis, z_axis), eye)

# The rear stereo-pair descriptor in the EXACT shape sensors_emit.build_sensors_json
# consumes for its `stereo_rear` parameter: {left, right, baseline_m}. Pass the Array
# returned by build() so the baseline is recomputed from the ACTUAL built rear-camera
# world positions (relative to the rover) — identical-by-construction to
# |extrinsic(rear_left).pos - extrinsic(rear_right).pos|, exactly as sensors_emit derives
# the front "stereo" baseline. Returns null if the rear pair was not built (so a caller can
# pass it straight through; null => stereo_rear absent, the M1 case). The rear pair is a
# SEPARATE top-level "stereo_rear" — NEVER a replacement for the front "stereo" (§2.2/§4).
#
# INTEGRATION NOTE (orchestrator wires at merge): sidecar.gd::_cameras_capture currently
# calls build_sensors_json(..., sun, null, null) with stereo_rear = null (the frozen M1
# call-site). To emit stereo_rear, that ONE argument becomes
#   CameraRigScript.rear_pair_descriptor(cams)
# (sidecar.gd is frozen for this lane; this is a 1-line change recorded for the merge).
static func rear_pair_descriptor(cams: Array, mount: Node3D = null):
	var lpos = _extr_pos_of(cams, "rear_left", mount)
	var rpos = _extr_pos_of(cams, "rear_right", mount)
	if lpos == null or rpos == null:
		return null
	var baseline: float = (lpos as Vector3).distance_to(rpos as Vector3)
	return {"left": "rear_left", "right": "rear_right", "baseline_m": baseline}

# Extrinsic (rover-local) position of camera `name` in the built `cams` array, or null if
# absent. If `mount` is given we express the world camera origin in the rover frame
# (matching sensors_emit's extrinsic_in_base_link); otherwise we use the world origin
# directly (the magnitude of the L-R DIFFERENCE — the baseline — is frame-invariant for a
# rigid pair, so either yields the same baseline_m).
static func _extr_pos_of(cams: Array, name: String, mount: Node3D):
	for e in cams:
		if String(e["name"]) == name:
			var cam: Camera3D = e["cam"]
			var world_pos: Vector3 = cam.global_transform.origin
			if mount != null:
				return mount.global_transform.affine_inverse() * world_pos
			return world_pos
	return null
