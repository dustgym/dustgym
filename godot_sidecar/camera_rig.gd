extends RefCounted
class_name CameraRig
# Declarative front-stereo camera rig for the M1 SLAM egress
# (docs/sensor_bridge_contract.md §2). A small extrinsics TABLE + a builder that
# mounts Camera3Ds (each in its own shared-World3D SubViewport, mirroring
# sidecar.gd::_probe_multicam_capture) on the articulated rover root.
#
# FRAME: all offsets below are in the ROVER ROOT's LOCAL frame, which is the
# §3 Godot convention (right-handed, +Y up, camera looks -Z). The rover's FORWARD
# is local +X (sidecar.gd header: front wheels LF/RF sit at +X; the chase camera
# treats +X as forward). So:
#   * forward  = local +X   (both cameras look this way -> the lander/tag)
#   * up       = local +Y   (mast height is +Y)
#   * lateral  = local +Z / -Z  (the stereo baseline runs along Z)
#
# M1 = TWO cameras only: front_left, front_right. Rear stereo / side monos /
# drum cams are M3 (§4) and are deliberately NOT here so adding them stays purely
# additive.

# --- [CALIB] rig geometry -----------------------------------------------------
# Stereo baseline: metric left<->right separation. M1 default 0.100 m, flagged
# [CALIB] until an IPEx engineering figure is sourced (contract §2.2). The two
# cameras sit at +/- BASELINE_M/2 along the lateral (Z) axis, so their world
# separation == BASELINE_M exactly (the sensors.json baseline_m MUST equal
# |extrinsic_left.pos - extrinsic_right.pos|).
const BASELINE_M := 0.100              # [CALIB]
# Mast height above the rover body origin (base_link). ~0.4 m raises the stereo
# pair clear of the chassis so both see the lander 2.5 m ahead. [CALIB].
const MAST_HEIGHT_M := 0.40            # [CALIB]
# Forward standoff of the optical centres from base_link along +X, so the lenses
# sit just ahead of the mast rather than inside the body. [CALIB].
const FORWARD_OFFSET_M := 0.20         # [CALIB]
# Horizontal field of view (degrees) used for BOTH cameras. Drives the pinhole
# intrinsics (fx = fy = (w/2)/tan(fov_x/2)). Godot Camera3D.fov IS the horizontal
# fov when keep_aspect = KEEP_WIDTH (set on each cam below). [CALIB].
const FOV_X_DEG := 70.0                # [CALIB]
const NEAR_M := 0.02
const FAR_M := 100.0

# The declarative extrinsics table. Local offsets (rover frame) + the shared
# forward look direction. left = +Z half-baseline, right = -Z half-baseline
# (so that, viewed looking along +X forward with +Y up, "left" is on the +Z side;
# the actual left/right image handedness is C1's concern after the §3 conversion;
# what matters here is the pair is laterally separated by BASELINE_M and both look
# forward). frame_id matches the contract §2.2 schema.
const CAMERAS := [
	{
		"name": "front_left",
		"frame_id": "front_left_optical",
		"image": "front_left.png",
		"offset": Vector3(FORWARD_OFFSET_M, MAST_HEIGHT_M, 0.5 * BASELINE_M),
	},
	{
		"name": "front_right",
		"frame_id": "front_right_optical",
		"image": "front_right.png",
		"offset": Vector3(FORWARD_OFFSET_M, MAST_HEIGHT_M, -0.5 * BASELINE_M),
	},
]

# Local basis for a camera that LOOKS ALONG ROVER FORWARD (+X) with up = +Y.
# A Godot Camera3D looks down its local -Z, with +Y up and +X right. We want the
# optical axis (-Z) to point along the mount's +X, and camera up (+Y) = mount +Y.
# So the camera +Z column (back of view) = -forward = (-1,0,0). The +X (right)
# column must complete a RIGHT-HANDED basis (det +1): X = Y x Z.
#   Z (back) = (-1, 0, 0)
#   Y (up)   = ( 0, 1, 0)
#   X (right)= Y x Z = (0,1,0) x (-1,0,0) = (0, 0, 1)
# (det = X . (Y x Z) = +1; a proper rotation, not a mirror.) Basis cols are (X,Y,Z).
static func forward_look_basis() -> Basis:
	var x_axis := Vector3(0, 0, 1)    # camera right
	var y_axis := Vector3(0, 1, 0)    # camera up
	var z_axis := Vector3(-1, 0, 0)   # camera back (so -Z = +X forward)
	return Basis(x_axis, y_axis, z_axis)

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

# Build the front-stereo cameras as shared-World3D SubViewports parented under
# `parent` (the sidecar root), each carrying a Camera3D positioned at its rig
# offset RELATIVE TO `mount` (the rover root) and looking along rover forward.
#
# We DO NOT parent the Camera3D under the rover node (a SubViewport renders its
# OWN child cameras), so we compose the world transform explicitly:
#   cam.global_transform = mount.global_transform * local_offset_transform
# This keeps the proven probe mechanism (one SubViewport per view, shared world)
# while still riding the rover's pose/yaw.
#
# Returns an Array of Dictionaries: {name, frame_id, image, sv, cam} so the caller
# can read each camera's global_transform for sensors.json and grab the rendered
# texture per view.
static func build(parent: Node, mount: Node3D, world: World3D,
		view_size: Vector2i) -> Array:
	var look := forward_look_basis()
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
		# World pose = rover pose composed with the local rig offset + forward look.
		var local_xf := Transform3D(look, spec["offset"])
		cam.global_transform = mount.global_transform * local_xf
		cam.current = true                       # active cam for THIS subviewport

		out.append({
			"name": String(spec["name"]),
			"frame_id": String(spec["frame_id"]),
			"image": String(spec["image"]),
			"sv": sv,
			"cam": cam,
		})
	return out
