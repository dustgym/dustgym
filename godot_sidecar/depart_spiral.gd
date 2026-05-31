extends RefCounted
class_name DepartSpiral
# OWNER LANE: DEMO-GODOT (fixed-center lander + spiral egress).
#
# Drives the --depart-spiral flag (demo_spiral_contract.md §2): the rover departs the
# scene-center AprilTag lander along an Archimedean spiral, the FRONT stereo facing the
# lander each step, so the egress sweeps the "runtime parameters of a larger, longer
# simulation" -- the wanted, observed failure modes (out-of-range, occlusion, shadow).
#
# THE behaviour difference from --cameras / --cameras-seq (demo_spiral_contract.md §0/§2):
#   * --cameras (sidecar.gd::_cameras_capture) and --cameras-seq (capture_seq.gd) RE-PLACE
#     the lander at rover_pos + fwd*standoff EVERY frame -- it rides WITH the rover.
#   * Here the 4-face lander bundle is placed ONCE at the scene-center cell at frame 0 and
#     its Transform3D is HELD CONSTANT for all N frames. Ground truth depends on a constant
#     T_map_lander (§0 consequence 1); getting this wrong silently invalidates every pose
#     comparison, so the constant pose is captured once and re-emitted verbatim each frame.
#
# REUSE, NOT FORK. This NEVER edits the frozen seams it CALLS:
#   * lander_bundle.gd  -- LanderBundle._build_4face_lander() builds the SAME 4-face LIT
#     bundle (ids 0..3, known pose_in_lander) the --lander-faces path builds, and
#     LanderBundle._build_faces_array() builds the v1.1 apriltags[] superset. We call them
#     directly (the leading _ is GDScript convention, not access control) instead of
#     copying their bodies -- so the bundle stays byte-identical to --lander-faces.
#   * camera_rig.gd     -- CameraRig.build() mounts the front stereo (BASELINE_M=0.070,
#     FOV_X_DEG=73.99) per frame on the live rover; CameraRig.intrinsics is the idealized
#     pinhole (distortion_model 'plumb_bob', D=[0,0,0,0,0], §0).
#   * sensors_emit.gd   -- SensorsEmit.build_sensors_json() is the frozen schema sink; we
#     hand it the per-frame rover FLOAT pose + the 4-face apriltags[] + the CONSTANT lander
#     pose. SensorsEmit.sun_block() emits the additive "sun" block.
#   * apriltag_gen.gd   -- reached transitively through lander_bundle's bundle build (the
#     canonical tag36h11 bitmaps + the QuadMesh texture-yaw convention).
# The per-frame egress + sensors.json assembly mirror capture_seq.gd::run_capture_seq
# verbatim (the proven _cameras_capture await/save pattern), EXCEPT the lander is built once
# and held, and the rover footprint is the spiral rover_rc (look-at-lander yaw) not a
# straight approach.
#
# DISPATCH (orchestrator wires at merge, 1 line, mirrors capture_seq.gd ~209). sidecar.gd
# is FROZEN for this lane; it already preloads sibling lane scripts as DepartSpiralScript-
# style consts and awaits void coroutines before quit(0). The merge adds, alongside the
# --cameras-seq / --lander-faces dispatch (sidecar.gd ~205-218):
#       const DepartSpiralScript := preload("res://depart_spiral.gd")   # DEMO-GODOT
#       var _depart_spiral_mode := false
#   in _parse_args():   "--depart-spiral": _depart_spiral_mode = true ; _drums_up = true
#   in _ready():        if _depart_spiral_mode:
#                           await DepartSpiralScript.run_depart_spiral(self)
#                           get_tree().quit(0); return
# It MUST be awaited (this is a coroutine awaiting frame_post_draw per frame); an un-awaited
# call before quit(0) renders only one post-quit frame -> a black egress. The --depart-spiral
# flag inherits the --cameras --drums-up side effect so the drum arms clear the front-stereo
# FOV (mirrors --cameras-seq, capture_seq.gd header). This is recorded for the orchestrator
# here exactly as capture_seq.gd records its own `await` requirement.

# --- spiral parameters (demo_spiral_contract.md §1) ------------------------------
# These mirror scripts/demo/spiral_path.py::spiral_rc / look_at_yaw (DEMO-TRAJ lane). That
# lane is NOT yet merged into this worktree, so the spiral is computed HERE in-engine with
# the SAME math (Archimedean r(theta)=r0+r_growth*theta/2pi, monotonically increasing range
# so the rover progressively departs -> the out_of_range failure). When DEMO-TRAJ merges,
# the orchestrator may instead author per-frame rover_rc into the scene and iterate it like
# sidecar.gd::_run_sequence; the in-engine spiral here is the standalone path. The defaults
# are [CALIB] demo-framing values (NOT a sourced sensor spec), tuned to the shipped 256x256
# @ 0.02 m scenes: the spiral spans ~0.4 m (r0) out to a few metres so the rover crosses the
# resolvable -> out-of-range boundary within the patch.
const DEFAULT_FRAMES := 80       # 16 pts/lap * 5 laps (overridable via --stride)
const TURNS := 5.0               # [CALIB] revolutions of the spiral (16 frames/lap at 80 frames)
const R0_CELLS := 30.0           # [CALIB] start radius 15 m (0.5 m/cell); inside 15 m uses the lander's close-in recharging fiducials (out of scope here)
const R_GROWTH_CELLS := 36.0     # [CALIB] radius gained per full turn (~18 m/turn -> ~100 m at 5 laps, 0.5 m/cell)

# Archimedean spiral rover_rc waypoints about center_rc (the lander cell), in the SAME
# (row,col) field convention sidecar.gd uses (row -> world +Z, col -> world +X). theta runs
# [0, 2pi*turns]; r(theta)=r0+r_growth*theta/2pi. Returns Array[Vector2] (float row,col) so
# the sub-cell float pose is preserved (the integer rover_rc placement channel is separate).
# Mirrors spiral_path.spiral_rc (DEMO-TRAJ §1): r in CELLS, center in cells, n waypoints.
static func _spiral_rc(center_rc: Vector2, n_frames: int) -> Array:
	var out: Array = []
	var theta_max := TAU * TURNS
	for k in range(n_frames):
		# Sample theta uniformly across the arc; n_frames>=2 so the divisor is safe.
		var theta := theta_max * (float(k) / float(maxi(1, n_frames - 1)))
		var r := R0_CELLS + R_GROWTH_CELLS * (theta / TAU)
		# Field axes: row (+Z) = r*sin(theta), col (+X) = r*cos(theta) about the center cell.
		var row := center_rc.x + r * sin(theta)
		var col := center_rc.y + r * cos(theta)
		out.append(Vector2(row, col))
	return out

# Heading (radians about +Y) that points the rover's +forward (front stereo, local +X) AT
# the lander center, in the SAME yaw convention as sidecar.gd::_heading_yaw (col delta ->
# +X, row delta -> +Z; yaw = atan2(-dz, dx)). Here the travel vector is rover -> lander, so
# the front pair stays on the tag each step. Mirrors spiral_path.look_at_yaw (DEMO-TRAJ §1).
static func _look_at_yaw(rover_rc: Vector2, center_rc: Vector2) -> float:
	var dx := float(center_rc.y - rover_rc.y)   # col delta -> +X (toward lander)
	var dz := float(center_rc.x - rover_rc.x)   # row delta -> +Z (toward lander)
	if absf(dx) < 1e-6 and absf(dz) < 1e-6:
		return 0.0
	return atan2(-dz, dx)

# Load the host-emitted per-frame conform pose track (drive_spiral.py rover_pose.json):
# {"records":[{frame, rc:[row,col], yaw_rad, up:[x,y,z], z_m, pitch_deg, roll_deg, fiducial_cam}]}
# (or a bare list). Returns the records Array, or [] on any failure so a missing file simply
# falls back to the in-engine look-at-lander yaw + flat pose (back-compat). Shared by both the
# --depart-spiral (rover-cam) and --topdown-spiral drivers; reuses the _load_qt_leaves idiom.
static func load_pose_track(path: String) -> Array:
	var p := path
	if not (p.begins_with("res://") or p.begins_with("user://") or p.begins_with("/")):
		p = "res://" + p
	var f := FileAccess.open(p, FileAccess.READ)
	if f == null:
		push_warning("depart_spiral: cannot open rover-pose '%s' (err %d) -- using look-at-lander" % [
			p, FileAccess.get_open_error()])
		return []
	var txt := f.get_as_text()
	f.close()
	var parsed = JSON.parse_string(txt)
	if typeof(parsed) == TYPE_ARRAY:
		return parsed
	if typeof(parsed) == TYPE_DICTIONARY and parsed.has("records"):
		return parsed["records"]
	push_warning("depart_spiral: rover-pose '%s' did not parse to records" % p)
	return []

# ENTRY POINT (dispatched by sidecar.gd at merge: `await run_depart_spiral(self)` then
# get_tree().quit(0)). VOID coroutine (the capture_seq.gd pattern): builds the constant
# lander once, then per frame places the rover at the spiral rover_rc + look-at yaw and
# writes out/cam/<scene>/<NNN>/{front_left,front_right}.png + sensors.json via the frozen
# egress. `sidecar` is the sidecar Node3D, already past _setup_environment + _build_layers.
static func run_depart_spiral(sidecar) -> void:
	var sf = sidecar.sf
	if sf == null:
		push_error("depart_spiral: --depart-spiral requires a loaded scene (--scene <dir>)")
		sidecar.get_tree().quit(2)
		return
	if not sidecar._layers.has("rover"):
		push_error("depart_spiral: --depart-spiral requires the 'rover' layer; add 'rover' to --layers")
		sidecar.get_tree().quit(4)
		return

	# Frame count: --stride doubles as the frame-count knob (capture_seq.gd idiom; sidecar.gd
	# parses it into _seq_stride, no new flag). Floor at 2 so the spiral has a baseline.
	var n_frames: int = maxi(2, sidecar._seq_stride if sidecar._seq_stride > 2 else DEFAULT_FRAMES)

	# --- scene-center cell + world position (the FIXED lander site) ------------------
	# Center CELL (row,col); .5 keeps it at the true grid center for even dims (256 -> 127.5).
	var center_rc := Vector2(float(sf.height - 1) * 0.5, float(sf.width - 1) * 0.5)
	# Center WORLD (x,y,z): col -> +X, row -> +Z, surface-snapped via the bilinear height_uv
	# (the SAME mapping sidecar.gd::_build_rover uses for the rover_rc placement branch).
	var cu: float = clampf(center_rc.y / float(sf.width - 1), 0.0, 1.0)
	var cv: float = clampf(center_rc.x / float(sf.height - 1), 0.0, 1.0)
	var center_world := Vector3(
		sf.world_min.x + center_rc.y * sf.cell_m,
		sf.height_uv(cu, cv),
		sf.world_min.y + center_rc.x * sf.cell_m)

	# --- spiral trajectory (rover_rc + look-at yaw per frame) ------------------------
	var rc_seq: Array = _spiral_rc(center_rc, n_frames)        # Array[Vector2] (float row,col)
	var yaw_seq: Array = []
	for k in range(n_frames):
		yaw_seq.append(_look_at_yaw(rc_seq[k], center_rc))

	# Optional conform pose track (drive_spiral.py): overrides look-at-lander with the
	# TRAVEL-TANGENT heading + wheel-plane tilt, so the rover faces its direction of travel and
	# the SIDE mono (not the front stereo) acquires the lander fiducial. Absent => legacy behavior.
	var pose_track: Array = []
	if sidecar._rover_pose_path != "":
		pose_track = load_pose_track(sidecar._rover_pose_path)
		print("depart_spiral: loaded %d rover-pose records from %s" % [pose_track.size(), sidecar._rover_pose_path])

	var scene: String = sf.scene_name
	# Output dir override (mirrors topdown_spiral): separate lit/unlit rover-cam runs off one
	# driven scene (e.g. --out-scene-name haworth_spiral_lit / haworth_spiral_unlit).
	var out_scene: String = sidecar._out_scene_name if sidecar._out_scene_name != "" else scene
	print("depart_spiral: --depart-spiral scene='%s' out='%s' frames=%d center_rc=(%.1f,%.1f) turns=%.1f r0_cells=%.1f r_growth_cells=%.1f" % [
		scene, out_scene, n_frames, center_rc.x, center_rc.y, TURNS, R0_CELLS, R_GROWTH_CELLS])

	# --- build the 4-face lander bundle ONCE at the scene center (held constant) ------
	# lander_bundle._build_4face_lander places the lander at rover_pos + fwd*standoff and
	# orients lander +X = -fwd (the tag outward normal toward that rover_pos). We want the
	# lander pinned AT center_world; so we choose a synthetic placement rover_pos so that
	# rover_pos + fwd*standoff == center_world (i.e. rover_pos = center_world - fwd*standoff),
	# and we pick fwd = world -X (an arbitrary fixed direction): the front (id0) face then
	# points toward world +X. Any fixed fwd works -- the multi-face bundle gives >=1 resolvable
	# face around most of the orbit, so the front face need not track the rover (§0/§2: the
	# 4-face bundle is REQUIRED precisely because no single face stays visible over a spiral).
	# standoff is forced to a fixed value (the --lander-standoff default path uses 2.5 when <0,
	# but here we pass an explicit positive value so center placement is exact regardless of CLI).
	var standoff: float = sidecar._lander_standoff if sidecar._lander_standoff > 0.0 else 2.5
	var fixed_fwd := Vector3(-1, 0, 0)                          # lander +X (id0 normal) -> world +X
	var place_rover_pos := center_world - fixed_fwd * standoff
	# DEMO illumination A/B: propagate --tag-unlit to the lander tag material (UNSHADED when set).
	sidecar.LanderBundleScript.unlit_tags = sidecar._tag_unlit
	var lander_root: Node3D = sidecar.LanderBundleScript._build_4face_lander(
		sidecar, sf, place_rover_pos, fixed_fwd, standoff, sidecar._lander_yaw_deg)
	# CAPTURE the constant lander Transform3D ONCE (the load-bearing T_map_lander, §0). The
	# lander node is never rebuilt or moved below, so this pose is the ground-truth datum
	# every frame's sensors.json re-emits verbatim (a constant lander pose across frames is
	# the contract's hard requirement, distinct from the per-frame moving-lander paths).
	var lander_xf_const: Transform3D = lander_root.global_transform
	print("depart_spiral: FIXED lander at world (%.3f,%.3f,%.3f) held constant for all %d frames" % [
		lander_xf_const.origin.x, lander_xf_const.origin.y, lander_xf_const.origin.z, n_frames])

	# The 4-face apriltags[] superset (ids 0..3, known pose_in_lander), via the frozen
	# lander_bundle builder -> the bundle's sensors.json is schema-identical to --lander-faces.
	# Constant across frames (the lander geometry never changes), so build it once.
	var faces: Array = sidecar.LanderBundleScript._build_faces_array(sidecar)

	# --- per-frame egress loop -------------------------------------------------------
	var world: World3D = sidecar.get_viewport().world_3d
	var n_written := 0
	for k in range(n_frames):
		var rc: Vector2 = rc_seq[k]
		# Drive the rover to this frame's spiral footprint + look-at-lander yaw. _build_rover
		# reads these members for its rover_rc placement branch; the integer cell is the
		# placement channel (sidecar rounds via float->int in the u/v lookup), while the FLOAT
		# rover pose reported in sensors.json comes from rover_root.global_transform below.
		sidecar._rover_rc_override = Vector2i(int(round(rc.x)), int(round(rc.y)))
		sidecar._rover_yaw = yaw_seq[k]
		sidecar._rover_up = Vector3.UP
		# Conform pose track override: TRAVEL-TANGENT yaw + wheel-plane tilt (rover-physics pass).
		if k < pose_track.size():
			var prec: Dictionary = pose_track[k]
			sidecar._rover_yaw = float(prec.get("yaw_rad", yaw_seq[k]))
			var upv = prec.get("up", null)
			if typeof(upv) == TYPE_ARRAY and upv.size() == 3:
				sidecar._rover_up = Vector3(float(upv[0]), float(upv[1]), float(upv[2]))
			var prc = prec.get("rc", null)
			if typeof(prc) == TYPE_ARRAY and prc.size() == 2:
				sidecar._rover_rc_override = Vector2i(int(round(float(prc[0]))), int(round(float(prc[1]))))

		# Rebuild ONLY the per-frame layer nodes (terrain/clasts/rover) -- this re-places the
		# articulated rover at the new spiral rover_rc/yaw, leaving the sun + WorldEnvironment
		# AND the constant lander in place. _clear_frame_nodes() frees children that are not
		# Camera3D/DirectionalLight3D/WorldEnvironment -- INCLUDING our lander_root. So we
		# DETACH the lander before the clear and RE-ATTACH it after, preserving its node (and
		# thus its captured constant Transform3D) untouched across the rebuild. This keeps the
		# lander byte-stable (no per-frame re-snap that the moving-lander paths incur).
		if lander_root.get_parent() != null:
			lander_root.get_parent().remove_child(lander_root)
		sidecar._clear_frame_nodes()
		sidecar._build_layers()
		sidecar.add_child(lander_root)
		lander_root.global_transform = lander_xf_const         # re-assert the held pose (paranoia)

		var rover_root = sidecar._find_rover_root()
		if rover_root == null:
			push_error("depart_spiral: frame %d found no rover root after rebuild" % k)
			sidecar.get_tree().quit(4)
			return

		# Front-stereo rig via the FROZEN camera_rig (shared-World3D SubViewports riding the
		# rover). Rebuilt per frame because the rover root is rebuilt; rigid extrinsics ->
		# baseline_m constant by const. --cam-pitch honored (front pair pitchable) so the
		# stereo can tilt down toward the textured regolith (§6 depth narration).
		# Build the rig, then AIM it at the tag: a small ground-level tag seen from the
		# elevated rover over a 1->8 m spiral needs a per-frame look-at pitch (steep down when
		# near, near-level when far) or it falls out of frame. We build once to read the front
		# camera's world height, compute the pitch onto the lander tag center (lander origin ==
		# id0 tag center), then rebuild the rig pitched to it -- preserving the parallel stereo
		# pair (a rig pitch, NOT a per-camera toe-in, so the baseline/extrinsics stay rigid).
		var cams: Array = sidecar.CameraRigScript.build(
			sidecar, rover_root, world, sidecar._viewport_size, sidecar._cam_pitch_deg)
		# TRAVEL-TANGENT runs (pose track present) leave the front stereo looking along travel and
		# let the SIDE mono frame the lander; only the LEGACY look-at-lander path aims the stereo
		# pitch onto the ground tag (steep-down near, level far) so the tag stays in frustum.
		if pose_track.is_empty():
			var cam_o: Vector3 = rover_root.global_transform.origin
			for e in cams:
				if String(e.get("image", "")).begins_with("front_left"):
					cam_o = (e["cam"] as Node3D).global_transform.origin
					break
			var tag_c: Vector3 = lander_xf_const.origin
			var horiz: float = Vector2(tag_c.x - cam_o.x, tag_c.z - cam_o.z).length()
			# pitch_deg POSITIVE = downward tilt (camera_rig.gd:234); cam above tag -> add atan2.
			var look_pitch: float = clampf(
				sidecar._cam_pitch_deg + rad_to_deg(atan2(cam_o.y - tag_c.y, maxf(horiz, 1e-3))),
				-20.0, 75.0)
			for e in cams:
				(e["sv"] as SubViewport).queue_free()
			cams = sidecar.CameraRigScript.build(
				sidecar, rover_root, world, sidecar._viewport_size, look_pitch)

		# Settle the subviewports, then capture. Geometry added this tick registers into the
		# world scenario only on the NEXT tree frame, so we MUST await frame_post_draw (the
		# proven sidecar._cameras_capture / capture_seq.gd pattern; first frame samples stale).
		for _w in range(3):
			await RenderingServer.frame_post_draw

		# --- write this frame's directory out/cam/<scene>/<NNN>/ -----------------------
		var nnn := "%03d" % k
		var out_dir := "res://out/cam/%s/%s" % [out_scene, nnn]
		DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))
		for e in cams:
			var img: Image = e["sv"].get_texture().get_image()
			var path := "%s/%s" % [out_dir, e["image"]]
			var err := img.save_png(path)
			if err != OK:
				push_error("depart_spiral: save_png failed (%d) for %s" % [err, path])
			else:
				print("depart_spiral: frame %s wrote %s (%dx%d)" % [
					nnn, ProjectSettings.globalize_path(path), img.get_width(), img.get_height()])

		# --- assemble + write the per-frame sensors.json (FROZEN sink) -----------------
		# REAL monotonic frame_index k; the CONSTANT lander pose (lander_root never moved,
		# its global_transform == lander_xf_const) + the 4-face apriltags[]; the per-frame
		# rover FLOAT pose (rover_root.global_transform). Idealized pinhole intrinsics (§0).
		var sun = sidecar.SensorsEmitScript.sun_block(
			sidecar._sun_elev_deg, sidecar._sun_azim_deg, 0.0)
		var doc = sidecar.SensorsEmitScript.build_sensors_json(
			scene, k, sidecar._viewport_size, rover_root, lander_root, cams,
			Callable(sidecar.CameraRigScript, "intrinsics"), sidecar.CameraRigScript.FOV_X_DEG,
			sun, faces, null)
		var json_path := "%s/sensors.json" % out_dir
		var jf := FileAccess.open(json_path, FileAccess.WRITE)
		if jf == null:
			push_error("depart_spiral: cannot open %s for write" % json_path)
			sidecar.get_tree().quit(6)
			return
		jf.store_string(JSON.stringify(doc, "  "))
		jf.close()

		# Honesty log: report the per-frame rover FLOAT pose + the constant lander pose +
		# the rover->lander range (the spiral's monotonically increasing departure range,
		# the out_of_range driver). The range is the float-pose channel (NOT the quantized
		# rover_rc), matching the channel-hygiene rule (§0 consequence 4 / eval_schema).
		var rp: Array = doc["rover"]["position_m"]
		var lp: Array = doc["lander"]["position_m"]
		var rng := Vector3(float(rp[0]), float(rp[1]), float(rp[2])).distance_to(
			Vector3(float(lp[0]), float(lp[1]), float(lp[2])))
		print("depart_spiral: frame %s frame_index=%d rover_pos=(%.3f,%.3f,%.3f) lander_pos=(%.3f,%.3f,%.3f) range_m=%.3f apriltags=%d baseline_m=%.4f" % [
			nnn, int(doc["frame_index"]), float(rp[0]), float(rp[1]), float(rp[2]),
			float(lp[0]), float(lp[1]), float(lp[2]), rng,
			(doc["lander"]["apriltags"] as Array).size(), float(doc["stereo"]["baseline_m"])])
		n_written += 1

	print("depart_spiral: --depart-spiral wrote %d frames to %s (lander held constant)" % [
		n_written, ProjectSettings.globalize_path("res://out/cam/%s" % out_scene)])

# ---------------------------------------------------------------------------
# SELF-TEST. The deliverable is a RefCounted (preloaded by sidecar.gd for its static
# methods), which `--headless --script` cannot run directly (it requires a SceneTree). So
# this is a STATIC test entry a tiny SceneTree harness calls: it exercises the PURE
# trajectory + look-at math that decides the demo's geometry (the engine-bound capture is
# verified separately by the render harness / --check-only). Returns the failure count.
# Asserts the load-bearing properties the contract names:
#   1. MONOTONIC range departure (drives out_of_range): r(theta) strictly increases.
#   2. look_at yaw points the rover +forward (local +X under Basis(UP,yaw)) AT the lander
#      center each frame (the tag stays in the stereo frustum) -- the +X column of
#      Basis(UP,yaw), projected to XZ, is colinear (positive dot) with the rover->lander
#      vector, in the SAME atan2(-dz,dx) convention as sidecar._heading_yaw.
#   3. the spiral is centered on the lander cell (frame-0 radius == R0_CELLS).
static func run_self_test() -> int:
	var fails := 0
	var center := Vector2(127.5, 127.5)        # 256x256 scene center cell (row,col)
	var n := 24
	var rc := _spiral_rc(center, n)
	if rc.size() != n:
		push_error("SELFTEST FAIL: spiral produced %d waypoints, expected %d" % [rc.size(), n])
		fails += 1

	# 1. Monotonic range departure (each step's distance-to-center strictly grows).
	var prev_r := -1.0
	var mono := true
	for k in range(n):
		var rck: Vector2 = rc[k]
		var r: float = (rck - center).length()
		if r <= prev_r:
			mono = false
		prev_r = r
	if not mono:
		push_error("SELFTEST FAIL: spiral range is not monotonically increasing (out_of_range driver broken)")
		fails += 1
	else:
		var rc_first: Vector2 = rc[0]
		var rc_last: Vector2 = rc[n - 1]
		print("SELFTEST ok: range monotonic, r[0]=%.2f cells -> r[%d]=%.2f cells" % [
			(rc_first - center).length(), n - 1, (rc_last - center).length()])

	# 1b. frame-0 radius == R0_CELLS (spiral starts at the configured start radius).
	var rc0: Vector2 = rc[0]
	var r0: float = (rc0 - center).length()
	if absf(r0 - R0_CELLS) > 1e-3:
		push_error("SELFTEST FAIL: frame-0 radius %.4f != R0_CELLS %.4f" % [r0, R0_CELLS])
		fails += 1

	# 2. look_at yaw aims the rover +forward at the lander, every frame. Basis(UP,yaw)*+X is
	#    the world forward; project to the XZ field plane and require a POSITIVE dot with the
	#    rover->lander direction (and near-unit, i.e. well-aligned, allowing float epsilon).
	var worst_align := 1.0
	for k in range(n):
		var rck: Vector2 = rc[k]
		var yaw := _look_at_yaw(rck, center)
		# Basis(UP,yaw)*Vector3(1,0,0) = (cos yaw, 0, -sin yaw): the world forward (+X) dir.
		var fwd_x := cos(yaw)
		var fwd_z := -sin(yaw)
		# rover->lander in field axes mapped to world (col->+X, row->+Z):
		var to_x := float(center.y - rck.y)
		var to_z := float(center.x - rck.x)
		var to_len: float = sqrt(to_x * to_x + to_z * to_z)
		if to_len < 1e-6:
			continue
		var align := (fwd_x * to_x + fwd_z * to_z) / to_len   # cos(angle); 1.0 == perfectly aimed
		worst_align = minf(worst_align, align)
	if worst_align < 0.9999:
		push_error("SELFTEST FAIL: look_at yaw misaligned with lander (worst cos=%.6f, expected ~1.0)" % worst_align)
		fails += 1
	else:
		print("SELFTEST ok: look_at yaw aims +forward at lander every frame (worst cos=%.6f)" % worst_align)

	if fails == 0:
		print("depart_spiral SELFTEST: ALL CHECKS PASSED (%d frames)" % n)
	else:
		push_error("depart_spiral SELFTEST: %d CHECK(S) FAILED" % fails)
	return fails
