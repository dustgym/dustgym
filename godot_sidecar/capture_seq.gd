extends RefCounted
class_name CaptureSeq
# OWNER LANE: M2-egress (multi-frame camera sequence egress).
#
# Drives the --cameras-seq flag: produces the MOVING front-stereo sequence that
# rtabmap / COLMAP consume. The single-frame --cameras path (sidecar.gd::
# _cameras_capture) renders ONE frame; this is its multi-frame generalisation.
#
# Contract (FROZEN v1.1 §2.5 multi-frame egress dir convention):
#   out/cam/<scene>/<NNN>/{front_left,front_right}.png + a per-frame sensors.json
#   carrying the REAL monotonic frame_index + the per-frame rover pose_in_world;
#   intrinsics / baseline / extrinsic_in_base_link CONSTANT across frames (rigid
#   rig); <NNN> zero-padded 3 digits from 000, monotonically +1 per frame.
# The --cameras-seq flag inherits the live --cameras side effect (_drums_up=true,
# wired in sidecar.gd::_parse_args) so the drum arms clear the front-stereo FOV.
#
# This NEVER edits sidecar.gd, camera_rig.gd, or sensors_emit.gd: the --cameras-seq
# flag + dispatch call-site are wired in sidecar.gd by L0; the camera rig and the
# schema-assembly sink are FROZEN seams we CALL (camera_rig.gd::build /
# ::intrinsics, sensors_emit.gd::build_lander / ::build_sensors_json / ::sun_block),
# so the per-frame schema is byte-for-byte the single-frame --cameras schema except
# the monotonic frame_index + the moving rover/camera poses.
#
# TRAJECTORY SOURCING. The contract iterates a scene's per-frame rover_rc exactly as
# sidecar.gd::_run_sequence does. The shipped scenes (samples/tread_track_4wheel,
# samples/tread_track, ...) author rover_rc on their FINAL driven frame only (the
# earlier tNNN are the pre-drive null frame, rover_rc:null). A SLAM/COLMAP sequence
# needs >=2 frames whose rover position DIFFERS, so we anchor on the scene's authored
# driven rover_rc and synthesise a short straight APPROACH trajectory: N waypoints
# stepping the rover backward from the anchor along its path heading, so the rover
# genuinely moves frame-to-frame and ARRIVES at the authored driven pose on the last
# frame. The per-frame rover placement + heading-yaw math mirrors sidecar.gd
# (_build_rover's rover_rc branch + _heading_yaw) verbatim, so a real multi-rover_rc
# scene would slot in unchanged (the synthesised waypoints are just stand-in rover_rc).

# Default sequence length when --stride is left at its default. --stride doubles as the
# frame-count knob here (sidecar.gd parses it into _seq_stride; we never add a flag).
const DEFAULT_FRAMES := 6
# Per-frame rover advance toward the anchor, in GRID CELLS along the heading. ~6 cells
# at 0.02 m/cell ~= 0.12 m/frame on the tread scenes: a visible baseline between frames
# (well above the stereo baseline) without leaving the authored active zone.
const STEP_CELLS := 6

# Entry point dispatched from sidecar.gd::_ready (after _setup_environment +
# _build_layers have already built the scene once). `sidecar` is the sidecar Node3D;
# we read its loaded `sf`, its tuning members, and call its instance helpers
# (_clear_frame_nodes / _build_layers / _find_rover_root) + the FROZEN seam scripts.
static func run_capture_seq(sidecar) -> void:
	var sf = sidecar.sf
	if sf == null:
		push_error("capture_seq: --cameras-seq requires a loaded scene (--scene <dir>)")
		sidecar.get_tree().quit(2)
		return
	if not sidecar._layers.has("rover"):
		push_error("capture_seq: --cameras-seq requires the 'rover' layer; add 'rover' to --layers")
		sidecar.get_tree().quit(4)
		return

	# --- resolve the trajectory anchor: the scene's authored driven rover_rc -------
	var anchor: Vector2i = _resolve_anchor(sidecar, sf)
	if anchor.x < 0:
		push_error("capture_seq: scene '%s' has no rover_rc; pick a driven scene (e.g. samples/tread_track_4wheel)" % sf.scene_name)
		sidecar.get_tree().quit(5)
		return

	var n_frames: int = maxi(2, sidecar._seq_stride if sidecar._seq_stride > 2 else DEFAULT_FRAMES)

	# --- synthesise the approach trajectory (rover_rc per frame) -------------------
	# Heading: from the scene's authored heading if a real prior rover_rc exists, else
	# along grid +X (col axis). We step BACKWARD from the anchor by STEP_CELLS each
	# frame, then reverse so the rover APPROACHES and ENDS at the authored anchor: the
	# last frame == the single-frame --cameras pose, earlier frames trail behind it.
	var heading: Vector2 = _approach_heading(sidecar, sf, anchor)   # unit (drow, dcol)
	var rc_seq: Array = []          # Array[Vector2i], frame 0..n_frames-1 (moving)
	for k in range(n_frames):
		var back := float(n_frames - 1 - k) * float(STEP_CELLS)
		var r := int(round(float(anchor.x) - heading.x * back))
		var c := int(round(float(anchor.y) - heading.y * back))
		r = clampi(r, 0, sf.height - 1)
		c = clampi(c, 0, sf.width - 1)
		rc_seq.append(Vector2i(r, c))

	# Per-frame yaw from consecutive waypoints (same convention as sidecar._heading_yaw:
	# col delta -> +X, row delta -> +Z; yaw = atan2(-dz, dx) points rover forward +X
	# along travel). Constant heading here, but computed per frame so it is correct for
	# a curved real trajectory too.
	var yaw_seq: Array = []
	for k in range(n_frames):
		var a: Vector2i = rc_seq[maxi(0, k - 1)]
		var b: Vector2i = rc_seq[mini(n_frames - 1, k + 1)]
		var dx := float(b.y - a.y)      # col delta -> +X
		var dz := float(b.x - a.x)      # row delta -> +Z
		var yaw := 0.0
		if absf(dx) > 1e-6 or absf(dz) > 1e-6:
			yaw = atan2(-dz, dx)
		yaw_seq.append(yaw)

	var scene: String = sf.scene_name
	print("capture_seq: --cameras-seq scene='%s' frames=%d anchor_rc=%s step_cells=%d" % [
		scene, n_frames, str(anchor), STEP_CELLS])

	# --- per-frame egress loop -----------------------------------------------------
	var n_written := 0
	for k in range(n_frames):
		var rc: Vector2i = rc_seq[k]
		# Drive the rover to this frame's footprint + heading. _build_rover (rebuilt
		# below) reads these members for its rover_rc placement branch.
		sidecar._rover_rc_override = rc
		sidecar._rover_yaw = yaw_seq[k]

		# Rebuild ONLY the per-frame layer nodes (terrain/clasts/rover), leaving the
		# sun + WorldEnvironment in place — exactly the sidecar sequence-mode pattern.
		# This re-places the articulated rover (drums up, since --cameras-seq set
		# _drums_up) at the new rover_rc/yaw.
		sidecar._clear_frame_nodes()
		sidecar._build_layers()

		var rover_root = sidecar._find_rover_root()
		if rover_root == null:
			push_error("capture_seq: frame %d found no rover root after rebuild" % k)
			sidecar.get_tree().quit(4)
			return

		# Rover forward (+X local) in world, projected to the XZ plane (yaw only), so
		# the lander stands ahead of the rover on the surface (mirrors _cameras_capture).
		var rover_xf: Transform3D = rover_root.global_transform
		var fwd: Vector3 = rover_xf.basis * Vector3(1, 0, 0)
		fwd.y = 0.0
		if fwd.length() < 1e-5:
			fwd = Vector3(1, 0, 0)
		fwd = fwd.normalized()

		# Build the AprilTag-bearing procedural lander ahead of THIS frame's rover, via
		# the FROZEN shared sink (same call shape as _cameras_capture). The lander/tag
		# follow the moving rover so both front cameras keep seeing the id-0 face.
		var lander_root = SensorsEmitScript_ref(sidecar).build_lander(
			sidecar, sf, sidecar.AprilTagGenScript, rover_xf.origin, fwd,
			sidecar._lander_standoff, sidecar._lander_yaw_deg)

		# Build the front-stereo rig (shared-World3D SubViewports riding the rover) via
		# the FROZEN camera_rig.gd. Rigid extrinsics -> constant across frames by const.
		var world: World3D = sidecar.get_viewport().world_3d
		var cams: Array = sidecar.CameraRigScript.build(
			sidecar, rover_root, world, sidecar._viewport_size, sidecar._cam_pitch_deg)

		# Let the subviewports settle, then capture (first frame can sample a stale
		# buffer) — the EXACT proven pattern of sidecar.gd::_cameras_capture /
		# _probe_multicam_capture. A SubViewport sharing the world only renders the
		# freshly-(re)built terrain/rover/lander geometry after a REAL process+draw
		# frame elapses (verified: force_draw alone renders the environment background
		# but NOT geometry added the same tick — geometry registers into the world
		# scenario on the next tree frame). So we MUST await frame_post_draw here.
		#
		# REQUIRES the L0 dispatch to AWAIT this coroutine. sidecar.gd ~207 currently
		# calls `CaptureSeqScript.run_capture_seq(self)` un-awaited and then immediately
		# `get_tree().quit(0)` — which grants this coroutine exactly ONE post-quit frame
		# and tears the tree down before the sequence finishes (black frames). The
		# one-word fix is `await CaptureSeqScript.run_capture_seq(self)` (mirrors the
		# `await _cameras_capture()` single-frame call-site, sidecar.gd ~201). This is
		# recorded for the orchestrator in docs/lanes/M2-egress.md + integration_notes;
		# the verification harness (docs/lanes) drives this coroutine via `await`.
		for _w in range(3):
			await RenderingServer.frame_post_draw

		# --- write this frame's directory out/cam/<scene>/<NNN>/ -------------------
		var nnn := "%03d" % k
		var out_dir := "res://out/cam/%s/%s" % [scene, nnn]
		DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))
		for e in cams:
			var img: Image = e["sv"].get_texture().get_image()
			var path := "%s/%s" % [out_dir, e["image"]]
			var err := img.save_png(path)
			if err != OK:
				push_error("capture_seq: save_png failed (%d) for %s" % [err, path])
			else:
				print("capture_seq: frame %s wrote %s (%dx%d)" % [
					nnn, ProjectSettings.globalize_path(path), img.get_width(), img.get_height()])

		# --- assemble + write the per-frame sensors.json (FROZEN sink) -------------
		# REAL monotonic frame_index k (contract §2.2 widening); moving rover/camera
		# pose_in_world; intrinsics/baseline/extrinsics constant (read from frame 000).
		var sun = SensorsEmitScript_ref(sidecar).sun_block(
			sidecar._sun_elev_deg, sidecar._sun_azim_deg, 0.0)
		var doc = SensorsEmitScript_ref(sidecar).build_sensors_json(
			scene, k, sidecar._viewport_size, rover_root, lander_root, cams,
			Callable(sidecar.CameraRigScript, "intrinsics"), sidecar.CameraRigScript.FOV_X_DEG,
			sun, null, null)
		var json_path := "%s/sensors.json" % out_dir
		var jf := FileAccess.open(json_path, FileAccess.WRITE)
		if jf == null:
			push_error("capture_seq: cannot open %s for write" % json_path)
			sidecar.get_tree().quit(6)
			return
		jf.store_string(JSON.stringify(doc, "  "))
		jf.close()
		var rp: Array = doc["rover"]["position_m"]
		print("capture_seq: frame %s wrote %s frame_index=%d rover_pos=(%.3f,%.3f,%.3f) baseline_m=%.4f" % [
			nnn, ProjectSettings.globalize_path(json_path), int(doc["frame_index"]),
			float(rp[0]), float(rp[1]), float(rp[2]), float(doc["stereo"]["baseline_m"])])
		n_written += 1

	print("capture_seq: --cameras-seq wrote %d frames to %s" % [
		n_written, ProjectSettings.globalize_path("res://out/cam/%s" % scene)])

# The frozen schema-assembly sink, reached through the sidecar's preloaded const so we
# never add a res:// preload the sidecar already owns (camera_rig-style script reuse).
static func SensorsEmitScript_ref(sidecar):
	return sidecar.SensorsEmitScript

# Resolve the trajectory anchor rover_rc: prefer the loaded scene's rover_rc; else scan
# the scene dir's tNNN frames for the authored driven rover_rc (the final frame in the
# shipped tread scenes). Returns (-1,-1) if none.
static func _resolve_anchor(sidecar, sf) -> Vector2i:
	if sf.has_rover_rc and sf.rover_rc.x >= 0:
		return sf.rover_rc
	# Scan sibling tNNN frame dirs (mirrors sidecar._list_frames + _peek_rover_rc).
	var scene_dir: String = String(sidecar._scene_dir).trim_suffix("/")
	var parent := scene_dir.get_base_dir()
	# _scene_dir may already point at a frame dir OR a scene root that holds tNNN dirs.
	for base in [scene_dir, parent]:
		var d := DirAccess.open(base)
		if d == null:
			continue
		var names: Array = []
		d.list_dir_begin()
		var nm := d.get_next()
		while nm != "":
			if d.current_is_dir() and nm.begins_with("t") and nm.length() == 4 and nm.substr(1).is_valid_int():
				names.append(nm)
			nm = d.get_next()
		d.list_dir_end()
		names.sort()
		# Walk frames latest-first; the driven rover_rc lives on the final frame.
		names.reverse()
		for fn in names:
			var rc := _peek_rover_rc(base + "/" + fn)
			if rc.x >= 0:
				return rc
	return Vector2i(-1, -1)

# Read just rover_rc from a frame's metadata.json (mirrors sidecar._peek_rover_rc).
static func _peek_rover_rc(fdir: String) -> Vector2i:
	var f := FileAccess.open(fdir + "/metadata.json", FileAccess.READ)
	if f == null:
		return Vector2i(-1, -1)
	var parsed = JSON.parse_string(f.get_as_text())
	f.close()
	if typeof(parsed) != TYPE_DICTIONARY:
		return Vector2i(-1, -1)
	var rc = parsed.get("rover_rc", null)
	if typeof(rc) == TYPE_ARRAY and rc.size() == 2:
		return Vector2i(int(rc[0]), int(rc[1]))
	return Vector2i(-1, -1)

# Unit approach heading (drow, dcol) the rover travels INTO the anchor. If the scene
# has a real prior rover_rc (a true trajectory), use anchor - prior; else default to
# travel along grid +col (world +X) so the rover advances across the patch toward the
# anchor. Toward the patch interior is chosen when the anchor sits near an edge so the
# synthesised back-track stays on the authored terrain.
static func _approach_heading(sidecar, sf, anchor: Vector2i) -> Vector2:
	# Steer the heading so stepping BACKWARD from the anchor stays inside the grid:
	# point travel from the patch center toward the anchor (so backward = toward center).
	var cr := float(sf.height) * 0.5
	var cc := float(sf.width) * 0.5
	var dr := float(anchor.x) - cr
	var dc := float(anchor.y) - cc
	if absf(dr) < 1e-3 and absf(dc) < 1e-3:
		return Vector2(0.0, 1.0)          # degenerate: travel along +col (+X)
	var v := Vector2(dr, dc).normalized()
	return v
