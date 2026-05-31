extends RefCounted
class_name SunSweep
# OWNER LANE: A2-sweep (sun-elevation/azimuth sweep + per-frame boulder manifest).
# Fills the L0 NO-OP skeleton. NEVER edits sidecar.gd (the --sun-sweep flag + dispatch
# call-site are already wired in sidecar.gd by L0: sidecar.gd:211-214 awaits
# SunSweepScript.run_sun_sweep(self) AFTER _setup_environment() built the
# DirectionalLight3D + WorldEnvironment and _build_layers() built terrain+clasts).
#
# Contract (FROZEN docs/sun_sweep_manifest.md, sun_sweep/1.0):
#   out/sun_sweep/<scene>/{manifest.json, NNN.png}. A lunar-day sun-sweep renders a polar
#   boulder field under a MOVING sun (azimuth advances 360deg per synodic period; elevation
#   oscillates in the grazing 0-7deg polar band per the DOCUMENTED ASSUMPTION, §3) and emits
#   a dataset manifest for the visual-fiducial experiment. Each frame records sun
#   {azimuth_deg, elevation_deg} + time_delta_s; boulder poses + per-frame shadows come from
#   SunSweep -> BoulderManifest, reusing the FROZEN SensorsEmit.pose_dict / .sun_block.
#
# JOHN-DECISION cadence (binding): a GRAZING slice, n=12 frames, time_delta_s_step=88560.0
# (~one Earth-day slice of the synodic month), giving az 215->~245deg, el 0->~7deg.

# Companion boulder-manifest builder (OWNED sibling, same lane). Preloaded directly rather
# than via its global class_name: the class_name registry is not guaranteed visible while
# THIS script first compiles (verified — a bare `BoulderManifest` reference raises
# "Identifier not declared" at parse on Godot 4.6.3 here), so we preload the res:// path
# exactly like capture_seq.gd reaches its frozen seams. Both files are this lane's; the
# preload is internal to the lane and adds nothing to the frozen sidecar.
const BoulderManifestScript := preload("res://boulder_manifest.gd")

# --- §3 lunar-day sun MODEL constants (DOCUMENTED ASSUMPTION, not a mission value) ---
# Synodic month ~29.53 d ~= 708 h ~= 2.551e6 s — the standard published lunar synodic
# period (docs/sun_sweep_manifest.md:135-141). Used ONLY to set the azimuth sweep rate;
# NOT claimed to be the site's true solar track.
const T_SYNODIC_S := 2.551e6
# Grazing polar elevation band cap. SUN_ELEVATION_DEG_POLAR = 7.0
# (terrain_authority/constants.py:46, marked [FIXED] spec §5.1; a DOCUMENTED ASSUMPTION,
# the hillshade polar band, NOT a mission value). The sweep elevation oscillates in [0, EL_MAX].
const EL_MAX := 7.0

# JOHN-DECISION cadence: n=12 frames over a grazing slice. n is exposed via the EXISTING
# --stride flag (sidecar parses it into _seq_stride, sidecar.gd:437-438), mirroring how
# capture_seq reuses its stride member — so NO new sidecar flag is added. When --stride is
# left at its default (1, below the sweep's useful minimum) we hardcode n=12 (DEFAULT_N).
const DEFAULT_N := 12
# Per-frame time advance [s]: ~one Earth-day slice of the synodic month (JOHN-DECISION).
# 12 frames * 88560 s ~= 0.417 synodic period in TIME, but the elevation only reaches EL_MAX
# at the synodic half-period; over this front slice el rises 0 -> ~7deg and az 215 -> ~245deg
# (the §2 cadence invariants below are satisfied with the REALIZED frame[0]/frame[n-1] values).
const TIME_DELTA_S_STEP := 88560.0

# §3 azimuth model: linear, 360deg per synodic period, anchored at the sidecar default AZ0.
static func _azimuth_deg(t: float, az0: float) -> float:
	return fmod(az0 + 360.0 * t / T_SYNODIC_S, 360.0)

# §3 elevation model: raised half-cosine in the grazing band [0, EL_MAX]; el(0)=0 (sun on
# the horizon) rising toward EL_MAX at the synodic half-period, never exceeding the band.
static func _elevation_deg(t: float) -> float:
	return 0.5 * EL_MAX * (1.0 - cos(2.0 * PI * t / T_SYNODIC_S))

# Entry point dispatched + AWAITED from sidecar.gd:211-214 (after _setup_environment +
# _build_layers built the scene once). MUST keep the (sidecar) arity and be a coroutine
# (the body awaits frame_post_draw before each save — what prevents the post-quit
# black-render bug, exactly the capture_seq.gd:162-163 discipline).
static func run_sun_sweep(sidecar) -> void:
	var sf = sidecar.sf
	if sf == null:
		push_error("sun_sweep: --sun-sweep requires a loaded scene (--scene <dir>)")
		sidecar.get_tree().quit(2)
		return

	# The oblique whole-field camera (_setup_camera, sidecar.gd:548) is NOT called before
	# the --sun-sweep dispatch (it lives at sidecar.gd:220, AFTER), so we call it ourselves
	# before the first render. It honors --pose if the user gave one (_has_pose branch).
	sidecar._setup_camera()

	# Reach the single DirectionalLight3D by iterating the sidecar's children — the
	# established idiom at sidecar.gd:421/:699. We do NOT add a member ref to sidecar.
	var sun: DirectionalLight3D = null
	var n_suns := 0
	for ch in sidecar.get_children():
		if ch is DirectionalLight3D:
			sun = ch as DirectionalLight3D
			n_suns += 1
	assert(n_suns == 1, "sun_sweep: expected exactly one DirectionalLight3D, found %d" % n_suns)
	if sun == null:
		push_error("sun_sweep: no DirectionalLight3D found (was _setup_environment run?)")
		sidecar.get_tree().quit(3)
		return

	# n via the EXISTING --stride flag (mirror capture_seq's stride reuse); hardcode n=12
	# when --stride is left at its default (1) — the sweep needs >=2 and the John-decision
	# cadence is 12.
	var n: int = sidecar._seq_stride if sidecar._seq_stride >= 2 else DEFAULT_N

	# AZ0 anchors at the sidecar's live default azimuth _sun_azim_deg = 215.0 (§3).
	var az0: float = sidecar._sun_azim_deg
	# Larger world span [m] for the grazing-shadow length clamp (boulder_manifest §6).
	var field_extent_m: float = maxf(sf.extent_m().x, sf.extent_m().y)
	var scene: String = sf.scene_name

	# Output dir out/sun_sweep/<scene>/ (§1).
	var out_dir := "res://out/sun_sweep/%s" % scene
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))

	# (GDScript's % operator has no %e; format the synodic seconds as a plain float.)
	print("sun_sweep: --sun-sweep scene='%s' n=%d az0=%.3f el_max=%.1f time_step_s=%.1f T_synodic_s=%.1f" % [
		scene, n, az0, EL_MAX, TIME_DELTA_S_STEP, T_SYNODIC_S])

	# The FROZEN schema sink, reached through the sidecar's preloaded const (same script-reuse
	# pattern as capture_seq.gd:207-208) so we never add a res:// preload the sidecar owns.
	var sensors_emit = sidecar.SensorsEmitScript
	var pose_dict_fn := Callable(sensors_emit, "pose_dict")

	var frames: Array = []
	for k in range(n):
		var t: float = float(k) * TIME_DELTA_S_STEP          # seconds since sweep start (frame 0 == 0.0)
		var azim: float = _azimuth_deg(t, az0)
		var elev: float = _elevation_deg(t)

		# Drive the sun: rotation_degrees = Vector3(-elev, azim, 0) — matches the live sidecar
		# convention at sidecar.gd:516 (elevation is the angle ABOVE the horizon). Keep the
		# sidecar's live sun members in sync so SensorsEmit.sun_block reads the swept value.
		sun.rotation_degrees = Vector3(-elev, azim, 0.0)
		sidecar._sun_elev_deg = elev
		sidecar._sun_azim_deg = azim

		# Settle, then save the MAIN viewport via the FROZEN single-frame path sidecar._render_to
		# (terrain + boulders under the sun; the --cameras=off look). _render_to itself awaits
		# frame_post_draw; we await it so the dispatch's await actually drives this coroutine
		# through every frame (the post-quit black-render guard, capture_seq.gd:155-163).
		var nnn := "%03d" % k
		var img_path := "%s/%s.png" % [out_dir, nnn]
		var ok: bool = await sidecar._render_to(img_path)
		if not ok:
			push_error("sun_sweep: frame %d render/save failed -> %s" % [k, img_path])
			sidecar.get_tree().quit(6)
			return

		# Per-frame sun block via the FROZEN SensorsEmit.sun_block (sensors_emit.gd:256),
		# carrying the REAL per-frame time_delta_s off this lane's lunar-day model.
		var sun_b: Dictionary = sensors_emit.sun_block(elev, azim, t)
		# Per-frame boulders[] (poses + shadow) from the companion BoulderManifest (preloaded
		# sibling). Boulders are constant across frames; only the light/shadow move.
		var boulders: Array = BoulderManifestScript.build_boulders(sf, pose_dict_fn, azim, elev, field_extent_m)

		frames.append({
			"frame_index": k,
			"time_delta_s": t,
			"sun": {"azimuth_deg": azim, "elevation_deg": elev},
			"image": "%s.png" % nnn,
			"boulders": boulders,
		})
		print("sun_sweep: frame %s t=%.1fs az=%.3f el=%.3f boulders=%d -> %s" % [
			nnn, t, azim, elev, boulders.size(), ProjectSettings.globalize_path(img_path)])

	# --- assemble manifest.json with REALIZED cadence values (§2 invariants) -----------
	# cadence.n == len(frames); az0/el0 == frames[0].sun; az1/el1 == frames[n-1].sun. We read
	# them back off the produced frames so the recorded cadence cannot drift from the samples.
	var f0: Dictionary = frames[0]["sun"]
	var fN: Dictionary = frames[frames.size() - 1]["sun"]
	var manifest := {
		"schema_version": "sun_sweep/1.0",
		"scene": scene,
		"frame_convention": "godot",          # REP-103 stays C1's job (frames.py), §2
		# DOCUMENTED-ASSUMPTION flag: the §3 sun model is a cited stand-in, NOT a real
		# ephemeris (docs/sun_sweep_manifest.md:118-123). Stated plainly so a consumer never
		# mistakes these az/el for a mission solar track.
		"sun_model": {
			"documented_assumption": true,
			"description": "grazing-polar lunar-day stand-in (azimuth 360deg/synodic period; elevation raised half-cosine in [0, SUN_ELEVATION_DEG_POLAR]); illustrative, NOT a mission ephemeris",
			"T_synodic_s": T_SYNODIC_S,
			"el_max_deg": EL_MAX,
			"az0_deg": az0,
			"sources": [
				"terrain_authority/constants.py:46 SUN_ELEVATION_DEG_POLAR=7.0 [FIXED] spec 5.1",
				"sidecar.gd:114 _sun_azim_deg default 215.0",
				"lunar synodic month ~29.53 d ~= 2.551e6 s (standard published value)",
			],
		},
		"cadence": {
			"time_delta_s_step": TIME_DELTA_S_STEP,
			"az0": float(f0["azimuth_deg"]),
			"az1": float(fN["azimuth_deg"]),
			"el0": float(f0["elevation_deg"]),
			"el1": float(fN["elevation_deg"]),
			"n": frames.size(),
		},
		"frames": frames,
	}

	var json_path := "%s/manifest.json" % out_dir
	var jf := FileAccess.open(json_path, FileAccess.WRITE)
	if jf == null:
		push_error("sun_sweep: cannot open %s for write" % json_path)
		sidecar.get_tree().quit(6)
		return
	jf.store_string(JSON.stringify(manifest, "  "))
	jf.close()

	print("sun_sweep: --sun-sweep wrote %d frames + manifest -> %s (n=%d boulders/frame=%d az[%.2f..%.2f] el[%.2f..%.2f])" % [
		frames.size(), ProjectSettings.globalize_path(json_path), manifest["cadence"]["n"],
		(frames[0]["boulders"] as Array).size(),
		manifest["cadence"]["az0"], manifest["cadence"]["az1"],
		manifest["cadence"]["el0"], manifest["cadence"]["el1"]])
