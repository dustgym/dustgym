extends RefCounted
class_name CaptureSeq
# OWNER LANE: M2-egress (multi-frame camera sequence egress).
# NO-OP skeleton landed in the L0 contracts-first pass. The M2-egress lane fills
# this in; it NEVER edits sidecar.gd (the --cameras-seq flag + dispatch call-site
# are already wired in sidecar.gd by L0).
#
# Contract (FROZEN v1.1 §7 multi-frame egress dir convention):
#   out/cam/<scene>/<NNN>/{front_left,front_right,...}.png + a per-frame sensors.json
#   carrying the REAL monotonic frame_index + the per-frame rover pose_in_world;
#   intrinsics/baseline constant across frames; <NNN> zero-padded 3 digits from 000.
# The --cameras-seq flag inherits the live --cameras side effect (_drums_up=true) so
# the drum arms clear the front-stereo FOV (wired in sidecar.gd::_parse_args).
#
# When implemented this will assemble each frame's sensors.json via the shared sink
# SensorsEmit.build_sensors_json(...) (passing the running frame_index), reusing the
# same camera_rig.gd rig + SensorsEmit.build_lander(...) the single-frame --cameras
# path uses, so the schema stays identical across the M1/M2 egress.

# Stub entry point (signature TBD by the M2-egress lane). Logs + returns; performs
# no capture. Async (await) shape mirrors sidecar.gd::_cameras_capture so wiring the
# real implementation is a drop-in.
static func run_capture_seq(_sidecar: Node) -> void:
	print("capture_seq: stub (M2-egress lane)")
