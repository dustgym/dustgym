extends RefCounted
class_name LanderBundle
# OWNER LANE: M3-tag (4-face AprilTag bundle on the lander).
# NO-OP skeleton landed in the L0 contracts-first pass. The M3-tag lane fills this
# in; it NEVER edits sidecar.gd (the --lander-faces flag + dispatch call-site are
# already wired in sidecar.gd by L0).
#
# Contract (FROZEN v1.1 §3 + §6):
#   Build the 4-face tag bundle (ids 0..3, one per lander vertical face) and produce
#   the v1.1 OPTIONAL "lander"."apriltags":[{family,id,size_m,pose_in_lander}] that
#   the shared sink SensorsEmit.build_sensors_json(faces=...) emits. apriltags[]
#   SUPERSEDES the single apriltag{}; the FRONT face (id 0) MUST keep the existing
#   identity pose_in_lander so the M1 reading (12.7mm/7.15deg) is unchanged.
#   PER-FACE RELABEL (§6): R_face per face = rotation from lander axes into THAT
#   face's tag-quad axes, derived from each face's pose_in_lander basis (NOT the
#   single front-face frames.R_LANDER_TAG). The front face must reduce to
#   R_LANDER_TAG. M3-tag (Godot, defines the face quad orientations) and M3-bundle
#   (ROS, _compute_truth) must agree on this convention.
#
# This lane defines the Godot-side face quad orientations and hands the resulting
# faces[] to SensorsEmit; it reuses SensorsEmit.pose_dict for each pose_in_lander.

# Stub entry point (signature TBD by the M3-tag lane). Logs + returns. When
# implemented it builds the 4-face lander and returns the faces[] for the sink.
static func build_lander_faces(_sidecar: Node) -> Array:
	print("lander_bundle: stub (M3-tag lane)")
	return []
