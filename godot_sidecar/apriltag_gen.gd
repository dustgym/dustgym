extends RefCounted
class_name AprilTagGen
# Procedural generator for the canonical tag36h11 id-0 fiducial marker
# (the G1<->C1 fiducial seam, docs/sensor_bridge_contract.md §1).
#
# Returns an ImageTexture of the 10x10-cell tag36h11 id-0 bitmap, scaled up
# nearest-neighbour (default 32 px/cell) so it stays crisp/aliasing-free when a
# camera resolves it. The 10x10 layout is the AprilRobotics apriltag-imgs
# convention:
#   * outer 1-cell ring  = WHITE quiet zone (OUTSIDE size_m; §1)
#   * next 1-cell ring   = BLACK border       \ together the 8x8 "black border
#   * inner 6x6          = the id-0 payload    / square" = the detector `size` = size_m
# So the printed marker spans 10 cells but the metric size_m (0.150 m) is the
# 8x8 black-border square; the white quiet zone adds (10/8) padding around it.
# build_tag_quad() bakes exactly that ratio into the quad geometry.
#
# Bitmap provenance (contract §1: "how the bitmap is produced is G1's choice"):
# the cells below are the EXACT pixels of the canonical
#   apriltag-imgs/tag36h11/tag36_11_00000.png   (10x10 px, 1 px/cell, BSD data)
# baked in as a 1/0 string grid so generation is dependency-free and offline
# (no network at render time). The integration test (C1 decodes this as
# family=tag36h11, id=0) is the acceptance check, not these pixels.
#
# A fiducial is matte print, NOT regolith: the tag quad is rendered UNLIT
# (StandardMaterial3D SHADING_MODE_UNSHADED, no Hapke) so its black/white cells
# stay high-contrast under the grazing ~5deg lunar sun.

# size_m (0.150) is the side of the 8x8 BLACK-BORDER square (the detector `size`
# parameter; contract §1). The full 10x10 printed marker is larger by 10/8 because
# of the 1-cell white quiet zone on each side.
const CELLS_TOTAL := 10            # full printed marker, incl. white quiet ring
const CELLS_BLACK_SQUARE := 8      # the black-border square == size_m
const QUIET_RATIO := float(CELLS_TOTAL) / float(CELLS_BLACK_SQUARE)  # 1.25

# Canonical tag36h11 id-0 cells (1 = white, 0 = black), row-major, top-left origin.
# Decoded verbatim from apriltag-imgs/tag36h11/tag36_11_00000.png.
const TAG36H11_ID0 := [
	"1111111111",
	"1000000001",
	"1011010101",
	"1001110101",
	"1001100001",
	"1010100001",
	"1001011001",
	"1000010001",
	"1000000001",
	"1111111111",
]

# Build the marker ImageTexture: each of the 10x10 cells expanded to
# px_per_cell x px_per_cell nearest-neighbour, so the result is
# (10*px_per_cell)^2 px of crisp black/white. RGB8, no mips (sharp at any scale
# the camera resolves; we keep it large so the 6x6 payload survives downsampling).
static func make_texture(px_per_cell: int = 32) -> ImageTexture:
	px_per_cell = maxi(px_per_cell, 1)
	var dim := CELLS_TOTAL * px_per_cell
	var img := Image.create(dim, dim, false, Image.FORMAT_RGB8)
	var white := Color(1, 1, 1)
	var black := Color(0, 0, 0)
	for cell_r in range(CELLS_TOTAL):
		var line: String = TAG36H11_ID0[cell_r]
		for cell_c in range(CELLS_TOTAL):
			var col: Color = white if line[cell_c] == "1" else black
			var x0 := cell_c * px_per_cell
			var y0 := cell_r * px_per_cell
			for y in range(y0, y0 + px_per_cell):
				for x in range(x0, x0 + px_per_cell):
					img.set_pixel(x, y, col)
	return ImageTexture.create_from_image(img)

# Build the lander-facing tag as a MeshInstance3D (a QuadMesh + UNLIT textured
# material), ready to parent onto the lander's rover-facing face.
#
# Geometry contract (§1):
#   * the QuadMesh side = size_m * QUIET_RATIO, so the 8x8 BLACK-BORDER square
#     (the detector `size`) spans exactly size_m and the white quiet ring sits
#     OUTSIDE it (not counted in size_m).
#   * the quad is centred on its local origin; the tag CENTER == this origin ==
#     (placed by the caller at) the lander origin, with the quad's local +Z =
#     the tag outward normal. pose_in_lander is therefore identity (§1).
#
# A QuadMesh faces local +Z by default, so the caller orients this node so +Z
# points back toward the rover.
static func build_tag_quad(size_m: float = 0.150, px_per_cell: int = 32) -> MeshInstance3D:
	var quad := QuadMesh.new()
	var full := size_m * QUIET_RATIO
	quad.size = Vector2(full, full)

	var mat := StandardMaterial3D.new()
	mat.albedo_texture = make_texture(px_per_cell)
	# UNLIT: a fiducial is matte print, not a Hapke regolith surface. Unshaded keeps
	# the cells pure black/white regardless of the grazing sun (so the detector sees
	# the designed contrast). texture_filter NEAREST keeps cell edges hard.
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
	# Show the printed face from the front only is unnecessary; keep it double-sided
	# so a slightly-off camera angle never sees a culled back.
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	quad.material = mat

	var mi := MeshInstance3D.new()
	mi.name = "AprilTag_id0"
	mi.mesh = quad
	return mi
