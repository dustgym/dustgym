extends Node3D
# Cube-on-plane headless render smoke test.
# Spec ref: validates §2 render path before any state-field handoff.

const OUT_PATH := "res://out/cube_on_plane.png"

func _ready() -> void:
	_build_scene()
	# Two post-draw waits: first frame may use a stale buffer in headless modes.
	await RenderingServer.frame_post_draw
	await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var err := img.save_png(OUT_PATH)
	if err != OK:
		push_error("save_png failed: %d" % err)
		get_tree().quit(1)
		return
	print("wrote ", ProjectSettings.globalize_path(OUT_PATH),
		" size=", img.get_width(), "x", img.get_height())
	get_tree().quit(0)

func _build_scene() -> void:
	# Lunar-grazing sun: 5° elevation, no ambient fill.
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-5.0, 30.0, 0.0)
	sun.light_energy = 1.3
	add_child(sun)

	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.02, 0.02, 0.03)  # near-black sky, no atmosphere
	e.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	env.environment = e
	add_child(env)

	# Ground plane — neutral regolith placeholder.
	var plane := MeshInstance3D.new()
	var pm := PlaneMesh.new()
	pm.size = Vector2(20, 20)
	plane.mesh = pm
	var pmat := StandardMaterial3D.new()
	pmat.albedo_color = Color(0.45, 0.42, 0.40)
	pmat.roughness = 0.95
	plane.material_override = pmat
	add_child(plane)

	# Cube — stand-in for a clast.
	var cube := MeshInstance3D.new()
	cube.mesh = BoxMesh.new()
	cube.position = Vector3(0, 0.5, 0)
	var cmat := StandardMaterial3D.new()
	cmat.albedo_color = Color(0.55, 0.52, 0.48)
	cmat.roughness = 0.9
	cube.material_override = cmat
	add_child(cube)

	var cam := Camera3D.new()
	cam.fov = 60.0
	add_child(cam)
	cam.look_at_from_position(Vector3(4, 3, 4), Vector3(0, 0.3, 0), Vector3.UP)
