# Plan -> render loop: visualize the earthwork and how much regolith must move

`scripts/plan_render_pipeline.py` is the core of the select-area -> plan -> render loop. It loads a
scene bundle (BEFORE) into the conserved ColumnState, plans a flatten of a central pad to a level grade
(cut the cells above target into the drum, fill the cells below from it -- mass-conserved), writes the
worked AFTER bundle, renders BEFORE and AFTER in Godot, and shows the earthwork with cut/fill volumes.

- **flatten_crater_boulders.png** - flatten a pad on the real `crater_boulders` patch. BEFORE shows the
  crater; AFTER shows it partially filled by the conserved plan; the earthwork panel shows the rim to
  cut (red, 0.19 m3) and the void to fill (blue, 0.42 m3). Mass-conserved: 228 kg cut = 228 kg filled,
  drum residual 0. The honest finding: flattening this crater needs more fill (0.42 m3 void) than the
  local cut yields (0.19 m3), so it would need imported material -- a real construction result.

The same machinery accepts any INTERFACE.md bundle, including a DEM window cropped from a user-selected
map area (`build_from_dem`), so wiring a browser `/render` endpoint on top closes the full loop. Feedback
re-render = rerun with a different pad or target.

Regenerate: `python3 scripts/plan_render_pipeline.py --scene samples/crater_boulders --out <dir>` (needs
the Godot binary under `roversim/.tools/`).
