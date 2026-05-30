#!/usr/bin/env bash
# Convenience wrapper: headless-render the foss_ipex sidecar layer compositor.
#
# Calls render.sh with sidecar.tscn and forwards all CLI args after '--' to the
# sidecar (D4 layer-toggle CLI). Example:
#
#   ./render_layers.sh -- \
#       --scene ../samples/crater \
#       --layers terrain,clasts \
#       --pose 2.56,2.2,5.6,2.56,-0.1,2.56 \
#       --size 1024x768 \
#       --out crater_terrain.png
#
# Sidecar CLI args (everything after the literal '--'):
#   --scene <dir>            scene directory (INTERFACE.md layout)   [required]
#   --pose  x,y,z,tx,ty,tz   camera pos + look-at target (meters)
#   --layers a,b,c           heightmap|state|terrain|clasts|dust|distortion
#   --out   <png>            output (bare names land in out/)
#   --size  WxH              default 1024x768
#
# Output PNGs go to godot_sidecar/out/ (res://out/) unless --out is absolute.
set -euo pipefail
HERE="$(dirname "$(readlink -f "$0")")"
exec "$HERE/render.sh" "$HERE/sidecar.tscn" "$@"
