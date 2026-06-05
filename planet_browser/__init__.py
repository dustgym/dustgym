"""planet_browser -- the dustgym mission planner + mission-control report product.

The build-planning layer over the conserved `terrain_authority` core: place build orders on a real lunar
map, optimize the sequence under physics + battery + time, and emit a mission-control report. Exposes the
HTTP server (`dustgym-serve` console entry point -> `planet_browser.server:main`).
"""
