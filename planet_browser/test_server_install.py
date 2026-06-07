"""WP0.6 (RB-06) — the installed `dustgym[server]` must actually run.

The server imports the mission planner, which imports matplotlib AT MODULE LOAD, and the latlon
site-pick uses pyproj. So `pip install dustgym[server]` only works if the server extra declares those.
This guards that the extra covers the server's import graph, and that the app + console entrypoint +
registered envs import cleanly. (The full fresh-wheel clean-venv build smoke is a heavier follow-on.)
"""
from __future__ import annotations

import os

try:
    import tomllib                                       # py3.11+
except ModuleNotFoundError:                              # pragma: no cover - py3.10
    import tomli as tomllib  # type: ignore

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _extras():
    with open(os.path.join(_ROOT, "pyproject.toml"), "rb") as fh:
        return tomllib.load(fh)["project"]["optional-dependencies"]


def test_server_extra_covers_the_server_import_graph():
    # RB-06: a fresh `pip install dustgym[server]` ImportErrors on `import planet_browser.server`
    # unless the extra carries the planner's import-time deps. matplotlib is required at module load.
    server = " ".join(_extras()["server"]).lower()
    for dep in ("fastapi", "uvicorn", "matplotlib", "pyproj"):
        assert dep in server, f"server extra is missing the import-time dep {dep!r} (RB-06)"


def test_server_app_and_entrypoint_import():
    # the FastAPI app constructs and the console_scripts target (dustgym-serve = server:main) resolves.
    from planet_browser import server as srv
    assert srv.app is not None
    assert callable(srv.main)                            # pyproject [project.scripts] dustgym-serve -> server:main


def test_registered_envs_import_and_make():
    # importing dustgym registers the gym envs; the import graph for every registered Dust/* env is intact.
    gym = __import__("gymnasium")
    import dustgym  # noqa: F401  (registers on import)
    dust_ids = [k for k in gym.envs.registry if str(k).startswith("Dust/")]
    assert dust_ids, "no Dust/* environments registered on `import dustgym`"
    env = gym.make(dust_ids[0])                          # the first registered env constructs
    env.reset(seed=0)
    env.close()


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} server-install checks passed.")


if __name__ == "__main__":
    _run_all()
