"""WP0.0 (RB-02) — validate the requirement status manifest is well-formed and consistent with the PRD.

The manifest (`requirements_manifest.yaml`) is the generated status source (gap-analysis recommendation).
This test is its acceptance gate: every row carries the required fields, the status columns use the PRD
§3 vocabulary, ids are unique and match the PRD ID grammar, and every manifest id actually appears in
PRD.md (so the manifest cannot drift from the requirement document). No fabricated data — it checks the
real checked-in manifest against the real checked-in PRD.
"""
from __future__ import annotations

import os
import re

import pytest

yaml = pytest.importorskip("yaml")  # declared in .[dev]; skip cleanly if a bare env lacks it

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_MANIFEST = os.path.join(_ROOT, "requirements_manifest.yaml")
_PRD = os.path.join(_ROOT, "PRD.md")

_REQUIRED_FIELDS = {"id", "title", "priority", "I", "X", "V", "Q",
                    "owner", "source", "acceptance", "evidence", "blocked_by", "last_verified"}
_ID_RE = re.compile(r"^[A-Z]{2}-\d{2}$")          # RB-01, CT-03, VT-02, ...
_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")


def _load():
    with open(_MANIFEST) as fh:
        return yaml.safe_load(fh)


def test_manifest_header_is_well_formed():
    m = _load()
    assert m["schema_version"] == 1
    assert set(m["columns"]) == {"I", "X", "V", "Q"}
    assert set(m["status_values"]) == {"D", "P", "N", "G", "NA"}
    assert m["prd_version"] == "6.0"
    assert isinstance(m["requirements"], list) and m["requirements"]


def test_every_row_has_required_fields_and_valid_values():
    m = _load()
    statuses = set(m["status_values"])
    priorities = set(m["priorities"])
    for r in m["requirements"]:
        missing = _REQUIRED_FIELDS - set(r)
        assert not missing, f"{r.get('id')} missing fields {missing}"
        assert _ID_RE.match(r["id"]), f"bad id {r['id']!r}"
        assert r["priority"] in priorities, f"{r['id']} bad priority {r['priority']!r}"
        for col in ("I", "X", "V", "Q"):
            assert r[col] in statuses, f"{r['id']}.{col} = {r[col]!r} not in {statuses}"
        assert isinstance(r["blocked_by"], list)
        assert _DATE_RE.match(str(r["last_verified"])), f"{r['id']} bad last_verified"


def test_ids_unique_and_blocked_by_resolves():
    m = _load()
    ids = [r["id"] for r in m["requirements"]]
    assert len(ids) == len(set(ids)), "duplicate requirement ids"
    known = set(ids)
    for r in m["requirements"]:
        for dep in r["blocked_by"]:
            # a dependency may be a release blocker not yet in the (incremental) manifest; if it looks
            # like an id it must at least be well-formed.
            assert _ID_RE.match(dep), f"{r['id']} blocked_by malformed id {dep!r}"
            if dep.startswith("RB-"):
                assert dep in known, f"{r['id']} blocked_by unknown blocker {dep}"


def test_every_manifest_id_appears_in_the_prd():
    """The manifest may not invent requirement ids: each must be a real PRD id (no drift)."""
    prd = open(_PRD).read()
    m = _load()
    for r in m["requirements"]:
        assert re.search(rf"\b{re.escape(r['id'])}\b", prd), f"{r['id']} not found in PRD.md"


def test_all_release_blockers_are_tracked():
    """All six PRD §4.2 release blockers must be present and flagged blocks_release."""
    m = _load()
    rb = {r["id"]: r for r in m["requirements"] if r["id"].startswith("RB-")}
    assert set(rb) == {f"RB-0{i}" for i in range(1, 7)}, f"missing release blockers: {sorted(rb)}"
    for r in rb.values():
        assert r.get("blocks_release") is True, f"{r['id']} not flagged blocks_release"


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in fns:
        fn(); print(f"[PASS] {fn.__name__}")
    print(f"\n{len(fns)}/{len(fns)} requirement-manifest checks passed.")


if __name__ == "__main__":
    _run_all()
