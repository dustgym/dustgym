#!/usr/bin/env python3
"""Live UI eval harness — drive the planet_browser cockpit in a real headless browser, screenshot every
view pane + a planned mission, and assert each rendered with no page errors.

Run on demand (needs google-chrome + the `playwright` python package; uses swiftshader so it works without a
GPU). NOT a CI unit test — it launches a real browser against a running server, like the Godot/COLMAP eval
scripts. Usage:

    # start the server in one shell:
    PYTHONNOUSERSITE=1 PYTHONPATH=. <venv>/bin/python -m planet_browser.server --host 127.0.0.1 --port 8797
    # then drive + screenshot it:
    <venv>/bin/python scripts/ui_eval.py --url http://127.0.0.1:8797 --out validation/ui

Exits non-zero if any pane fails to render or the page logs a JS error. Writes <out>/<pane>.png + a
machine-readable <out>/ui_eval.json summary.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from playwright.sync_api import sync_playwright

PANES = ["plan", "perception", "metrics", "report", "validation", "api", "server", "config"]


def main() -> int:
    ap = argparse.ArgumentParser(description="live cockpit screenshot eval")
    ap.add_argument("--url", default="http://127.0.0.1:8797")
    ap.add_argument("--out", default="validation/ui")
    ap.add_argument("--channel", default="chrome")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    results: dict = {"url": args.url, "panes": {}, "errors": []}

    with sync_playwright() as p:
        browser = p.chromium.launch(channel=args.channel, headless=True,
                                    args=["--use-gl=swiftshader", "--no-sandbox"])
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        page.on("pageerror", lambda e: results["errors"].append(str(e)))
        page.goto(args.url, wait_until="networkidle", timeout=40000)
        page.wait_for_selector("#viewtabs .vtab", timeout=15000)

        # Plan a real mission first so Report/Metrics have content. Add a structure + a keep-out, then plan.
        page.eval_on_selector("#struct", "el => el.value = 'blast_berm'")
        page.click("#qstruct")                                   # blast berm -> mass-balanced orders
        page.fill("#kox", "20"); page.fill("#koy", "0"); page.fill("#kor", "8"); page.click("#koadd")
        page.click("#qplan")
        page.wait_for_function("document.getElementById('reportframe').classList.contains('show')", timeout=40000)
        status = page.eval_on_selector("#qstatus", "el => el.textContent")
        results["plan_status"] = status

        # Execute + watch so the Metrics canvas has a live frame, then screenshot every pane.
        if not page.eval_on_selector("#qexec", "el => el.disabled"):
            page.click("#qexec")
            page.wait_for_timeout(800)                           # let a few animation frames render

        for pane in PANES:
            page.click(f'.vtab[data-view="{pane}"]')
            page.wait_for_timeout(700)                           # let the pane load (figures/iframe/poll)
            active = page.eval_on_selector_all(".pane.active", "els => els.map(e => e.id)")
            shot = os.path.join(args.out, f"{pane}.png")
            page.screenshot(path=shot)
            ok = (pane == "plan" and active == []) or (pane != "plan" and len(active) == 1)
            results["panes"][pane] = {"screenshot": shot, "active_panes": active, "ok": bool(ok)}

        browser.close()

    with open(os.path.join(args.out, "ui_eval.json"), "w") as fh:
        json.dump(results, fh, indent=2)

    bad = [k for k, v in results["panes"].items() if not v["ok"]]
    print(f"plan: {results.get('plan_status', '')[:90]}")
    for k, v in results["panes"].items():
        print(f"  {'OK ' if v['ok'] else 'FAIL'} {k:11s} -> {v['screenshot']}  active={v['active_panes']}")
    print(f"page errors: {results['errors'] if results['errors'] else 'none'}")
    if bad or results["errors"]:
        print(f"EVAL FAILED: panes={bad} errors={len(results['errors'])}")
        return 1
    print(f"EVAL PASSED: {len(PANES)} panes rendered, no page errors")
    return 0


if __name__ == "__main__":
    sys.exit(main())
