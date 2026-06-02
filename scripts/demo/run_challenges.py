#!/usr/bin/env python3
"""Run the authored challenge library with baseline agents -> a leaderboard (M1 demo).

The "plan challenges" capability end to end:
    authored Challenge -> realize(seed) -> map + target -> run(agent) -> Scorecard.

Baseline agents are scripted (no RL lib needed); a trained policy would slot in as just
another `obs -> action` callable. Run from the repo root:
    python scripts/demo/run_challenges.py [--out scorecards.json]
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from terrain_authority import challenge as ch          # noqa: E402
from terrain_authority import challenge_runner as cr    # noqa: E402

AGENTS = {
    "noop": lambda obs: [0.0, 0.0, 0.0],
    "forward": lambda obs: [1.0, 0.0, 0.0],
    "dig_drive": lambda obs: [0.6, 0.0, 1.0],            # drive while cutting
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", help="write scorecards as JSON")
    args = ap.parse_args()

    challenges = ch.authored_challenges()
    rows = []
    hdr = f"{'challenge':14} {'tier':>4} {'agent':10} {'ok':>5} {'primary':>9} {'steps':>5} {'slip':>4} {'score':>8}"
    print(hdr)
    print("-" * len(hdr))
    for c in challenges:
        for name, agent in AGENTS.items():
            sc = cr.run(agent, c)
            rows.append({"agent": name, **sc.to_dict()})
            print(f"{c.id:14} {c.difficulty_tier:>4} {name:10} {str(sc.success):>5} "
                  f"{sc.primary_metric:9.4f} {sc.steps:5d} {sc.slip_events:4d} {sc.score:8.3f}")
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(rows, fh, indent=2)
        print("saved", args.out)


if __name__ == "__main__":
    main()
