"""Challenge runner (M1 / H3) — run an agent against a challenge, emit a scorecard.

`run(agent, challenge)` realizes the challenge into a `TerrainTargetEnv`, rolls the
agent (a callable obs -> action) to termination/truncation, and returns a deterministic
`Scorecard`: success + primary metric + constraint usage + a composite score. Because
the env's physics authority conserves mass and runs deterministically, the score is
reproducible and unhackable. The same scorecard doubles as the RL reward summary and the
LAC-style competition metric.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

from . import challenge as chmod
from .terrain_target_env import TerrainTargetEnv

SLIP_EVENT_THRESHOLD = 0.5


@dataclass
class Scorecard:
    challenge_id: str
    objective_type: str
    success: bool
    primary_metric: float        # final region H-RMSE [m] (flatten/berm) or goal distance [cells] (traverse)
    steps: int
    slip_events: int
    total_reward: float
    score: float                 # composite comparator: +success - time - slip - quality_penalty

    def to_dict(self):
        return asdict(self)


def run(agent, challenge: chmod.Challenge, *, seed: int | None = None,
        slip_event_threshold: float = SLIP_EVENT_THRESHOLD,
        env_kwargs: dict | None = None) -> Scorecard:
    """Run ``agent`` (callable obs->action) against ``challenge``; return a Scorecard.

    ``seed`` overrides the challenge map seed (held-out evaluation); None uses the
    challenge's own seed. Deterministic for a deterministic agent.
    """
    env = TerrainTargetEnv(challenge, **(env_kwargs or {}))
    obs, info = env.reset(seed=seed)
    total = 0.0
    steps = 0
    slip_events = 0
    success = False
    final = info
    while True:
        obs, r, terminated, truncated, info = env.step(agent(obs))
        total += r
        steps += 1
        if info["slip"] > slip_event_threshold:
            slip_events += 1
        final = info
        if terminated or truncated:
            success = bool(info["success"])
            break

    obj = challenge.objective.type
    if obj == "traverse":
        primary = float(final["goal_dist"])
        quality_penalty = primary / challenge.map.grid       # normalized distance still to go
    else:
        primary = float(final["rmse"])
        quality_penalty = primary                            # metres of residual H-error

    s = challenge.scoring
    score = ((1.0 if success else 0.0)
             - s.w_time * steps
             - s.w_slip * slip_events
             - quality_penalty)
    return Scorecard(challenge_id=challenge.id, objective_type=obj, success=success,
                     primary_metric=primary, steps=steps, slip_events=slip_events,
                     total_reward=float(total), score=float(score))
