"""Challenge profiles that preserve a runnable fallback at every stage."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


StateSource = Literal["onboard", "external_kalman"]
ControllerSource = Literal["onboard_position", "outer_loop"]
SpeedPolicy = Literal["fixed", "covariance"]
CrossingPolicy = Literal["centre", "optimized"]


@dataclass(frozen=True)
class ChallengeProfile:
    name: str
    state_source: StateSource
    controller: ControllerSource
    speed_policy: SpeedPolicy
    crossing_policy: CrossingPolicy
    fixed_speed_m_s: float


# Instructor-validated minimum viable race. Keep this profile unchanged while
# developing more advanced modes so there is always a known recovery path.
FALLBACK = ChallengeProfile(
    name="fallback",
    state_source="onboard",
    controller="onboard_position",
    speed_policy="fixed",
    crossing_policy="centre",
    fixed_speed_m_s=0.30,
)

# Exercises the external estimator without making flight safety depend on the
# participant outer-loop controller.
ESTIMATION_ONLY = ChallengeProfile(
    name="estimation_only",
    state_source="external_kalman",
    controller="onboard_position",
    speed_policy="fixed",
    crossing_policy="centre",
    fixed_speed_m_s=0.30,
)

# Full challenge target after simulation and tethered/low-speed qualification.
FULL = ChallengeProfile(
    name="full",
    state_source="external_kalman",
    controller="outer_loop",
    speed_policy="covariance",
    crossing_policy="optimized",
    fixed_speed_m_s=0.30,
)


PROFILES = {profile.name: profile for profile in (FALLBACK, ESTIMATION_ONLY, FULL)}


def get_profile(name: str) -> ChallengeProfile:
    try:
        return PROFILES[name]
    except KeyError as exc:
        raise ValueError(f"unknown profile {name!r}; choose from {sorted(PROFILES)}") from exc
