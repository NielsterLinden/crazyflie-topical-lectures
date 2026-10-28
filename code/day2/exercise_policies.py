"""Small participant-owned decisions for E4 and E5. No logging or plotting here."""

from dataclasses import dataclass

from scipy.stats import chi2  # Available for the E4 decision below.


@dataclass(frozen=True)
class StationaryWindow:
    """Causal, one-second sensor summaries supplied by the experiment runner."""

    duration_s: float
    gyro_max_rad_s: float
    acceleration_norm_mean_m_s2: float
    acceleration_norm_std_m_s2: float
    motors_idle: bool
    flow_speed_m_s: float | None


def innovation_threshold(dimension: int, false_rejection_probability: float) -> float | None:
    """E4: return the NIS threshold for this measurement dimension.

    Use the worksheet's chi-square rule. Think about the assumptions that make
    that rule applicable. None deliberately leaves the starter ungated.
    """
    # TODO E4: replace this return with your dimension-aware threshold.
    return None


def stationary_update_allowed(window: StationaryWindow) -> bool:
    """E5: decide whether these observations justify a zero-velocity update.

    Choose and justify thresholds in the units above. Include window length,
    gyro, acceleration, motor status and an available independent motion cue.
    Quiet IMU readings also occur during constant-velocity translation. A missing
    flow_speed_m_s means there is no fresh flow evidence in this experiment.
    """
    # TODO E5: replace this conservative baseline with your stationary decision.
    return False
