"""Pure patrol-sequence helpers shared by ROS and UI layers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


SUPPORTED_PATROL_MODES = {"ONCE", "LOOP", "PING_PONG", "LAPS", "TIMED"}
SUPPORTED_FINAL_ACTIONS = {"HOLD", "RETURN_HOME"}


@dataclass(frozen=True)
class PatrolConfig:
    """Validated patrol execution settings."""

    mode: str = "ONCE"
    laps: int = 1
    duration_sec: float = 0.0
    dwell_sec: float = 0.0
    final_action: str = "HOLD"
    home: Tuple[float, float, float, float] = (0.0, 0.0, 1.5, 0.0)

    @classmethod
    def from_mapping(cls, value: object) -> "PatrolConfig":
        """Build a configuration from a JSON-like mapping."""
        if not isinstance(value, dict):
            raise ValueError("patrol config must be an object")
        mode = str(value.get("mode", "ONCE")).upper()
        if mode not in SUPPORTED_PATROL_MODES:
            raise ValueError(f"unsupported patrol mode: {mode}")
        laps = int(value.get("laps", 1))
        if laps < 0:
            raise ValueError("laps must be zero or positive")
        if mode in {"ONCE", "LAPS"} and laps == 0:
            laps = 1
        duration_sec = float(value.get("duration_sec", 0.0))
        if duration_sec < 0.0:
            raise ValueError("duration_sec must be non-negative")
        if mode == "TIMED" and duration_sec <= 0.0:
            raise ValueError("TIMED patrol requires duration_sec > 0")
        dwell_sec = float(value.get("dwell_sec", 0.0))
        if dwell_sec < 0.0:
            raise ValueError("dwell_sec must be non-negative")
        final_action = str(value.get("final_action", "HOLD")).upper()
        if final_action not in SUPPORTED_FINAL_ACTIONS:
            raise ValueError(f"unsupported final action: {final_action}")
        raw_home = value.get("home", (0.0, 0.0, 1.5, 0.0))
        if not isinstance(raw_home, (list, tuple)) or len(raw_home) not in {3, 4}:
            raise ValueError("home must contain x, y, z[, yaw]")
        home_values = tuple(float(item) for item in raw_home)
        if len(home_values) == 3:
            home_values = home_values + (0.0,)
        return cls(
            mode=mode,
            laps=laps,
            duration_sec=duration_sec,
            dwell_sec=dwell_sec,
            final_action=final_action,
            home=home_values,
        )


def patrol_cycle_indices(count: int, mode: str) -> Tuple[int, ...]:
    """Return one traversal cycle for the requested mode."""
    if count < 2:
        raise ValueError("patrol requires at least two waypoints")
    normalized = mode.upper()
    if normalized == "PING_PONG":
        return tuple(range(count)) + tuple(range(count - 2, -1, -1))
    if normalized in {"ONCE", "LOOP", "LAPS", "TIMED"}:
        return tuple(range(count))
    raise ValueError(f"unsupported patrol mode: {mode}")


def completed_fraction(
    completed_segments: int,
    cycle_size: int,
    current_lap: int,
    config: PatrolConfig,
) -> float:
    """Return a bounded best-effort patrol progress value."""
    if cycle_size <= 0:
        return 0.0
    if config.mode == "ONCE":
        return min(1.0, completed_segments / cycle_size)
    if config.mode == "LAPS" or (config.mode == "LOOP" and config.laps > 0):
        total = max(1, config.laps) * cycle_size
        done = max(0, current_lap - 1) * cycle_size + completed_segments
        return min(1.0, done / total)
    return min(0.999, completed_segments / cycle_size)


def should_continue_after_cycle(
    config: PatrolConfig,
    completed_laps: int,
    elapsed_sec: float,
) -> bool:
    """Decide whether another patrol cycle should start."""
    if config.mode == "ONCE":
        return False
    if config.mode == "TIMED":
        return elapsed_sec < config.duration_sec
    if config.mode == "LOOP" and config.laps == 0:
        return True
    return completed_laps < max(1, config.laps)
