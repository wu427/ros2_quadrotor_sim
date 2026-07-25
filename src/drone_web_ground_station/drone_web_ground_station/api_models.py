"""Small JSON validation helpers for the web API."""

from __future__ import annotations

import math
from typing import Mapping, Sequence


def finite_number(value: object, name: str) -> float:
    """Return one finite float or raise a user-facing ValueError."""
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def parse_goal(value: object) -> tuple[float, float, float, float]:
    """Parse x/y/z/yaw from an API object or sequence."""
    if isinstance(value, Mapping):
        return (
            finite_number(value.get("x"), "x"),
            finite_number(value.get("y"), "y"),
            finite_number(value.get("z"), "z"),
            finite_number(value.get("yaw", 0.0), "yaw"),
        )
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        if len(value) not in {3, 4}:
            raise ValueError("goal must contain x, y, z[, yaw]")
        values = [finite_number(item, "goal component") for item in value]
        if len(values) == 3:
            values.append(0.0)
        return tuple(values)  # type: ignore[return-value]
    raise ValueError("goal must be an object or numeric array")


def parse_waypoints(value: object) -> list[tuple[float, float, float, float]]:
    """Parse and validate an ordered waypoint array."""
    if not isinstance(value, list):
        raise ValueError("waypoints must be an array")
    result = [parse_goal(item) for item in value]
    if not result:
        raise ValueError("waypoints must not be empty")
    return result
