"""Pure waypoint generators for showcase missions."""

from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

from drone_map.collision_geometry import Point3


def hover(point: Sequence[float]) -> Tuple[Point3, ...]:
    """Return a single hover waypoint."""
    return (_point(point),)


def point(target: Sequence[float]) -> Tuple[Point3, ...]:
    """Return a single point-to-point target."""
    return (_point(target),)


def square(
    centre: Sequence[float],
    side_length: float,
) -> Tuple[Point3, ...]:
    """Return a closed, axis-aligned square at constant altitude."""
    x, y, z = _point(centre)
    half = 0.5 * _positive(side_length, "side_length")
    corners = (
        (x - half, y - half, z),
        (x + half, y - half, z),
        (x + half, y + half, z),
        (x - half, y + half, z),
    )
    return corners + (corners[0],)


def circle(
    centre: Sequence[float],
    radius: float,
    samples: int = 24,
) -> Tuple[Point3, ...]:
    """Return a closed circular waypoint sequence."""
    x, y, z = _point(centre)
    checked_radius = _positive(radius, "radius")
    checked_samples = _sample_count(samples)
    return tuple(
        (
            x + checked_radius * math.cos(
                2.0 * math.pi * index / checked_samples
            ),
            y + checked_radius * math.sin(
                2.0 * math.pi * index / checked_samples
            ),
            z,
        )
        for index in range(checked_samples + 1)
    )


def figure_eight(
    centre: Sequence[float],
    radius: float,
    samples: int = 32,
) -> Tuple[Point3, ...]:
    """Return a closed horizontal Gerono figure-eight."""
    x, y, z = _point(centre)
    checked_radius = _positive(radius, "radius")
    checked_samples = _sample_count(samples)
    result = tuple(
        (
            x + checked_radius * math.sin(
                2.0 * math.pi * index / checked_samples
            ),
            y + 0.5 * checked_radius * math.sin(
                4.0 * math.pi * index / checked_samples
            ),
            z,
        )
        for index in range(checked_samples + 1)
    )
    return result[:-1] + (result[0],)


def custom_path(
    waypoints: Iterable[Sequence[float]],
) -> Tuple[Point3, ...]:
    """Validate and return a user-provided waypoint sequence."""
    result = tuple(_point(value) for value in waypoints)
    if not result:
        raise ValueError("custom path must contain at least one waypoint")
    return result


def _point(values: Sequence[float]) -> Point3:
    point_value = tuple(float(value) for value in values)
    if (
        len(point_value) != 3
        or not all(math.isfinite(value) for value in point_value)
    ):
        raise ValueError("waypoint must contain three finite values")
    return point_value  # type: ignore[return-value]


def _positive(value: float, name: str) -> float:
    checked = float(value)
    if not math.isfinite(checked) or checked <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return checked


def _sample_count(value: int) -> int:
    checked = int(value)
    if checked < 4:
        raise ValueError("samples must be at least four")
    return checked
