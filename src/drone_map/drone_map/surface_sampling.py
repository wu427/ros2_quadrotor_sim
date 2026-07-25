"""ROS-independent surface sampling for axis-aligned boxes."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import Point3


def sample_box_surfaces(
    obstacles: Iterable[AABB],
    spacing: float,
) -> Tuple[Point3, ...]:
    """Return deterministic, de-duplicated samples on all obstacle faces."""
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("spacing must be finite and positive")
    points = set()
    for obstacle in obstacles:
        axes = tuple(
            _axis_values(lower, upper, spacing)
            for lower, upper in zip(obstacle.minimum, obstacle.maximum)
        )
        for x in axes[0]:
            for y in axes[1]:
                points.add((x, y, obstacle.minimum[2]))
                points.add((x, y, obstacle.maximum[2]))
        for x in axes[0]:
            for z in axes[2]:
                points.add((x, obstacle.minimum[1], z))
                points.add((x, obstacle.maximum[1], z))
        for y in axes[1]:
            for z in axes[2]:
                points.add((obstacle.minimum[0], y, z))
                points.add((obstacle.maximum[0], y, z))
    return tuple(sorted(points))


def _axis_values(
    lower: float,
    upper: float,
    spacing: float,
) -> Tuple[float, ...]:
    intervals = max(1, int(math.ceil((upper - lower) / spacing)))
    return tuple(
        lower + (upper - lower) * index / intervals
        for index in range(intervals + 1)
    )
