"""Collision-aware path smoothing, validation, and arc-length sampling."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import StaticMap


@dataclass(frozen=True)
class ProcessedPath:
    """Result of collision-aware path post-processing."""

    points: Tuple[Point3, ...]
    smoothing_used: bool
    smoothing_fallback: bool
    smoothed_points: int
    smoothed_length: float
    dense_check_step: float
    sample_spacing: float


def process_path(
    path: Sequence[Sequence[float]],
    static_map: StaticMap,
    smoothing_iterations: int = 2,
    dense_check_step: float = 0.05,
    sample_spacing: float = 0.25,
    minimum_z: float | None = None,
) -> ProcessedPath:
    """Smooth when safe, otherwise fall back, then resample by arc length."""
    source = _checked_path(path)
    if smoothing_iterations < 0:
        raise ValueError("smoothing_iterations must be non-negative")
    _positive(dense_check_step, "dense_check_step")
    _positive(sample_spacing, "sample_spacing")
    if not densely_collision_free(
        source, static_map, dense_check_step, minimum_z
    ):
        raise ValueError("input path is not collision free")

    selected = source
    smoothing_used = False
    for iterations in range(smoothing_iterations, 0, -1):
        candidate = chaikin_smooth(source, iterations)
        if densely_collision_free(
            candidate, static_map, dense_check_step, minimum_z
        ):
            selected = candidate
            smoothing_used = True
            break

    sampled = resample_by_arc_length(selected, sample_spacing)
    if not densely_collision_free(
        sampled, static_map, dense_check_step, minimum_z
    ):
        sampled = resample_by_arc_length(source, sample_spacing)
        smoothing_used = False
    if not densely_collision_free(
        sampled, static_map, dense_check_step, minimum_z
    ):
        raise RuntimeError("resampled path failed collision validation")
    return ProcessedPath(
        sampled,
        smoothing_used,
        smoothing_iterations > 0 and not smoothing_used,
        len(selected),
        _path_length(selected),
        dense_check_step,
        sample_spacing,
    )


def chaikin_smooth(
    path: Sequence[Sequence[float]],
    iterations: int = 1,
) -> Tuple[Point3, ...]:
    """Apply endpoint-preserving Chaikin corner cutting."""
    points = _checked_path(path)
    if iterations < 0:
        raise ValueError("iterations must be non-negative")
    for _ in range(iterations):
        if len(points) < 3:
            break
        result = [points[0]]
        for first, second in zip(points, points[1:]):
            result.append(_blend(first, second, 0.25))
            result.append(_blend(first, second, 0.75))
        result[-1] = points[-1]
        points = tuple(result)
    return points


def resample_by_arc_length(
    path: Sequence[Sequence[float]],
    spacing: float,
) -> Tuple[Point3, ...]:
    """Resample a polyline at approximately uniform arc-length spacing."""
    points = _checked_path(path)
    _positive(spacing, "spacing")
    if len(points) == 1:
        return points
    lengths = [
        math.dist(first, second)
        for first, second in zip(points, points[1:])
    ]
    total = sum(lengths)
    if total <= 1.0e-12:
        return (points[0],)
    targets = {
        min(index * spacing, total)
        for index in range(int(math.floor(total / spacing)) + 1)
    }
    elapsed = 0.0
    targets.add(0.0)
    for length in lengths:
        elapsed += length
        targets.add(elapsed)
    targets.add(total)

    result = []
    segment = 0
    elapsed = 0.0
    for target in sorted(targets):
        while (
            segment < len(lengths) - 1
            and elapsed + lengths[segment] < target
        ):
            elapsed += lengths[segment]
            segment += 1
        length = lengths[segment]
        ratio = 0.0 if length <= 1.0e-12 else (
            target - elapsed
        ) / length
        result.append(_blend(points[segment], points[segment + 1], ratio))
    result[0] = points[0]
    result[-1] = points[-1]
    return tuple(result)


def densely_collision_free(
    path: Sequence[Sequence[float]],
    static_map: StaticMap,
    step: float = 0.05,
    minimum_z: float | None = None,
) -> bool:
    """Check every segment at no more than ``step`` metres apart."""
    points = _checked_path(path)
    _positive(step, "step")
    for first, second in zip(points, points[1:]):
        if not static_map.segment_is_collision_free(first, second):
            return False
        samples = max(1, int(math.ceil(math.dist(first, second) / step)))
        for index in range(samples + 1):
            point = _blend(first, second, index / samples)
            if minimum_z is not None and point[2] < minimum_z - 1.0e-9:
                return False
            if not static_map.is_traversable(point):
                return False
    return all(
        minimum_z is None or point[2] >= minimum_z - 1.0e-9
        for point in points
    )


def _checked_path(
    path: Sequence[Sequence[float]],
) -> Tuple[Point3, ...]:
    if not path:
        raise ValueError("path must contain at least one point")
    result = []
    for value in path:
        point = tuple(float(component) for component in value)
        if len(point) != 3 or not all(math.isfinite(item) for item in point):
            raise ValueError("path points must contain three finite values")
        if not result or math.dist(result[-1], point) > 1.0e-12:
            result.append(point)
    return tuple(result)  # type: ignore[return-value]


def _blend(
    first: Sequence[float],
    second: Sequence[float],
    ratio: float,
) -> Point3:
    return tuple(
        first[axis] + ratio * (second[axis] - first[axis])
        for axis in range(3)
    )  # type: ignore[return-value]


def _positive(value: float, name: str) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")


def _path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(
        math.dist(first, second)
        for first, second in zip(path, path[1:])
    )
