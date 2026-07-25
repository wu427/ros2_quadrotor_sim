"""Deterministic random AABB-map generation with validator callbacks."""

from __future__ import annotations

import random
import math
from typing import Callable, Dict, List, Sequence, Tuple

MapPayload = Dict[str, object]
PathValidator = Callable[[MapPayload, Sequence[float], Sequence[float]], bool]


def generate_validated_map(
    seed: int,
    obstacle_count: int,
    start: Sequence[float],
    goal: Sequence[float],
    path_validator: PathValidator,
    max_attempts: int = 50,
    *,
    bounds: Sequence[Sequence[float]] = (
        (-1.0, -4.0, 0.0),
        (6.0, 4.0, 3.5),
    ),
    obstacle_size_range: Sequence[float] = (0.55, 0.95),
    start_clearance: float = 0.80,
    goal_clearance: float = 0.80,
    minimum_spacing: float = 0.35,
) -> MapPayload:
    """
    Generate a reproducible map and reject candidates without a valid path.

    The callback keeps ``drone_map`` independent from a planner package.
    Callers must pass the project's 3D A* validator before a map is accepted.
    """
    if obstacle_count < 0:
        raise ValueError("obstacle_count must be non-negative")
    if max_attempts <= 0:
        raise ValueError("max_attempts must be positive")
    checked_start = _point(start, "start")
    checked_goal = _point(goal, "goal")
    checked_bounds = (_point(bounds[0], "lower bounds"),
                      _point(bounds[1], "upper bounds"))
    if any(
        lower >= upper
        for lower, upper in zip(*checked_bounds)
    ):
        raise ValueError("map bounds must be ordered")
    if len(obstacle_size_range) != 2:
        raise ValueError("obstacle_size_range needs min and max")
    minimum_size, maximum_size = (
        float(value) for value in obstacle_size_range
    )
    if minimum_size <= 0.0 or maximum_size < minimum_size:
        raise ValueError("obstacle size range is invalid")
    for value, name in (
        (start_clearance, "start_clearance"),
        (goal_clearance, "goal_clearance"),
        (minimum_spacing, "minimum_spacing"),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    generator = random.Random(seed)
    for attempt in range(max_attempts):
        payload = _candidate(
            generator,
            seed,
            obstacle_count,
            attempt,
            checked_bounds,
            (minimum_size, maximum_size),
            checked_start,
            checked_goal,
            start_clearance,
            goal_clearance,
            minimum_spacing,
        )
        if payload is None:
            continue
        if path_validator(payload, checked_start, checked_goal):
            return payload
    raise RuntimeError(
        f"no valid random map after {max_attempts} attempts for seed {seed}"
    )


def _candidate(
    generator: random.Random,
    seed: int,
    obstacle_count: int,
    attempt: int,
    bounds: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
    size_range: Tuple[float, float],
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    start_clearance: float,
    goal_clearance: float,
    minimum_spacing: float,
) -> MapPayload | None:
    obstacles: List[Dict[str, object]] = []
    for index in range(obstacle_count):
        placed = False
        for _ in range(100):
            width = generator.uniform(*size_range)
            depth = generator.uniform(*size_range)
            height = generator.uniform(1.7, min(2.6, bounds[1][2]))
            centre_x = generator.uniform(
                bounds[0][0] + 0.5 * width,
                bounds[1][0] - 0.5 * width,
            )
            centre_y = generator.uniform(
                bounds[0][1] + 0.5 * depth,
                bounds[1][1] - 0.5 * depth,
            )
            candidate = {
                "id": f"seed_{seed}_attempt_{attempt}_{index:02d}",
                "center_x": round(centre_x, 3),
                "center_y": round(centre_y, 3),
                "center_z": round(0.5 * height, 3),
                "size_x": round(width, 3),
                "size_y": round(depth, 3),
                "size_z": round(height, 3),
            }
            if _point_box_distance(start, candidate) < start_clearance:
                continue
            if _point_box_distance(goal, candidate) < goal_clearance:
                continue
            if any(
                _box_spacing(candidate, existing) < minimum_spacing
                for existing in obstacles
            ):
                continue
            obstacles.append(candidate)
            placed = True
            break
        if not placed:
            return None
    return {
        "map": {
            "frame_id": "map",
            "x_min": bounds[0][0],
            "x_max": bounds[1][0],
            "y_min": bounds[0][1],
            "y_max": bounds[1][1],
            "z_min": bounds[0][2],
            "z_max": bounds[1][2],
            "drone_radius": 0.25,
            "safety_margin": 0.35,
            "grid_resolution": 0.25,
            "seed": seed,
            "obstacles": obstacles,
        }
    }


def _point(values: Sequence[float], name: str) -> Tuple[float, float, float]:
    checked = tuple(float(value) for value in values)
    if len(checked) != 3:
        raise ValueError(f"{name} must contain three values")
    return checked  # type: ignore[return-value]


def _point_box_distance(
    point_value: Sequence[float],
    box: Dict[str, object],
) -> float:
    squared = 0.0
    for axis, prefix in enumerate(("x", "y", "z")):
        centre = float(box[f"center_{prefix}"])
        half_size = 0.5 * float(box[f"size_{prefix}"])
        delta = max(abs(point_value[axis] - centre) - half_size, 0.0)
        squared += delta * delta
    return math.sqrt(squared)


def _box_spacing(
    first: Dict[str, object],
    second: Dict[str, object],
) -> float:
    squared = 0.0
    for prefix in ("x", "y", "z"):
        first_centre = float(first[f"center_{prefix}"])
        second_centre = float(second[f"center_{prefix}"])
        half_sizes = 0.5 * (
            float(first[f"size_{prefix}"])
            + float(second[f"size_{prefix}"])
        )
        delta = max(abs(first_centre - second_centre) - half_sizes, 0.0)
        squared += delta * delta
    return math.sqrt(squared)
