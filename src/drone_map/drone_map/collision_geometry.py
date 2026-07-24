"""ROS-independent static-map and axis-aligned collision geometry."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
import math
from pathlib import Path
from typing import Iterable, Sequence, Tuple

import yaml

Point3 = Tuple[float, float, float]


def _point3(values: Iterable[float], name: str) -> Point3:
    point = tuple(float(value) for value in values)
    if len(point) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    if not all(math.isfinite(value) for value in point):
        raise ValueError(f"{name} must contain only finite values")
    return point  # type: ignore[return-value]


@dataclass(frozen=True)
class AABB:
    """Axis-aligned bounding box with inclusive collision boundaries."""

    obstacle_id: str
    minimum: Point3
    maximum: Point3

    def __post_init__(self) -> None:
        object.__setattr__(self, "minimum", _point3(self.minimum, "minimum"))
        object.__setattr__(self, "maximum", _point3(self.maximum, "maximum"))
        if not self.obstacle_id:
            raise ValueError("obstacle_id must not be empty")
        if any(
            lower >= upper
            for lower, upper in zip(self.minimum, self.maximum)
        ):
            raise ValueError("AABB maximum must be greater than minimum")

    @classmethod
    def from_center_size(
        cls,
        obstacle_id: str,
        center: Sequence[float],
        size: Sequence[float],
    ) -> "AABB":
        """Construct an AABB from a centre point and positive dimensions."""
        checked_center = _point3(center, "center")
        checked_size = _point3(size, "size")
        if any(value <= 0.0 for value in checked_size):
            raise ValueError("AABB dimensions must be positive")
        half = tuple(value * 0.5 for value in checked_size)
        minimum = tuple(
            checked_center[index] - half[index] for index in range(3)
        )
        maximum = tuple(
            checked_center[index] + half[index] for index in range(3)
        )
        return cls(obstacle_id, minimum, maximum)

    @property
    def center(self) -> Point3:
        """Return the box centre."""
        return tuple(
            0.5 * (lower + upper)
            for lower, upper in zip(self.minimum, self.maximum)
        )  # type: ignore[return-value]

    @property
    def size(self) -> Point3:
        """Return the box dimensions."""
        return tuple(
            upper - lower
            for lower, upper in zip(self.minimum, self.maximum)
        )  # type: ignore[return-value]

    def inflated(self, distance: float) -> "AABB":
        """Return a box expanded uniformly by ``distance``."""
        if not math.isfinite(distance) or distance < 0.0:
            raise ValueError("inflation distance must be finite and non-negative")
        minimum = tuple(value - distance for value in self.minimum)
        maximum = tuple(value + distance for value in self.maximum)
        return AABB(self.obstacle_id, minimum, maximum)

    def contains(self, point: Sequence[float]) -> bool:
        """Return whether a point lies inside or on the box boundary."""
        checked = _point3(point, "point")
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(
                checked, self.minimum, self.maximum
            )
        )

    def intersects_segment(
        self,
        start: Sequence[float],
        end: Sequence[float],
    ) -> bool:
        """Test a closed line segment against this box using slab clipping."""
        first = _point3(start, "segment start")
        second = _point3(end, "segment end")
        lower_t = 0.0
        upper_t = 1.0
        for index in range(3):
            delta = second[index] - first[index]
            if abs(delta) <= 1.0e-15:
                if not (
                    self.minimum[index]
                    <= first[index]
                    <= self.maximum[index]
                ):
                    return False
                continue
            entry = (self.minimum[index] - first[index]) / delta
            exit_ = (self.maximum[index] - first[index]) / delta
            if entry > exit_:
                entry, exit_ = exit_, entry
            lower_t = max(lower_t, entry)
            upper_t = min(upper_t, exit_)
            if lower_t > upper_t:
                return False
        return True

    def point_distance(self, point: Sequence[float]) -> float:
        """Return non-negative Euclidean distance from a point to the box."""
        checked = _point3(point, "point")
        squared = 0.0
        for value, lower, upper in zip(
            checked, self.minimum, self.maximum
        ):
            if value < lower:
                squared += (lower - value) ** 2
            elif value > upper:
                squared += (value - upper) ** 2
        return math.sqrt(squared)

    def segment_distance(
        self,
        start: Sequence[float],
        end: Sequence[float],
    ) -> float:
        """Return the exact minimum distance from a segment to the box."""
        first = _point3(start, "segment start")
        second = _point3(end, "segment end")
        if self.intersects_segment(first, second):
            return 0.0

        delta = tuple(second[index] - first[index] for index in range(3))
        breakpoints = {0.0, 1.0}
        for index in range(3):
            if abs(delta[index]) <= 1.0e-15:
                continue
            for boundary in (self.minimum[index], self.maximum[index]):
                parameter = (boundary - first[index]) / delta[index]
                if 0.0 < parameter < 1.0:
                    breakpoints.add(parameter)
        ordered = sorted(breakpoints)

        def squared_distance(parameter: float) -> float:
            point = tuple(
                first[index] + parameter * delta[index]
                for index in range(3)
            )
            distance = self.point_distance(point)
            return distance * distance

        best = min(squared_distance(value) for value in ordered)
        for interval_start, interval_end in zip(ordered, ordered[1:]):
            midpoint = 0.5 * (interval_start + interval_end)
            quadratic = 0.0
            linear = 0.0
            for index in range(3):
                midpoint_value = first[index] + midpoint * delta[index]
                if midpoint_value < self.minimum[index]:
                    offset = first[index] - self.minimum[index]
                elif midpoint_value > self.maximum[index]:
                    offset = first[index] - self.maximum[index]
                else:
                    continue
                quadratic += delta[index] * delta[index]
                linear += 2.0 * offset * delta[index]
            if quadratic > 0.0:
                candidate = -linear / (2.0 * quadratic)
                candidate = min(
                    interval_end, max(interval_start, candidate)
                )
                best = min(best, squared_distance(candidate))
        return math.sqrt(max(0.0, best))


@dataclass(frozen=True)
class StaticMap:
    """Validated immutable static-map collision model."""

    minimum: Point3
    maximum: Point3
    drone_radius: float
    safety_margin: float
    grid_resolution: float
    obstacles: Tuple[AABB, ...]
    frame_id: str = "map"

    def __post_init__(self) -> None:
        object.__setattr__(self, "minimum", _point3(self.minimum, "map minimum"))
        object.__setattr__(self, "maximum", _point3(self.maximum, "map maximum"))
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        if self.frame_id != "map":
            raise ValueError("static-map frame_id must be 'map'")
        if any(
            lower >= upper
            for lower, upper in zip(self.minimum, self.maximum)
        ):
            raise ValueError("map maximum must be greater than minimum")
        for name, value in (
            ("drone_radius", self.drone_radius),
            ("safety_margin", self.safety_margin),
            ("grid_resolution", self.grid_resolution),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.drone_radius <= 0.0:
            raise ValueError("drone_radius must be positive")
        if self.safety_margin < 0.0:
            raise ValueError("safety_margin must be non-negative")
        if self.grid_resolution <= 0.0:
            raise ValueError("grid_resolution must be positive")
        identifiers = [obstacle.obstacle_id for obstacle in self.obstacles]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("obstacle IDs must be unique")
        for obstacle in self.obstacles:
            if any(
                obstacle.minimum[index] < self.minimum[index]
                or obstacle.maximum[index] > self.maximum[index]
                for index in range(3)
            ):
                raise ValueError(
                    f"obstacle '{obstacle.obstacle_id}' is outside map bounds"
                )

    @property
    def inflation_distance(self) -> float:
        """Return drone-radius plus safety-margin inflation."""
        return self.drone_radius + self.safety_margin

    @cached_property
    def inflated_obstacles(self) -> Tuple[AABB, ...]:
        """Return collision boxes inflated for the vehicle envelope."""
        return tuple(
            obstacle.inflated(self.inflation_distance)
            for obstacle in self.obstacles
        )

    def contains_point(self, point: Sequence[float]) -> bool:
        """Return whether a point lies inside the inclusive map bounds."""
        checked = _point3(point, "point")
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(
                checked, self.minimum, self.maximum
            )
        )

    def is_occupied(self, point: Sequence[float]) -> bool:
        """Return whether a point is inside an inflated obstacle."""
        checked = _point3(point, "point")
        return any(
            obstacle.contains(checked)
            for obstacle in self.inflated_obstacles
        )

    def is_traversable(self, point: Sequence[float]) -> bool:
        """Return whether a point is in bounds and outside inflated boxes."""
        checked = _point3(point, "point")
        return self.contains_point(checked) and not self.is_occupied(checked)

    def segment_is_collision_free(
        self,
        start: Sequence[float],
        end: Sequence[float],
    ) -> bool:
        """Return whether a segment stays in bounds and avoids inflation."""
        first = _point3(start, "segment start")
        second = _point3(end, "segment end")
        return (
            self.contains_point(first)
            and self.contains_point(second)
            and not any(
                obstacle.intersects_segment(first, second)
                for obstacle in self.inflated_obstacles
            )
        )

    def path_collides(self, path: Sequence[Sequence[float]]) -> bool:
        """Return whether any point or segment in a polyline is unsafe."""
        if not path:
            return False
        checked = tuple(_point3(point, "path point") for point in path)
        if any(not self.is_traversable(point) for point in checked):
            return True
        return any(
            not self.segment_is_collision_free(start, end)
            for start, end in zip(checked, checked[1:])
        )

    def minimum_clearance(
        self,
        path: Sequence[Sequence[float]],
    ) -> float:
        """Return minimum path distance to original, uninflated obstacles."""
        if not path:
            raise ValueError("path must contain at least one point")
        checked = tuple(_point3(point, "path point") for point in path)
        if not self.obstacles:
            return math.inf
        if len(checked) == 1:
            return min(
                obstacle.point_distance(checked[0])
                for obstacle in self.obstacles
            )
        return min(
            obstacle.segment_distance(start, end)
            for start, end in zip(checked, checked[1:])
            for obstacle in self.obstacles
        )


def load_static_map(path: Path | str) -> StaticMap:
    """Load and validate a static-map YAML file."""
    map_path = Path(path)
    try:
        document = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"failed to read static map '{map_path}': {error}") from error
    if not isinstance(document, dict):
        raise ValueError("static-map YAML must contain a mapping")
    data = document.get("map", document)
    if not isinstance(data, dict):
        raise ValueError("static-map 'map' entry must be a mapping")

    required = (
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
        "drone_radius",
        "safety_margin",
        "grid_resolution",
        "obstacles",
    )
    missing = [name for name in required if name not in data]
    if missing:
        raise ValueError(f"missing static-map fields: {', '.join(missing)}")
    obstacle_data = data["obstacles"]
    if not isinstance(obstacle_data, list):
        raise ValueError("obstacles must be a list")

    obstacles = []
    for item in obstacle_data:
        if not isinstance(item, dict):
            raise ValueError("each obstacle must be a mapping")
        try:
            obstacle = AABB.from_center_size(
                str(item["id"]),
                (
                    item["center_x"],
                    item["center_y"],
                    item["center_z"],
                ),
                (
                    item["size_x"],
                    item["size_y"],
                    item["size_z"],
                ),
            )
        except KeyError as error:
            raise ValueError(
                f"obstacle is missing field {error.args[0]}"
            ) from error
        obstacles.append(obstacle)

    return StaticMap(
        minimum=(data["x_min"], data["y_min"], data["z_min"]),
        maximum=(data["x_max"], data["y_max"], data["z_max"]),
        drone_radius=float(data["drone_radius"]),
        safety_margin=float(data["safety_margin"]),
        grid_resolution=float(data["grid_resolution"]),
        obstacles=tuple(obstacles),
        frame_id=str(data.get("frame_id", "map")),
    )
