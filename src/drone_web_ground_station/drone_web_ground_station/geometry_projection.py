"""Map/canvas conversion and target validation helpers."""

from __future__ import annotations

from typing import Iterable, Mapping, Tuple


Bounds2D = Tuple[float, float, float, float]


def canvas_to_map(
    pixel_x: float,
    pixel_y: float,
    width: float,
    height: float,
    bounds: Bounds2D,
) -> Tuple[float, float]:
    """Convert a top-view canvas pixel to map coordinates."""
    if width <= 0.0 or height <= 0.0:
        raise ValueError("canvas dimensions must be positive")
    x_min, x_max, y_min, y_max = bounds
    x = x_min + pixel_x / width * (x_max - x_min)
    y = y_max - pixel_y / height * (y_max - y_min)
    return x, y


def map_to_canvas(
    x: float,
    y: float,
    width: float,
    height: float,
    bounds: Bounds2D,
) -> Tuple[float, float]:
    """Convert map coordinates to a top-view canvas pixel."""
    if width <= 0.0 or height <= 0.0:
        raise ValueError("canvas dimensions must be positive")
    x_min, x_max, y_min, y_max = bounds
    if x_max <= x_min or y_max <= y_min:
        raise ValueError("invalid map bounds")
    pixel_x = (x - x_min) / (x_max - x_min) * width
    pixel_y = (y_max - y) / (y_max - y_min) * height
    return pixel_x, pixel_y


def point_inside_box(point: tuple[float, float, float], box: Mapping[str, object]) -> bool:
    """Return whether a point lies inside an axis-aligned marker box."""
    x, y, z = point
    return (
        abs(x - float(box.get("x", 0.0))) <= 0.5 * float(box.get("sx", 0.0))
        and abs(y - float(box.get("y", 0.0))) <= 0.5 * float(box.get("sy", 0.0))
        and abs(z - float(box.get("z", 0.0))) <= 0.5 * float(box.get("sz", 0.0))
    )


def validate_target(
    point: tuple[float, float, float],
    bounds: tuple[float, float, float, float, float, float],
    inflated_obstacles: Iterable[Mapping[str, object]],
) -> tuple[bool, str]:
    """Validate a clicked target against map bounds and inflated boxes."""
    x, y, z = point
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    if not (x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max):
        return False, "GOAL_OUT_OF_BOUNDS"
    if any(point_inside_box(point, box) for box in inflated_obstacles):
        return False, "GOAL_OCCUPIED"
    return True, "VALID"
