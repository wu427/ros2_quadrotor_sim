"""Line-of-sight simplification for collision-free polylines."""

from __future__ import annotations

from typing import Sequence, Tuple

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import StaticMap
from drone_planner.astar_3d import path_length


def simplify_path(
    path: Sequence[Sequence[float]],
    static_map: StaticMap,
) -> Tuple[Point3, ...]:
    """Greedily connect each waypoint to the farthest visible successor."""
    if not path:
        return ()
    checked = tuple(
        tuple(float(value) for value in point)
        for point in path
    )
    if static_map.path_collides(checked):
        raise ValueError("input path must be collision-free")
    if len(checked) <= 2:
        return checked  # type: ignore[return-value]

    simplified = [checked[0]]
    current = 0
    while current < len(checked) - 1:
        candidate = len(checked) - 1
        while candidate > current + 1:
            if static_map.segment_is_collision_free(
                checked[current], checked[candidate]
            ):
                break
            candidate -= 1
        simplified.append(checked[candidate])
        current = candidate

    result = tuple(simplified)
    if static_map.path_collides(result):
        raise RuntimeError("simplified path failed collision revalidation")
    if path_length(result) > path_length(checked) + 1.0e-9:
        raise RuntimeError("simplification increased path length")
    return result  # type: ignore[return-value]
