"""Unit tests for the ROS-independent collision model."""

import math

import pytest

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap


def make_map() -> StaticMap:
    """Return a compact map shared by geometry tests."""
    return StaticMap(
        minimum=(-2.0, -2.0, 0.0),
        maximum=(2.0, 2.0, 3.0),
        drone_radius=0.2,
        safety_margin=0.3,
        grid_resolution=0.25,
        obstacles=(
            AABB.from_center_size(
                "box", (0.0, 0.0, 1.0), (1.0, 1.0, 1.0)
            ),
        ),
    )


def test_map_bounds_are_inclusive() -> None:
    static_map = make_map()
    assert static_map.contains_point((-2.0, -2.0, 0.0))
    assert static_map.contains_point((2.0, 2.0, 3.0))
    assert not static_map.contains_point((2.0001, 0.0, 1.0))


def test_point_occupancy_uses_inflated_box() -> None:
    static_map = make_map()
    assert static_map.is_occupied((0.9, 0.0, 1.0))
    assert not static_map.is_occupied((1.01, 0.0, 1.0))


def test_inflated_dimensions_include_vehicle_envelope() -> None:
    inflated = make_map().inflated_obstacles[0]
    assert inflated.minimum == pytest.approx((-1.0, -1.0, 0.0))
    assert inflated.maximum == pytest.approx((1.0, 1.0, 2.0))


def test_collision_free_segment() -> None:
    static_map = make_map()
    assert static_map.segment_is_collision_free(
        (-1.5, -1.5, 1.0), (1.5, -1.5, 1.0)
    )


def test_segment_through_obstacle() -> None:
    static_map = make_map()
    assert not static_map.segment_is_collision_free(
        (-1.5, 0.0, 1.0), (1.5, 0.0, 1.0)
    )


def test_segment_touching_inflated_boundary_collides() -> None:
    static_map = make_map()
    assert not static_map.segment_is_collision_free(
        (-1.5, 1.0, 1.0), (1.5, 1.0, 1.0)
    )


def test_point_distance_is_zero_inside_raw_obstacle() -> None:
    obstacle = make_map().obstacles[0]
    assert obstacle.point_distance((0.0, 0.0, 1.0)) == 0.0
    assert obstacle.point_distance((1.5, 0.0, 1.0)) == pytest.approx(1.0)


def test_segment_distance_handles_interior_minimum() -> None:
    obstacle = make_map().obstacles[0]
    distance = obstacle.segment_distance(
        (-1.0, 1.5, 1.0), (1.0, 1.5, 1.0)
    )
    assert distance == pytest.approx(1.0)


def test_path_minimum_clearance_uses_raw_obstacle() -> None:
    static_map = make_map()
    path = [(-1.5, 1.25, 1.0), (1.5, 1.25, 1.0)]
    assert static_map.minimum_clearance(path) == pytest.approx(0.75)


def test_path_collision_checks_every_segment() -> None:
    static_map = make_map()
    assert static_map.path_collides(
        [
            (-1.5, -1.5, 1.0),
            (-1.5, 0.0, 1.0),
            (1.5, 0.0, 1.0),
        ]
    )


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_input_is_rejected(value: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        make_map().contains_point((value, 0.0, 1.0))


def test_duplicate_obstacle_ids_are_rejected() -> None:
    obstacle = AABB.from_center_size(
        "same", (0.0, 0.0, 1.0), (0.5, 0.5, 0.5)
    )
    with pytest.raises(ValueError, match="unique"):
        StaticMap(
            (-2.0, -2.0, 0.0),
            (2.0, 2.0, 3.0),
            0.2,
            0.1,
            0.25,
            (obstacle, obstacle),
        )
