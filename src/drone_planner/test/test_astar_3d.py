"""Unit tests for the ROS-independent three-dimensional A* planner."""

import math

import pytest

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap
from drone_planner.astar_3d import AStar3D


def make_map(obstacles=()) -> StaticMap:
    """Return a deterministic grid map for planner tests."""
    return StaticMap(
        minimum=(-1.0, -2.0, 0.0),
        maximum=(3.0, 2.0, 2.0),
        drone_radius=0.2,
        safety_margin=0.2,
        grid_resolution=0.25,
        obstacles=tuple(obstacles),
    )


def wall(obstacle_id: str, x: float, y: float = 0.0) -> AABB:
    """Return a full-height pillar used by tests."""
    return AABB.from_center_size(
        obstacle_id, (x, y, 1.0), (0.5, 0.8, 2.0)
    )


def test_world_grid_round_trip_returns_containing_cell_center() -> None:
    planner = AStar3D(make_map())
    index = planner.world_to_grid((0.13, -0.44, 0.77))
    centre = planner.grid_to_world(index)
    assert all(abs(a - b) <= 0.125 for a, b in zip(centre, (0.13, -0.44, 0.77)))


def test_exact_upper_bound_maps_to_final_cell() -> None:
    planner = AStar3D(make_map())
    assert planner.world_to_grid((3.0, 2.0, 2.0)) == (15, 15, 7)


def test_grid_index_outside_map_is_rejected() -> None:
    with pytest.raises(ValueError, match="outside"):
        AStar3D(make_map()).grid_to_world((16, 0, 0))


def test_unobstructed_astar_succeeds() -> None:
    static_map = make_map()
    result = AStar3D(static_map).plan((0.0, 0.0, 0.25), (2.5, 0.0, 1.25))
    assert result.success
    assert result.reason == "SUCCESS"
    assert result.path[0] == (0.0, 0.0, 0.25)
    assert result.path[-1] == (2.5, 0.0, 1.25)
    assert not static_map.path_collides(result.path)


def test_single_obstacle_requires_detour() -> None:
    static_map = make_map((wall("one", 1.0),))
    start = (0.0, 0.0, 0.25)
    goal = (2.5, 0.0, 1.25)
    result = AStar3D(static_map).plan(start, goal)
    assert result.success
    assert not static_map.segment_is_collision_free(start, goal)
    assert not static_map.path_collides(result.path)
    assert result.raw_path_length > math.dist(start, goal)


def test_multiple_obstacles_require_safe_route() -> None:
    static_map = make_map(
        (
            wall("one", 0.8, -0.3),
            wall("two", 1.7, 0.5),
        )
    )
    result = AStar3D(static_map).plan(
        (0.0, 0.0, 0.25), (2.7, 0.0, 1.25)
    )
    assert result.success
    assert not static_map.path_collides(result.path)


def test_start_inside_inflated_obstacle_fails() -> None:
    result = AStar3D(make_map((wall("one", 1.0),))).plan(
        (1.0, 0.0, 1.0), (2.5, 0.0, 1.0)
    )
    assert not result.success
    assert result.reason == "START_OCCUPIED"


def test_goal_inside_inflated_obstacle_fails() -> None:
    result = AStar3D(make_map((wall("one", 1.0),))).plan(
        (0.0, 0.0, 1.0), (1.0, 0.0, 1.0)
    )
    assert not result.success
    assert result.reason == "GOAL_OCCUPIED"


def test_goal_outside_map_fails() -> None:
    result = AStar3D(make_map()).plan(
        (0.0, 0.0, 1.0), (4.0, 0.0, 1.0)
    )
    assert not result.success
    assert result.reason == "GOAL_OUT_OF_BOUNDS"


def test_fully_sealed_map_returns_no_path() -> None:
    sealed = AABB.from_center_size(
        "sealed", (1.0, 0.0, 1.0), (0.5, 4.0, 2.0)
    )
    result = AStar3D(make_map((sealed,))).plan(
        (0.0, 0.0, 1.0), (2.5, 0.0, 1.0)
    )
    assert not result.success
    assert result.reason == "NO_PATH"


def test_max_expansion_limit_is_reported() -> None:
    result = AStar3D(
        make_map(),
        max_expanded_nodes=1,
        planning_timeout_sec=1.0,
    ).plan((0.0, 0.0, 0.25), (2.5, 1.5, 1.5))
    assert not result.success
    assert result.reason == "MAX_EXPANSIONS"


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_non_finite_start_is_rejected(value: float) -> None:
    result = AStar3D(make_map()).plan(
        (value, 0.0, 1.0), (2.0, 0.0, 1.0)
    )
    assert not result.success
    assert result.reason == "START_NON_FINITE"
