"""Unit tests for line-of-sight path simplification."""

import pytest

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap
from drone_planner.astar_3d import path_length
from drone_planner.path_simplifier import simplify_path


def make_map() -> StaticMap:
    """Return a map whose central inflated box blocks direct visibility."""
    return StaticMap(
        (-2.0, -2.0, 0.0),
        (2.0, 2.0, 2.0),
        0.2,
        0.2,
        0.25,
        (
            AABB.from_center_size(
                "box", (0.0, 0.0, 1.0), (0.6, 0.6, 2.0)
            ),
        ),
    )


def test_simplifier_preserves_start_and_goal() -> None:
    path = (
        (-1.5, 0.0, 1.0),
        (-1.0, 1.2, 1.0),
        (0.0, 1.2, 1.0),
        (1.0, 1.2, 1.0),
        (1.5, 0.0, 1.0),
    )
    simplified = simplify_path(path, make_map())
    assert simplified[0] == path[0]
    assert simplified[-1] == path[-1]


def test_simplified_path_remains_collision_free() -> None:
    static_map = make_map()
    path = (
        (-1.5, 0.0, 1.0),
        (-1.0, 1.2, 1.0),
        (0.0, 1.2, 1.0),
        (1.0, 1.2, 1.0),
        (1.5, 0.0, 1.0),
    )
    simplified = simplify_path(path, static_map)
    assert not static_map.path_collides(simplified)


def test_simplification_never_increases_length() -> None:
    path = (
        (-1.5, 0.0, 1.0),
        (-1.0, 1.2, 1.0),
        (0.0, 1.2, 1.0),
        (1.0, 1.2, 1.0),
        (1.5, 0.0, 1.0),
    )
    simplified = simplify_path(path, make_map())
    assert path_length(simplified) <= path_length(path)
    assert len(simplified) < len(path)


def test_direct_path_reduces_to_two_points() -> None:
    static_map = StaticMap(
        (-2.0, -2.0, 0.0),
        (2.0, 2.0, 2.0),
        0.2,
        0.2,
        0.25,
        (),
    )
    path = ((-1.0, 0.0, 1.0), (0.0, 0.2, 1.0), (1.0, 0.0, 1.0))
    assert simplify_path(path, static_map) == (path[0], path[-1])


def test_colliding_input_path_is_rejected() -> None:
    with pytest.raises(ValueError, match="collision-free"):
        simplify_path(
            ((-1.5, 0.0, 1.0), (1.5, 0.0, 1.0)),
            make_map(),
        )
