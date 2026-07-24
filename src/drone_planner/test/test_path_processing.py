import math

import pytest

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap
from drone_planner.path_processing import chaikin_smooth
from drone_planner.path_processing import densely_collision_free
from drone_planner.path_processing import process_path
from drone_planner.path_processing import resample_by_arc_length


def _map():
    return StaticMap(
        (-2.0, -2.0, 0.0),
        (5.0, 5.0, 4.0),
        0.2,
        0.1,
        0.2,
        (AABB("box", (1.0, 1.0, 0.0), (2.0, 2.0, 3.0)),),
    )


def test_resampling_preserves_endpoints_and_spacing():
    path = ((0.0, 0.0, 1.0), (1.0, 0.0, 1.0))
    sampled = resample_by_arc_length(path, 0.25)
    assert sampled[0] == path[0]
    assert sampled[-1] == path[-1]
    assert len(sampled) == 5
    assert all(
        math.dist(first, second) <= 0.25 + 1.0e-9
        for first, second in zip(sampled, sampled[1:])
    )


def test_safe_smoothing_and_dense_validation():
    path = (
        (0.0, 0.0, 1.0),
        (0.0, 3.0, 1.0),
        (3.0, 3.0, 1.0),
    )
    result = process_path(path, _map(), 2, 0.05, 0.2, 0.8)
    assert result.smoothing_used
    assert densely_collision_free(result.points, _map(), 0.05, 0.8)


def test_unsafe_smoothing_falls_back_to_original_route():
    path = (
        (0.0, 0.0, 1.0),
        (0.0, 2.6, 1.0),
        (2.6, 2.6, 1.0),
    )
    result = process_path(path, _map(), 3, 0.05, 0.2, 0.8)
    assert densely_collision_free(result.points, _map(), 0.05, 0.8)


def test_minimum_altitude_is_enforced():
    path = ((0.0, 0.0, 0.7), (1.0, 0.0, 0.7))
    assert not densely_collision_free(path, _map(), 0.05, 0.8)
    with pytest.raises(ValueError):
        process_path(path, _map(), minimum_z=0.8)


def test_chaikin_preserves_endpoints():
    path = ((0.0, 0.0, 1.0), (1.0, 1.0, 1.0), (2.0, 0.0, 1.0))
    smoothed = chaikin_smooth(path, 2)
    assert smoothed[0] == path[0]
    assert smoothed[-1] == path[-1]
