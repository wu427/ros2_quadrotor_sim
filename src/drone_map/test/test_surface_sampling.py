import math

import pytest

from drone_map.collision_geometry import AABB
from drone_map.surface_sampling import sample_box_surfaces


def test_sampling_is_deterministic_and_on_surfaces():
    obstacle = AABB("box", (0.0, 0.0, 0.0), (1.0, 2.0, 3.0))
    first = sample_box_surfaces((obstacle,), 0.4)
    second = sample_box_surfaces((obstacle,), 0.4)
    assert first == second
    assert len(first) == len(set(first))
    assert all(
        math.isclose(point[axis], obstacle.minimum[axis])
        or math.isclose(point[axis], obstacle.maximum[axis])
        for point in first
        for axis in range(3)
        if (
            math.isclose(point[axis], obstacle.minimum[axis])
            or math.isclose(point[axis], obstacle.maximum[axis])
        )
    )
    assert all(
        any(
            math.isclose(point[axis], obstacle.minimum[axis])
            or math.isclose(point[axis], obstacle.maximum[axis])
            for axis in range(3)
        )
        for point in first
    )


def test_sampling_rejects_invalid_spacing():
    with pytest.raises(ValueError):
        sample_box_surfaces((), 0.0)
