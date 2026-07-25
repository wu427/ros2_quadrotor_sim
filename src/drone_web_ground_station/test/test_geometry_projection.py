"""Tests for clicked-target geometry helpers."""

import math

from drone_web_ground_station.geometry_projection import canvas_to_map
from drone_web_ground_station.geometry_projection import map_to_canvas
from drone_web_ground_station.geometry_projection import validate_target


def test_canvas_round_trip() -> None:
    bounds = (-1.0, 6.0, -4.0, 4.0)
    pixel = map_to_canvas(2.5, -1.0, 700.0, 800.0, bounds)
    point = canvas_to_map(*pixel, 700.0, 800.0, bounds)
    assert math.isclose(point[0], 2.5)
    assert math.isclose(point[1], -1.0)


def test_target_rejects_inflated_obstacle() -> None:
    valid, reason = validate_target(
        (2.0, 0.0, 1.5),
        (-1.0, 6.0, -4.0, 4.0, 0.0, 3.5),
        [{
            "x": 2.0,
            "y": 0.0,
            "z": 1.5,
            "sx": 1.0,
            "sy": 1.0,
            "sz": 2.0,
        }],
    )
    assert not valid
    assert reason == "GOAL_OCCUPIED"


def test_target_rejects_bounds() -> None:
    valid, reason = validate_target(
        (9.0, 0.0, 1.5),
        (-1.0, 6.0, -4.0, 4.0, 0.0, 3.5),
        [],
    )
    assert not valid
    assert reason == "GOAL_OUT_OF_BOUNDS"
