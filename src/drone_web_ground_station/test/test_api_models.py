"""Tests for web API input validation."""

import pytest

from drone_web_ground_station.api_models import parse_goal
from drone_web_ground_station.api_models import parse_waypoints


def test_goal_object() -> None:
    assert parse_goal({"x": 1, "y": 2, "z": 1.5}) == (
        1.0, 2.0, 1.5, 0.0
    )


def test_waypoint_list() -> None:
    assert len(parse_waypoints([[0, 0, 1.5], [2, 1, 1.5]])) == 2


def test_empty_waypoints_rejected() -> None:
    with pytest.raises(ValueError):
        parse_waypoints([])
