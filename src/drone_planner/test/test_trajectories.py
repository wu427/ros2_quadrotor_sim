import math

import pytest

from drone_planner.trajectories import circle
from drone_planner.trajectories import custom_path
from drone_planner.trajectories import figure_eight
from drone_planner.trajectories import hover
from drone_planner.trajectories import square


def test_square_and_circle_are_closed():
    assert square((1.0, 2.0, 3.0), 2.0)[0] == square(
        (1.0, 2.0, 3.0), 2.0
    )[-1]
    generated = circle((0.0, 0.0, 1.0), 1.0, 12)
    assert len(generated) == 13
    assert math.dist(generated[0], generated[-1]) < 1.0e-9


def test_figure_eight_is_closed_and_crosses_centre():
    generated = figure_eight((2.0, 3.0, 1.5), 1.0, 16)
    assert math.dist(generated[0], (2.0, 3.0, 1.5)) < 1.0e-9
    assert generated[0] == generated[-1]


def test_hover_and_custom_validation():
    assert hover((1.0, 2.0, 3.0)) == ((1.0, 2.0, 3.0),)
    with pytest.raises(ValueError):
        custom_path(())
