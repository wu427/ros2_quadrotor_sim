"""Tests for pure patrol sequence helpers."""

from drone_planner.patrol import PatrolConfig
from drone_planner.patrol import patrol_cycle_indices
from drone_planner.patrol import should_continue_after_cycle


def test_once_cycle() -> None:
    assert patrol_cycle_indices(4, "ONCE") == (0, 1, 2, 3)


def test_ping_pong_cycle() -> None:
    assert patrol_cycle_indices(4, "PING_PONG") == (
        0, 1, 2, 3, 2, 1, 0
    )


def test_loop_can_be_infinite() -> None:
    config = PatrolConfig.from_mapping({"mode": "LOOP", "laps": 0})
    assert should_continue_after_cycle(config, 99, 100.0)


def test_laps_stop_at_requested_count() -> None:
    config = PatrolConfig.from_mapping({"mode": "LAPS", "laps": 2})
    assert should_continue_after_cycle(config, 1, 10.0)
    assert not should_continue_after_cycle(config, 2, 20.0)


def test_timed_patrol_uses_duration() -> None:
    config = PatrolConfig.from_mapping({
        "mode": "TIMED",
        "duration_sec": 30.0,
    })
    assert should_continue_after_cycle(config, 5, 29.0)
    assert not should_continue_after_cycle(config, 5, 30.0)
