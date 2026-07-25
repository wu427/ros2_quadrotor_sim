"""Tests for full-task recording and authoritative status export."""

import json
from pathlib import Path

from drone_web_ground_station.task_recorder import TaskRecorder


def test_record_freeze_and_export(tmp_path: Path) -> None:
    recorder = TaskRecorder(maximum_samples=20, sample_period=0.0001)
    recorder.start("POINT", target=(2.0, 1.0, 1.5, 0.0))
    recorder.mark_state("EXECUTING")
    recorder.update({
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "vx": 1.0,
        "rpm1": 5000.0,
    })
    assert recorder.freeze(
        "COMPLETED",
        "planner",
        {"planner": {"state": "COMPLETED"}},
    )
    target = recorder.export(
        tmp_path / "run",
        {"state": "COMPLETED", "collision_count": 0},
        {"state": "FAILED", "reason": "NO_PATH_LOADED"},
        {"state": "IDLE"},
    )
    summary = json.loads(
        (target / "mission_summary.json").read_text(encoding="utf-8")
    )
    assert summary["completion_status"] == "COMPLETED"
    assert summary["status_source"] == "planner"
    assert summary["planner"]["state"] == "COMPLETED"
    assert (target / "telemetry.csv").is_file()
