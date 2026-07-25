"""Thread-safe full-task telemetry recorder and exporter."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import threading
import time
from typing import Dict, Iterable, Mapping, Sequence


class TaskRecorder:
    """Record one complete point or patrol task and freeze at termination."""

    def __init__(self, maximum_samples: int = 10000, sample_period: float = 0.05) -> None:
        if maximum_samples <= 0 or sample_period <= 0.0:
            raise ValueError("recorder limits must be positive")
        self._maximum_samples = maximum_samples
        self._sample_period = sample_period
        self._lock = threading.RLock()
        self._latest: Dict[str, float] = {}
        self._samples: list[Dict[str, float]] = []
        self._events: list[dict[str, object]] = []
        self._meta: dict[str, object] = {}
        self._active = False
        self._frozen = False
        self._activity_seen = False
        self._last_sample = 0.0
        self._final: dict[str, object] = {}

    def start(
        self,
        task_type: str,
        target: Sequence[float] | None = None,
        waypoints: Iterable[Sequence[float]] = (),
        config: Mapping[str, object] | None = None,
    ) -> None:
        """Start a new isolated task recording."""
        with self._lock:
            now = time.time()
            self._samples = []
            self._events = []
            self._meta = {
                "task_type": str(task_type).upper(),
                "target": list(target) if target is not None else None,
                "waypoints": [list(point) for point in waypoints],
                "config": dict(config or {}),
                "start_time": now,
            }
            self._active = True
            self._frozen = False
            self._activity_seen = False
            self._last_sample = 0.0
            self._final = {}
            self._events.append({"time": now, "event": "TASK_STARTED"})

    def update(self, values: Mapping[str, object]) -> None:
        """Merge numeric values and periodically append one complete row."""
        with self._lock:
            for key, value in values.items():
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    self._latest[str(key)] = float(value)
            if not self._active or self._frozen:
                return
            now = time.time()
            if now - self._last_sample < self._sample_period:
                return
            row = {"time": now, **self._latest}
            self._samples.append(row)
            if len(self._samples) > self._maximum_samples:
                del self._samples[: len(self._samples) - self._maximum_samples]
            self._last_sample = now

    def event(self, name: str, details: Mapping[str, object] | None = None) -> None:
        """Append a timestamped event record."""
        with self._lock:
            self._events.append({
                "time": time.time(),
                "event": name,
                "details": dict(details or {}),
            })

    def mark_state(self, state: str) -> None:
        """Remember that a task entered an active state."""
        active_states = {
            "TAKEOFF",
            "PLANNING",
            "EXECUTING",
            "FINAL_APPROACH",
            "RUNNING",
            "DWELL",
            "RETURNING",
        }
        if state in active_states:
            with self._lock:
                self._activity_seen = True

    def freeze(
        self,
        completion_status: str,
        status_source: str,
        raw_module_states: Mapping[str, object],
    ) -> bool:
        """Freeze a task once its authoritative module reaches a terminal state."""
        with self._lock:
            if not self._active or self._frozen or not self._activity_seen:
                return False
            now = time.time()
            self._frozen = True
            self._active = False
            self._final = {
                "completion_status": completion_status,
                "status_source": status_source,
                "raw_module_states": dict(raw_module_states),
                "end_time": now,
            }
            self._events.append({
                "time": now,
                "event": "TASK_FROZEN",
                "details": dict(self._final),
            })
            return True

    def snapshot(self, maximum_points: int = 600) -> dict[str, object]:
        """Return metadata and downsampled history for the dashboard."""
        with self._lock:
            samples = list(self._samples)
            if maximum_points > 0 and len(samples) > maximum_points:
                stride = max(1, len(samples) // maximum_points)
                samples = samples[::stride]
                if samples[-1] != self._samples[-1]:
                    samples.append(dict(self._samples[-1]))
            return {
                "active": self._active,
                "frozen": self._frozen,
                "meta": dict(self._meta),
                "final": dict(self._final),
                "samples": [dict(row) for row in samples],
            }

    def export(
        self,
        directory: Path,
        planner: Mapping[str, object],
        mission: Mapping[str, object],
        patrol: Mapping[str, object],
    ) -> Path:
        """Export CSV, JSON, waypoints and events for the current task."""
        with self._lock:
            directory.mkdir(parents=True, exist_ok=True)
            rows = [dict(row) for row in self._samples]
            fields = sorted({key for row in rows for key in row})
            with (directory / "telemetry.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            waypoints = self._meta.get("waypoints", [])
            with (directory / "waypoints.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.writer(stream)
                writer.writerow(["index", "x", "y", "z", "yaw"])
                for index, point in enumerate(waypoints):
                    writer.writerow([index, *point])
            with (directory / "events.jsonl").open(
                "w", encoding="utf-8"
            ) as stream:
                for event in self._events:
                    stream.write(json.dumps(event, ensure_ascii=False) + "\n")
            summary = self._summary(rows, planner, mission, patrol)
            (directory / "mission_summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            return directory

    def _summary(
        self,
        rows: list[Dict[str, float]],
        planner: Mapping[str, object],
        mission: Mapping[str, object],
        patrol: Mapping[str, object],
    ) -> dict[str, object]:
        task_type = str(self._meta.get("task_type", "POINT"))
        target = self._meta.get("target")
        if target is None:
            waypoints = self._meta.get("waypoints", [])
            target = waypoints[-1] if waypoints else None
        speeds = [
            math.sqrt(row.get("vx", 0.0) ** 2 + row.get("vy", 0.0) ** 2 + row.get("vz", 0.0) ** 2)
            for row in rows
        ]
        errors: list[float] = []
        if isinstance(target, (list, tuple)) and len(target) >= 3:
            for row in rows:
                if all(key in row for key in ("x", "y", "z")):
                    errors.append(math.dist(
                        (row["x"], row["y"], row["z"]),
                        (float(target[0]), float(target[1]), float(target[2])),
                    ))
        rpm_saturation_count = sum(
            1 for row in rows
            if any(row.get(f"rpm{index}", 0.0) >= 9999.0 for index in range(1, 5))
        )
        final = dict(self._final)
        return {
            **self._meta,
            **final,
            "sample_count": len(rows),
            "maximum_speed": max(speeds, default=0.0),
            "maximum_position_error": max(errors, default=None),
            "final_position_error_from_record": errors[-1] if errors else None,
            "minimum_clearance": planner.get("minimum_clearance"),
            "planned_path_length": planner.get("planned_path_length", planner.get("path_length")),
            "actual_path_length": planner.get("actual_path_length"),
            "collision_count": planner.get("collision_count", 0),
            "rpm_saturation_count": rpm_saturation_count,
            "planner": dict(planner),
            "mission": dict(mission),
            "patrol": dict(patrol),
            "task_type": task_type,
        }
