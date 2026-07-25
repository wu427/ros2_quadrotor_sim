import pytest

from drone_ground_station.telemetry import TelemetryBuffer


def test_telemetry_buffer_is_bounded_and_returns_copy():
    buffer = TelemetryBuffer(2)
    buffer.append({"time": 1.0, "x": 1.0})
    buffer.append({"time": 2.0, "x": 2.0})
    buffer.append({"time": 3.0, "x": 3.0})
    rows = buffer.rows()
    assert [row["time"] for row in rows] == [2.0, 3.0]
    rows[0]["x"] = 99.0
    assert buffer.rows()[0]["x"] == 2.0


def test_telemetry_buffer_rejects_non_positive_capacity():
    with pytest.raises(ValueError):
        TelemetryBuffer(0)
