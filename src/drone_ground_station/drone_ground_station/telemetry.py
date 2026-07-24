"""Thread-safe GUI-side rolling telemetry model."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import threading
from typing import Deque, Dict, List


@dataclass
class TelemetryBuffer:
    """Store bounded numeric histories received through Qt signals."""

    maximum_samples: int = 2000
    _rows: Deque[Dict[str, float]] = field(init=False)
    _lock: threading.Lock = field(
        default_factory=threading.Lock,
        init=False,
    )

    def __post_init__(self) -> None:
        if self.maximum_samples <= 0:
            raise ValueError("maximum_samples must be positive")
        self._rows = deque(maxlen=self.maximum_samples)

    def append(self, row: Dict[str, float]) -> None:
        """Append a copy of a telemetry row."""
        with self._lock:
            self._rows.append(dict(row))

    def rows(self) -> List[Dict[str, float]]:
        """Return a consistent snapshot."""
        with self._lock:
            return [dict(row) for row in self._rows]

    def clear(self) -> None:
        """Remove all samples."""
        with self._lock:
            self._rows.clear()
