"""Owned subprocess management without global process killing."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import threading
from typing import Sequence


class OwnedProcess:
    """Manage only one subprocess group created by this object."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._process: subprocess.Popen[str] | None = None

    @property
    def running(self) -> bool:
        """Return whether the owned process is still alive."""
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def start(self, command: Sequence[str], cwd: Path | None = None) -> int:
        """Start one command in a separate process group."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("an owned process is already running")
            self._process = subprocess.Popen(
                list(command),
                cwd=str(cwd) if cwd else None,
                text=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            return int(self._process.pid)

    def stop(self, interrupt_timeout: float = 5.0) -> None:
        """Stop the owned group with SIGINT, then SIGTERM if needed."""
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=interrupt_timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
        finally:
            with self._lock:
                self._process = None
