"""QProcess-based launch control scoped to the process it created."""

from __future__ import annotations

import os
import signal
from typing import Dict

from PyQt5.QtCore import QObject
from PyQt5.QtCore import QProcess
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal


class SimulationProcess(QObject):
    """Start and stop one owned ROS2 launch process group."""

    output = pyqtSignal(str)
    state_changed = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self._process = QProcess(self)
        self._process.setProcessChannelMode(QProcess.MergedChannels)
        self._process.readyReadStandardOutput.connect(self._read_output)
        self._process.started.connect(
            lambda: self.state_changed.emit("RUNNING")
        )
        self._process.finished.connect(self._finished)
        self._stop_stage = 0

    @property
    def running(self) -> bool:
        """Return whether the owned launch process is alive."""
        return self._process.state() != QProcess.NotRunning

    def start(
        self,
        map_file: str,
        rviz: bool = False,
        parameters: Dict[str, object] | None = None,
    ) -> None:
        """Start showcase launch in a new process group."""
        if self.running:
            self.output.emit("仿真已在运行。\n")
            return
        overrides = " ".join(
            f"{name}:={_shell_quote(str(value).lower())}"
            for name, value in (parameters or {}).items()
        )
        arguments = [
            "-lc",
            (
                "exec setsid ros2 launch drone_bringup "
                "showcase.launch.py "
                f"map_file:={_shell_quote(map_file)} "
                f"rviz:={'true' if rviz else 'false'} "
                f"gui:=false {overrides}"
            ),
        ]
        self._stop_stage = 0
        self._process.start("/bin/bash", arguments)
        self.state_changed.emit("STARTING")

    def stop(self) -> None:
        """Gracefully stop only the process group created by this object."""
        if not self.running:
            self.state_changed.emit("STOPPED")
            return
        self._stop_stage = 1
        self._signal_group(signal.SIGINT)
        self.state_changed.emit("STOPPING")
        QTimer.singleShot(3000, self._escalate)

    def _escalate(self) -> None:
        if not self.running:
            return
        if self._stop_stage == 1:
            self._stop_stage = 2
            self._signal_group(signal.SIGTERM)
            QTimer.singleShot(2000, self._escalate)
        elif self._stop_stage == 2:
            self._stop_stage = 3
            self._signal_group(signal.SIGKILL)

    def _signal_group(self, requested_signal: signal.Signals) -> None:
        process_id = int(self._process.processId())
        if process_id <= 0:
            return
        try:
            os.killpg(process_id, requested_signal)
        except ProcessLookupError:
            pass
        except PermissionError as error:
            self.output.emit(f"停止进程组失败：{error}\n")

    def _read_output(self) -> None:
        data = bytes(self._process.readAllStandardOutput())
        self.output.emit(data.decode("utf-8", errors="replace"))

    def _finished(self, exit_code: int, _status: object) -> None:
        self.output.emit(f"\n仿真进程退出，code={exit_code}\n")
        self.state_changed.emit("STOPPED")


def _shell_quote(value: str) -> str:
    return "'" + value.replace("'", "'\"'\"'") + "'"
