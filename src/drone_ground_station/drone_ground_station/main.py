"""Ground-station executable entry point."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtWidgets import QApplication

from drone_ground_station.main_window import GroundStationWindow


def main() -> int:
    """Start the Qt event loop with clean signal handling."""
    application = QApplication(sys.argv)
    application.setApplicationName("ROS2 Quadrotor Ground Station")
    application.setQuitOnLastWindowClosed(True)
    _load_optional_font(application)
    window = GroundStationWindow()

    def request_shutdown(_signum: int, _frame: object) -> None:
        QTimer.singleShot(0, window.request_shutdown)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    signal_timer = QTimer()
    signal_timer.setInterval(200)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()
    window.show()
    return int(application.exec_())


def _load_optional_font(application: QApplication) -> None:
    """Load an operator-supplied font when the host lacks CJK fonts."""
    value = os.environ.get("DRONE_GROUND_STATION_FONT_FILE", "").strip()
    if not value:
        return
    font_path = Path(value).expanduser()
    if not font_path.is_file():
        print(
            f"Ground-station font file does not exist: {font_path}",
            file=sys.stderr,
        )
        return
    font_id = QFontDatabase.addApplicationFont(str(font_path))
    families = QFontDatabase.applicationFontFamilies(font_id)
    if not families:
        print(
            f"Ground-station font could not be loaded: {font_path}",
            file=sys.stderr,
        )
        return
    application.setFont(QFont(families[0], 10))


if __name__ == "__main__":
    raise SystemExit(main())
