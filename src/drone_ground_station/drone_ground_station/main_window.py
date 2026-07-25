"""Main PyQt5 ground-station window."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time
from typing import Dict, List

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtWidgets import QFileDialog
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QListWidget
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QPlainTextEdit
from PyQt5.QtWidgets import QProgressBar
from PyQt5.QtWidgets import QStackedWidget
from PyQt5.QtWidgets import QSpinBox
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget

from drone_ground_station.process_control import SimulationProcess
from drone_ground_station.profile_store import load_profile
from drone_ground_station.profile_store import save_profile
from drone_ground_station.profiles import load_profiles
from drone_ground_station.ros_worker import RosWorker
from drone_ground_station.telemetry import TelemetryBuffer
from drone_planner.trajectories import circle
from drone_planner.trajectories import figure_eight
from drone_planner.trajectories import square


class PlotCanvas(FigureCanvasQTAgg):
    """Small reusable Matplotlib canvas."""

    def __init__(self) -> None:
        self.figure = Figure(figsize=(7, 5), tight_layout=True, facecolor="#0b1220")
        self.axes = self.figure.add_subplot(111)
        self.axes.set_facecolor("#0b1220")
        super().__init__(self.figure)


class GroundStationWindow(QMainWindow):
    """Single-vehicle showcase, monitoring, mission, and test console."""

    def __init__(self) -> None:
        super().__init__()
        self._closing = False
        self.setWindowTitle("ROS2 Quadrotor Ground Station")
        self.resize(1380, 860)
        self.setMinimumSize(1120, 720)
        self._profiles = load_profiles()
        self._telemetry = TelemetryBuffer()
        self._latest: Dict[str, float] = {}
        self._planned_path: List[tuple[float, float, float]] = []
        self._actual_path: List[tuple[float, float, float]] = []
        self._obstacles: List[Dict[str, object]] = []
        self._planner_status: Dict[str, object] = {}
        self._mission_status: Dict[str, object] = {}
        self._custom_map_path = ""

        self._ros = RosWorker()
        self._process = SimulationProcess()
        self._connect_signals()
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(200)
        self._ros.start()

    def _connect_signals(self) -> None:
        self._ros.telemetry.connect(self._on_telemetry)
        self._ros.status.connect(self._on_planner_status)
        self._ros.mission_status.connect(self._on_mission_status)
        self._ros.planned_path.connect(self._on_planned_path)
        self._ros.actual_path.connect(self._on_actual_path)
        self._ros.obstacles.connect(self._on_obstacles)
        self._ros.connection.connect(self._on_connection)
        self._ros.log.connect(self._append_log)
        self._process.output.connect(self._append_log)
        self._process.state_changed.connect(self._on_process_state)

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QHBoxLayout(root)
        self._navigation = QListWidget()
        names = [
            "系统总览",
            "地图与轨迹",
            "飞行曲线",
            "电机 RPM",
            "规划详情",
            "任务控制",
            "测试中心",
            "日志",
        ]
        self._navigation.addItems(names)
        self._navigation.setFixedWidth(170)
        self._pages = QStackedWidget()
        self._pages.addWidget(self._overview_page())
        self._map_canvas = PlotCanvas()
        self._pages.addWidget(self._canvas_page(self._map_canvas))
        self._flight_canvas = PlotCanvas()
        self._pages.addWidget(self._canvas_page(self._flight_canvas))
        self._rpm_canvas = PlotCanvas()
        self._pages.addWidget(self._canvas_page(self._rpm_canvas))
        self._pages.addWidget(self._planning_page())
        self._pages.addWidget(self._mission_page())
        self._pages.addWidget(self._test_page())
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._pages.addWidget(self._log_view)
        self._navigation.currentRowChanged.connect(
            self._pages.setCurrentIndex
        )
        self._navigation.setCurrentRow(0)
        layout.addWidget(self._navigation)
        layout.addWidget(self._pages, 1)
        self.setCentralWidget(root)
        self._apply_style()

    def _overview_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(12)

        header = QLabel("QUADROTOR MISSION CONTROL")
        header.setObjectName("pageTitle")
        subtitle = QLabel("地图配置、系统连接、飞行状态与任务操作")
        subtitle.setObjectName("pageSubtitle")
        layout.addWidget(header)
        layout.addWidget(subtitle)

        form = QFormLayout()
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(9)
        self._profile = QComboBox()
        for name, profile in self._profiles.items():
            self._profile.addItem(str(profile["label"]), name)
        self._profile.addItem("自定义 YAML…", "__custom__")
        self._rviz_enabled = QCheckBox("启动 RViz")
        self._point_cloud_enabled = QCheckBox("发布 PointCloud2")
        self._vehicle_count = QSpinBox()
        self._vehicle_count.setRange(1, 1)
        self._vehicle_count.setValue(1)
        self._vehicle_note = QLabel(
            "当前版本仅支持 1 架无人机；multi-UAV not implemented."
        )
        self._connection_label = QLabel("ROS: CONNECTING")
        self._process_label = QLabel("仿真: STOPPED")
        self._state_label = QLabel("规划: IDLE")
        self._position_label = QLabel("位置: --")
        for status_label in (
            self._connection_label,
            self._process_label,
            self._state_label,
            self._position_label,
        ):
            status_label.setObjectName("statusCard")
        form.addRow("展示场景", self._profile)
        form.addRow("三维显示", self._rviz_enabled)
        form.addRow("点云显示", self._point_cloud_enabled)
        form.addRow("Vehicle Count", self._vehicle_count)
        form.addRow("", self._vehicle_note)
        form.addRow("连接", self._connection_label)
        form.addRow("进程", self._process_label)
        form.addRow("状态", self._state_label)
        form.addRow("实时位置", self._position_label)
        buttons = QHBoxLayout()
        start = QPushButton("启动仿真")
        start.setProperty("role", "primary")
        start.clicked.connect(self._start_simulation)
        attach = QPushButton("连接现有 ROS 图")
        attach.setProperty("role", "secondary")
        attach.clicked.connect(
            lambda: self._append_log("ROS 后台线程已连接现有图。\n")
        )
        stop = QPushButton("停止本窗口启动的仿真")
        stop.setProperty("role", "danger")
        stop.clicked.connect(self._process.stop)
        restart = QPushButton("重启仿真")
        restart.setProperty("role", "secondary")
        restart.clicked.connect(self._restart_simulation)
        choose_map = QPushButton("选择自定义 YAML")
        choose_map.setProperty("role", "secondary")
        choose_map.clicked.connect(self._choose_custom_map)
        hold = QPushButton("紧急悬停")
        hold.setProperty("role", "warning")
        hold.clicked.connect(
            lambda: self._ros.enqueue("mission_command", "PAUSE")
        )
        export = QPushButton("导出数据")
        export.setProperty("role", "secondary")
        export.clicked.connect(self._export_data)
        shot = QPushButton("保存截图")
        shot.setProperty("role", "secondary")
        shot.clicked.connect(self._save_screenshot)
        for button in (
            start,
            attach,
            stop,
            restart,
            choose_map,
            hold,
            export,
            shot,
        ):
            buttons.addWidget(button)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addStretch(1)
        return page

    def _planning_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        form = QFormLayout()
        defaults = {
            "drone_radius": "0.25",
            "safety_margin": "0.45",
            "grid_resolution": "0.25",
            "minimum_flight_z": "1.00",
            "cruise_altitude": "1.50",
            "waypoint_pass_radius": "0.22",
            "lookahead_distance": "0.55",
            "path_sample_spacing": "0.25",
            "smoothing_iterations": "2",
            "collision_check_step": "0.05",
            "planning_timeout_sec": "2.0",
            "max_expanded_nodes": "100000",
            "reference_update_rate": "20.0",
        }
        self._parameter_defaults = defaults
        self._parameter_edits = {}
        for name, value in defaults.items():
            edit = QLineEdit(value)
            self._parameter_edits[name] = edit
            form.addRow(name, edit)
        self._smoothing_enabled = QCheckBox("启用安全平滑")
        self._smoothing_enabled.setChecked(True)
        form.addRow("smoothing_enabled", self._smoothing_enabled)
        actions = QHBoxLayout()
        restore = QPushButton("恢复默认值")
        restore.setProperty("role", "secondary")
        restore.clicked.connect(self._restore_defaults)
        save = QPushButton("保存 JSON Profile")
        save.setProperty("role", "secondary")
        save.clicked.connect(self._save_profile)
        load = QPushButton("加载 JSON Profile")
        load.setProperty("role", "secondary")
        load.clicked.connect(self._load_profile)
        actions.addWidget(restore)
        actions.addWidget(save)
        actions.addWidget(load)
        self._planning_text = QPlainTextEdit()
        self._planning_text.setReadOnly(True)
        layout.addLayout(form)
        layout.addLayout(actions)
        layout.addWidget(self._planning_text)
        return page

    def _mission_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        form = QFormLayout()
        self._trajectory = QComboBox()
        self._trajectory.addItems([
            "单点",
            "悬停",
            "正方形",
            "圆",
            "8 字",
            "自定义",
        ])
        self._target = QLineEdit("5.0,0.0,1.5,0.0")
        self._size = QLineEdit("1.0")
        form.addRow("轨迹类型", self._trajectory)
        form.addRow("目标/中心 x,y,z,yaw", self._target)
        form.addRow("边长/半径", self._size)
        self._mission_progress = QProgressBar()
        buttons = QHBoxLayout()
        send = QPushButton("下发任务")
        send.setProperty("role", "primary")
        send.clicked.connect(self._send_mission)
        home = QPushButton("返航")
        home.setProperty("role", "secondary")
        home.clicked.connect(
            lambda: self._ros.enqueue("goal", (0.0, 0.0, 1.5, 0.0))
        )
        current_hold = QPushButton("保持当前位置")
        current_hold.setProperty("role", "warning")
        current_hold.clicked.connect(self._hold_current)
        for label, command in (
            ("开始", "START"),
            ("暂停", "PAUSE"),
            ("继续", "RESUME"),
            ("取消", "CANCEL"),
            ("清空", "CLEAR"),
        ):
            button = QPushButton(label)
            button.clicked.connect(
                lambda _checked, value=command: self._ros.enqueue(
                    "mission_command", value
                )
            )
            buttons.addWidget(button)
        layout.addLayout(form)
        layout.addWidget(self._mission_progress)
        layout.addWidget(send)
        layout.addWidget(home)
        layout.addWidget(current_hold)
        layout.addLayout(buttons)
        layout.addStretch(1)
        return page

    def _test_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        text = QPlainTextEdit()
        text.setReadOnly(True)
        text.setPlainText(
            "一键验收命令：\n"
            "scripts/run_acceptance.sh\n"
            "scripts/run_planning_acceptance.sh scenario:=multi\n"
            "scripts/run_showcase_acceptance.sh scenario:=default\n\n"
            "测试进程从终端运行并保存完整日志；GUI 不会终止外部进程。"
        )
        layout.addWidget(text)
        return page

    @staticmethod
    def _canvas_page(canvas: PlotCanvas) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.addWidget(canvas)
        return page

    def _start_simulation(self) -> None:
        publisher_count = self._ros.simulation_publishers()
        if publisher_count > 0:
            QMessageBox.warning(
                self,
                "仿真已运行",
                "检测到外部 /drone/odom 发布者，已阻止重复启动。"
                f"\nPublisher count: {publisher_count}",
            )
            return
        name = str(self._profile.currentData())
        if name == "__custom__":
            if not self._custom_map_path:
                self._choose_custom_map()
            map_path = self._custom_map_path
        else:
            map_path = str(self._profiles[name]["map_path"])
        if not map_path:
            return
        try:
            parameters = self._validated_launch_parameters()
        except ValueError as error:
            QMessageBox.warning(self, "规划参数错误", str(error))
            return
        parameters["publish_point_cloud"] = (
            self._point_cloud_enabled.isChecked()
        )
        self._process.start(
            map_path,
            self._rviz_enabled.isChecked(),
            parameters,
        )

    def _restart_simulation(self) -> None:
        self._process.stop()
        QTimer.singleShot(3500, self._start_simulation)

    def _choose_custom_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "选择静态地图", "", "YAML (*.yaml *.yml)"
        )
        if path:
            self._custom_map_path = path
            self._profile.setCurrentIndex(self._profile.count() - 1)

    def _validated_launch_parameters(self) -> Dict[str, object]:
        numeric = {
            name: float(edit.text())
            for name, edit in self._parameter_edits.items()
        }
        if any(value <= 0.0 for value in numeric.values()):
            raise ValueError("所有规划参数必须大于 0")
        if numeric["cruise_altitude"] < numeric["minimum_flight_z"]:
            raise ValueError("cruise_altitude 不得低于 minimum_flight_z")
        launch_names = {
            "minimum_flight_z",
            "cruise_altitude",
            "waypoint_pass_radius",
            "lookahead_distance",
            "path_sample_spacing",
            "smoothing_iterations",
            "collision_check_step",
            "planning_timeout_sec",
            "max_expanded_nodes",
            "reference_update_rate",
        }
        result: Dict[str, object] = {
            name: (
                int(value)
                if name in {"smoothing_iterations", "max_expanded_nodes"}
                else value
            )
            for name, value in numeric.items()
            if name in launch_names
        }
        result["smoothing_enabled"] = self._smoothing_enabled.isChecked()
        return result

    def _restore_defaults(self) -> None:
        for name, value in self._parameter_defaults.items():
            self._parameter_edits[name].setText(value)
        self._smoothing_enabled.setChecked(True)

    def _save_profile(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "保存 Profile", "showcase_profile.json", "JSON (*.json)"
        )
        if not path:
            return
        payload = {
            "map_profile": str(self._profile.currentData()),
            "custom_map_path": self._custom_map_path,
            "parameters": {
                name: edit.text()
                for name, edit in self._parameter_edits.items()
            },
            "smoothing_enabled": self._smoothing_enabled.isChecked(),
        }
        save_profile(path, payload)

    def _load_profile(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "加载 Profile", "", "JSON (*.json)"
        )
        if not path:
            return
        try:
            payload = load_profile(path)
            parameters = payload.get("parameters", {})
            if not isinstance(parameters, dict):
                raise ValueError("profile.parameters 必须是对象")
            for name, value in parameters.items():
                if name in self._parameter_edits:
                    self._parameter_edits[name].setText(str(value))
            self._custom_map_path = str(
                payload.get("custom_map_path", "")
            )
            self._smoothing_enabled.setChecked(
                bool(payload.get("smoothing_enabled", True))
            )
        except ValueError as error:
            QMessageBox.warning(self, "Profile 错误", str(error))

    def _hold_current(self) -> None:
        if all(key in self._latest for key in ("x", "y", "z")):
            self._ros.enqueue(
                "goal",
                (
                    self._latest["x"],
                    self._latest["y"],
                    self._latest["z"],
                    self._latest.get("yaw", 0.0),
                ),
            )

    def _send_mission(self) -> None:
        try:
            kind = self._trajectory.currentText()
            if kind == "自定义":
                points = tuple(
                    tuple(
                        float(component.strip())
                        for component in row.split(",")
                    )
                    for row in self._target.text().split(";")
                )
                if any(len(point) != 3 for point in points):
                    raise ValueError(
                        "自定义格式为 x,y,z;x,y,z;..."
                    )
                self._ros.enqueue("waypoints", points)
                return
            values = [
                float(value.strip())
                for value in self._target.text().split(",")
            ]
            if len(values) != 4:
                raise ValueError("目标必须包含 x,y,z,yaw")
            if kind in {"单点", "悬停"}:
                self._ros.enqueue("goal", values)
                return
            centre = values[:3]
            size = float(self._size.text())
            if kind == "正方形":
                points = square(centre, size)
            elif kind == "圆":
                points = circle(centre, size)
            elif kind == "8 字":
                points = figure_eight(centre, size)
            self._ros.enqueue(
                "waypoints",
                [tuple(point) + (values[3],) for point in points],
            )
        except (TypeError, ValueError) as error:
            QMessageBox.warning(self, "任务参数错误", str(error))

    def _on_telemetry(self, values: object) -> None:
        data = dict(values)
        self._latest.update(data)
        row = {"time": float(data.get("time", time.time()))}
        row.update({
            key: float(value)
            for key, value in self._latest.items()
            if isinstance(value, (int, float))
        })
        self._telemetry.append(row)

    def _on_planner_status(self, values: object) -> None:
        self._planner_status = dict(values)

    def _on_mission_status(self, values: object) -> None:
        self._mission_status = dict(values)

    def _on_planned_path(self, values: object) -> None:
        self._planned_path = list(values)

    def _on_actual_path(self, values: object) -> None:
        self._actual_path = list(values)

    def _on_obstacles(self, values: object) -> None:
        self._obstacles = list(values)

    def _on_connection(self, state: str) -> None:
        self._connection_label.setText(f"ROS: {state}")

    def _on_process_state(self, state: str) -> None:
        self._process_label.setText(f"仿真: {state}")

    def _append_log(self, text: str) -> None:
        if hasattr(self, "_log_view"):
            self._log_view.moveCursor(
                QTextCursor.End
            )
            self._log_view.insertPlainText(text)

    def _refresh(self) -> None:
        if self._closing:
            return
        self._state_label.setText(
            f"规划: {self._planner_status.get('state', 'IDLE')}"
        )
        if all(key in self._latest for key in ("x", "y", "z")):
            self._position_label.setText(
                "位置: "
                f"{self._latest['x']:.2f}, "
                f"{self._latest['y']:.2f}, "
                f"{self._latest['z']:.2f} m"
            )
        progress = float(self._mission_status.get("progress", 0.0))
        self._mission_progress.setValue(int(100.0 * progress))
        self._planning_text.setPlainText(
            json.dumps(
                {
                    "planner": self._planner_status,
                    "mission": self._mission_status,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        self._draw_map()
        self._draw_histories()

    def _draw_map(self) -> None:
        if self._closing:
            return
        axes = self._map_canvas.axes
        axes.clear()
        self._style_plot_axes(axes)
        for box in self._obstacles:
            axes.add_patch(Rectangle(
                (
                    float(box["x"]) - 0.5 * float(box["sx"]),
                    float(box["y"]) - 0.5 * float(box["sy"]),
                ),
                float(box["sx"]),
                float(box["sy"]),
                color="#c74332",
                alpha=0.65,
            ))
        for path, colour, label in (
            (self._planned_path, "#1f77b4", "planned"),
            (self._actual_path, "#2ca02c", "actual"),
        ):
            if path:
                axes.plot(
                    [point[0] for point in path],
                    [point[1] for point in path],
                    color=colour,
                    label=label,
                )
        if "x" in self._latest:
            axes.scatter(
                [self._latest["x"]],
                [self._latest["y"]],
                color="#ffcc00",
                marker="^",
                s=80,
            )
        axes.set_aspect("equal", adjustable="box")
        axes.grid(True, alpha=0.3)
        axes.set_xlabel("x (m)")
        axes.set_ylabel("y (m)")
        axes.set_title("Top-down Map and Trajectory")
        if self._planned_path or self._actual_path:
            axes.legend()
        self._style_plot_axes(axes)
        self._map_canvas.draw_idle()

    def _draw_histories(self) -> None:
        if self._closing:
            return
        rows = self._telemetry.rows()[-300:]
        if not rows:
            return
        start = rows[0]["time"]
        times = [row["time"] - start for row in rows]
        flight = self._flight_canvas.axes
        flight.clear()
        for key in ("x", "y", "z"):
            flight.plot(
                times,
                [row.get(key, float("nan")) for row in rows],
                label=key,
            )
        flight.set_xlabel("window time (s)")
        flight.set_ylabel("position (m)")
        flight.set_title("Position History")
        flight.grid(True, alpha=0.3)
        flight.legend()
        self._style_plot_axes(flight)
        self._flight_canvas.draw_idle()
        rpm = self._rpm_canvas.axes
        rpm.clear()
        for index in range(1, 5):
            key = f"rpm{index}"
            rpm.plot(
                times,
                [row.get(key, float("nan")) for row in rows],
                label=f"M{index}",
            )
        rpm.set_xlabel("window time (s)")
        rpm.set_ylabel("RPM")
        rpm.set_title("Motor Speed")
        rpm.grid(True, alpha=0.3)
        rpm.legend()
        self._style_plot_axes(rpm)
        self._rpm_canvas.draw_idle()

    def _export_data(self) -> None:
        directory = QFileDialog.getExistingDirectory(
            self, "选择导出目录"
        )
        if not directory:
            return
        self.export_to_directory(Path(directory))

    def export_to_directory(self, target: Path) -> None:
        """Export current telemetry and summary without UI interaction."""
        target.mkdir(parents=True, exist_ok=True)
        rows = self._telemetry.rows()
        fields = sorted({key for row in rows for key in row})
        with (target / "telemetry.csv").open(
            "w", encoding="utf-8", newline=""
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        speeds = [
            (
                row.get("vx", 0.0) ** 2
                + row.get("vy", 0.0) ** 2
                + row.get("vz", 0.0) ** 2
            ) ** 0.5
            for row in rows
        ]
        target_values: tuple[float, ...] = ()
        try:
            target_values = tuple(
                float(value.strip())
                for value in self._target.text().split(",")
            )
            target_position = target_values[:3]
            if len(target_position) != 3:
                raise ValueError
        except ValueError:
            target_position = ()
        position_errors = [
            math.dist(
                (
                    row.get("x", float("nan")),
                    row.get("y", float("nan")),
                    row.get("z", float("nan")),
                ),
                target_position,
            )
            for row in rows
            if target_position
            and all(name in row for name in ("x", "y", "z"))
        ]
        position_errors = [
            value for value in position_errors if math.isfinite(value)
        ]
        rpm_saturation_count = sum(
            1
            for row in rows
            if any(
                row.get(f"rpm{index}", 0.0) >= 10000.0 - 1.0e-6
                for index in range(1, 5)
            )
        )
        planned_length = self._planner_status.get(
            "planned_path_length",
            _polyline_length(self._planned_path),
        )
        actual_length = self._planner_status.get(
            "actual_path_length",
            _polyline_length(self._actual_path),
        )
        (target / "mission_summary.json").write_text(
            json.dumps({
                "map": str(self._profile.currentData()),
                "trajectory_type": self._trajectory.currentText(),
                "target": list(target_values) if target_values else None,
                "parameters": {
                    name: edit.text()
                    for name, edit in self._parameter_edits.items()
                },
                "planner": self._planner_status,
                "mission": self._mission_status,
                "start_time": rows[0]["time"] if rows else None,
                "end_time": rows[-1]["time"] if rows else None,
                "maximum_speed": max(speeds, default=0.0),
                "maximum_position_error": max(
                    position_errors, default=None
                ),
                "final_position_error": (
                    position_errors[-1] if position_errors else None
                ),
                "minimum_clearance": self._planner_status.get(
                    "minimum_clearance"
                ),
                "planned_path_length": planned_length,
                "actual_path_length": actual_length,
                "rpm_saturation_limit": 10000.0,
                "rpm_saturation_count": rpm_saturation_count,
                "completion_status": self._planner_status.get("state"),
                "collision_count": self._planner_status.get(
                    "collision_count", 0
                ),
            }, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self._map_canvas.figure.savefig(
            target / "trajectory.png", dpi=160
        )
        self._append_log(f"数据已导出到 {target}\n")

    def _save_screenshot(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存截图",
            "ground_station.png",
            "PNG (*.png)",
        )
        if path:
            self.grab().save(path, "PNG")

    def request_shutdown(self) -> None:
        """Request a normal Qt shutdown."""
        if not self._closing:
            self.close()

    def closeEvent(self, event: object) -> None:
        """Stop owned resources without touching an external ROS graph."""
        if self._closing:
            event.accept()
            return
        self._closing = True
        if hasattr(self, "_timer") and self._timer.isActive():
            self._timer.stop()
        if self._process.running:
            self._process.stop()
        self._ros.stop()
        event.accept()

    @staticmethod
    def _style_plot_axes(axes: object) -> None:
        """Apply a dark style to a Matplotlib axes."""
        axes.set_facecolor("#0b1220")
        axes.tick_params(colors="#94a3b8")
        axes.xaxis.label.set_color("#cbd5e1")
        axes.yaxis.label.set_color("#cbd5e1")
        axes.title.set_color("#f8fafc")
        axes.grid(True, color="#334155", alpha=0.45)
        for spine in axes.spines.values():
            spine.set_color("#334155")

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #0f172a;
                color: #e2e8f0;
                font-size: 13px;
            }
            QListWidget {
                background: #111827;
                border: 1px solid #334155;
                border-radius: 10px;
                padding: 7px;
                outline: none;
            }
            QListWidget::item {
                min-height: 36px;
                padding: 4px 10px;
                margin: 2px;
                border-radius: 6px;
                color: #94a3b8;
            }
            QListWidget::item:selected {
                background: #1d4ed8;
                color: white;
                font-weight: 600;
            }
            QPlainTextEdit, QLineEdit, QComboBox, QSpinBox {
                background: #111827;
                color: #e5e7eb;
                border: 1px solid #334155;
                border-radius: 7px;
                padding: 7px;
                selection-background-color: #2563eb;
            }
            QLineEdit:focus, QComboBox:focus, QPlainTextEdit:focus {
                border-color: #3b82f6;
            }
            QPushButton {
                background: #2563eb;
                border: 1px solid #3b82f6;
                border-radius: 7px;
                padding: 8px 13px;
                color: white;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #3b82f6;
            }
            QPushButton:pressed {
                background: #1d4ed8;
            }
            QProgressBar {
                background: #111827;
                border: 1px solid #334155;
                border-radius: 7px;
                text-align: center;
                color: #e2e8f0;
            }
            QProgressBar::chunk {
                background: #22c55e;
                border-radius: 6px;
            }
            QToolTip {
                background: #111827;
                color: #f8fafc;
                border: 1px solid #475569;
            }
            QLabel#pageTitle {
                color: #f8fafc;
                font-size: 22px;
                font-weight: 700;
            }
            QLabel#pageSubtitle {
                color: #94a3b8;
                padding-bottom: 5px;
            }
            QLabel#statusCard {
                background: #0b1220;
                border: 1px solid #334155;
                border-radius: 7px;
                padding: 9px 12px;
                color: #f8fafc;
                font-weight: 600;
            }
            QPushButton[role="secondary"] {
                background: #1e293b;
                border-color: #475569;
            }
            QPushButton[role="secondary"]:hover {
                background: #334155;
            }
            QPushButton[role="warning"] {
                background: #b45309;
                border-color: #d97706;
            }
            QPushButton[role="danger"] {
                background: #b91c1c;
                border-color: #dc2626;
            }
            QPushButton[role="success"] {
                background: #15803d;
                border-color: #22c55e;
            }
            """
        )


def _polyline_length(
    points: List[tuple[float, float, float]],
) -> float:
    """Return the Euclidean arc length of a three-dimensional polyline."""
    return sum(
        math.dist(first, second)
        for first, second in zip(points, points[1:])
    )
