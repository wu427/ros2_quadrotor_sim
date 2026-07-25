"""ROS2 state bridge used by the single-page HTTP dashboard."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Dict, Iterable, Mapping, Sequence

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from drone_web_ground_station.api_models import parse_goal
from drone_web_ground_station.geometry_projection import validate_target
from drone_web_ground_station.process_guard import OwnedProcess
from drone_planner.patrol import PatrolConfig
from drone_web_ground_station.task_recorder import TaskRecorder


TERMINAL_PLANNER_STATES = {"COMPLETED", "FAILED", "CANCELLED"}
TERMINAL_PATROL_STATES = {"COMPLETED", "FAILED", "STOPPED", "CANCELLED"}


class WebRosBridge(Node):
    """Collect ROS state, publish commands, and expose thread-safe snapshots."""

    def __init__(
        self,
        history_duration_sec: float = 60.0,
        max_history_points: int = 10000,
    ) -> None:
        super().__init__("web_ground_station")
        self._lock = threading.RLock()
        self._vehicle: Dict[str, float] = {}
        self._planner: Dict[str, object] = {"state": "IDLE"}
        self._mission: Dict[str, object] = {"state": "IDLE"}
        self._patrol: Dict[str, object] = {"state": "IDLE"}
        self._planned_path: list[tuple[float, float, float]] = []
        self._actual_path: list[tuple[float, float, float]] = []
        self._waypoints: list[tuple[float, float, float, float]] = []
        self._raw_obstacles: list[dict[str, object]] = []
        self._inflated_obstacles: list[dict[str, object]] = []
        self._bounds = (-1.0, 6.0, -4.0, 4.0, 0.0, 3.5)
        self._current_waypoint: dict[str, float] = {}
        self._mission_goal: dict[str, float] = {}
        self._last_clearance: float | None = None
        self._duplicate = False
        self._odom_publishers = 0
        self._rpm_publishers = 0
        self._connected_since = time.time()
        history_duration_sec = float(
            self.declare_parameter(
                "history_duration_sec", history_duration_sec
            ).value
        )
        max_history_points = int(
            self.declare_parameter(
                "max_history_points", max_history_points
            ).value
        )
        sample_period = max(
            0.02,
            min(0.2, history_duration_sec / max_history_points),
        )
        self._recorder = TaskRecorder(
            max_history_points, sample_period
        )
        self._owned_simulation = OwnedProcess()
        self._owned_rviz = OwnedProcess()

        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/drone/odom", self._on_odom, 20)
        self.create_subscription(Imu, "/drone/imu", self._on_imu, 20)
        self.create_subscription(
            Float32MultiArray, "/drone/motor_rpm", self._on_rpm, 20
        )
        self.create_subscription(
            PathMessage, "/drone/planned_path", self._on_planned_path, durable
        )
        self.create_subscription(
            PathMessage, "/drone/path", self._on_actual_path, 20
        )
        self.create_subscription(
            String, "/drone/planner_status", self._on_planner_status, durable
        )
        self.create_subscription(
            String, "/drone/mission_status", self._on_mission_status, durable
        )
        self.create_subscription(
            String, "/drone/patrol_status", self._on_patrol_status, durable
        )
        self.create_subscription(
            Float32,
            "/drone/min_obstacle_clearance",
            self._on_clearance,
            durable,
        )
        self.create_subscription(
            MarkerArray, "/map/obstacles", self._on_raw_obstacles, durable
        )
        self.create_subscription(
            MarkerArray,
            "/map/inflated_obstacles",
            self._on_inflated_obstacles,
            durable,
        )
        self.create_subscription(
            Marker, "/drone/current_waypoint", self._on_current_waypoint, durable
        )
        self.create_subscription(
            Marker,
            "/drone/mission_goal_marker",
            self._on_mission_goal_marker,
            durable,
        )

        self._goal_publisher = self.create_publisher(
            PoseStamped, "/drone/mission_goal", 10
        )
        self._command_publisher = self.create_publisher(
            String, "/drone/mission_command", 10
        )
        self._patrol_config_publisher = self.create_publisher(
            String, "/drone/patrol_config", durable
        )
        self._waypoint_publisher = self.create_publisher(
            PathMessage, "/drone/mission_waypoints", durable
        )
        self._rviz_altitude_publisher = self.create_publisher(
            Float32, "/drone/rviz_target_altitude", 10
        )
        self.create_timer(1.0, self._refresh_graph_state)

    def destroy_node(self) -> bool:
        """Stop only subprocesses created by this web bridge."""
        self._owned_rviz.stop()
        self._owned_simulation.stop()
        return super().destroy_node()

    def snapshot(self) -> dict[str, object]:
        """Return one consistent dashboard snapshot."""
        with self._lock:
            speed = math.sqrt(
                self._vehicle.get("vx", 0.0) ** 2
                + self._vehicle.get("vy", 0.0) ** 2
                + self._vehicle.get("vz", 0.0) ** 2
            )
            vehicle = dict(self._vehicle)
            vehicle["speed"] = speed
            recording = self._recorder.snapshot()
            task_actual = [
                [row["x"], row["y"], row["z"]]
                for row in recording.get("samples", [])
                if all(key in row for key in ("x", "y", "z"))
            ]
            return {
                "timestamp": time.time(),
                "connection": {
                    "ros": self._odom_publishers > 0,
                    "simulation_running": self._odom_publishers > 0,
                    "owned_simulation": self._owned_simulation.running,
                    "owned_rviz": self._owned_rviz.running,
                    "odom_publishers": self._odom_publishers,
                    "rpm_publishers": self._rpm_publishers,
                    "duplicate": self._duplicate,
                    "connected_since": self._connected_since,
                },
                "vehicle": vehicle,
                "planner": dict(self._planner),
                "mission": dict(self._mission),
                "patrol": dict(self._patrol),
                "map": {
                    "bounds": list(self._bounds),
                    "obstacles": [dict(item) for item in self._raw_obstacles],
                    "inflated": [dict(item) for item in self._inflated_obstacles],
                },
                "paths": {
                    "planned": [list(point) for point in self._planned_path],
                    "actual": (
                        task_actual
                        if len(task_actual) >= 2
                        else [list(point) for point in self._actual_path]
                    ),
                    "waypoints": [list(point) for point in self._waypoints],
                    "current_waypoint": dict(self._current_waypoint),
                    "mission_goal": dict(self._mission_goal),
                },
                "metrics": {
                    "minimum_clearance": self._last_clearance,
                    "planning_time": self._planner.get(
                        "planning_time_sec", self._planner.get("planning_time")
                    ),
                    "expanded_nodes": self._planner.get("expanded_nodes"),
                    "planned_path_length": self._planner.get(
                        "planned_path_length", self._planner.get("path_length")
                    ),
                    "actual_path_length": self._planner.get("actual_path_length"),
                    "final_position_error": self._planner.get("final_position_error"),
                    "collision_count": self._planner.get("collision_count", 0),
                },
                "recording": recording,
            }

    def validate_goal(self, goal: Sequence[float]) -> tuple[bool, str]:
        """Validate a point against current bounds and inflated markers."""
        x, y, z, _yaw = parse_goal(goal)
        with self._lock:
            return validate_target(
                (x, y, z), self._bounds, self._inflated_obstacles
            )

    def send_goal(self, goal: Sequence[float]) -> None:
        """Publish one safe mission goal and start POINT recording."""
        parsed = parse_goal(goal)
        valid, reason = self.validate_goal(parsed)
        if not valid:
            raise ValueError(reason)
        self._guard_task_publication()
        self._recorder.start("POINT", target=parsed)
        self._recorder.event("GOAL_SENT", {"goal": list(parsed)})
        self._goal_publisher.publish(self._pose(parsed))

    def set_waypoints(self, waypoints: Iterable[Sequence[float]]) -> None:
        """Validate and retain an ordered patrol path."""
        parsed = [parse_goal(point) for point in waypoints]
        if len(parsed) < 2:
            raise ValueError("patrol requires at least two waypoints")
        for point in parsed:
            valid, reason = self.validate_goal(point)
            if not valid:
                raise ValueError(reason)
        with self._lock:
            self._waypoints = parsed

    def start_patrol(self, config: Mapping[str, object]) -> None:
        """Load and start patrol only after the manager acknowledges the path."""
        self._guard_task_publication()
        with self._lock:
            waypoints = list(self._waypoints)
        if len(waypoints) < 2:
            raise ValueError("patrol requires at least two waypoints")
        validated = PatrolConfig.from_mapping(dict(config))
        normalized_config = {
            "mode": validated.mode,
            "laps": validated.laps,
            "duration_sec": validated.duration_sec,
            "dwell_sec": validated.dwell_sec,
            "final_action": validated.final_action,
            "home": list(validated.home),
        }

        config_message = String()
        config_message.data = json.dumps(
            normalized_config, separators=(",", ":")
        )
        self._patrol_config_publisher.publish(config_message)

        path = PathMessage()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self._pose(point) for point in waypoints]
        self._waypoint_publisher.publish(path)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self._lock:
                patrol = dict(self._patrol)
            state = str(patrol.get("state", ""))
            count = int(patrol.get("waypoint_count", 0) or 0)
            mode = str(patrol.get("patrol_mode", ""))
            if state == "FAILED":
                raise RuntimeError(
                    f"PATROL_LOAD_FAILED:{patrol.get('reason', 'UNKNOWN')}"
                )
            if (
                state == "READY"
                and count == len(waypoints)
                and mode == validated.mode
            ):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("PATROL_LOAD_TIMEOUT")

        self._recorder.start(
            "PATROL",
            waypoints=waypoints,
            config=normalized_config,
        )
        command = String()
        command.data = "START"
        self._command_publisher.publish(command)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self._lock:
                patrol = dict(self._patrol)
            state = str(patrol.get("state", ""))
            if state == "FAILED":
                raise RuntimeError(
                    f"PATROL_START_FAILED:{patrol.get('reason', 'UNKNOWN')}"
                )
            if state in {
                "RUNNING",
                "DWELL",
                "PAUSED",
                "RETURNING",
                "COMPLETED",
            }:
                self._recorder.event("PATROL_STARTED", normalized_config)
                return
            time.sleep(0.02)
        raise RuntimeError("PATROL_START_TIMEOUT")

    def set_rviz_altitude(self, value: float) -> None:
        """Publish the altitude used by RViz goal-selection tools."""
        altitude = float(value)
        if not math.isfinite(altitude) or altitude < 0.2 or altitude > 3.5:
            raise ValueError("RViz target altitude must be within [0.2, 3.5]")
        message = Float32()
        message.data = altitude
        self._rviz_altitude_publisher.publish(message)

    def command(self, value: str) -> None:
        """Publish one high-level mission or patrol command."""
        command = value.strip().upper()
        if command not in {
            "PAUSE", "RESUME", "SKIP", "STOP", "CANCEL",
            "RETURN_HOME", "CLEAR",
        }:
            raise ValueError(f"unsupported command: {command}")
        message = String()
        message.data = command
        self._command_publisher.publish(message)
        self._recorder.event("COMMAND", {"command": command})

    def export(self, output_root: Path) -> Path:
        """Export the frozen or active task under a timestamped directory."""
        stamp = time.strftime("%Y%m%d_%H%M%S")
        task = str(self._recorder.snapshot()["meta"].get("task_type", "task")).lower()
        target = output_root / f"{stamp}_{task}"
        return self._recorder.export(
            target, self._planner, self._mission, self._patrol
        )

    def start_simulation(
        self,
        map_file: str | None = None,
        rviz: bool = False,
    ) -> int:
        """Start planned_sim only when no existing simulation is present."""
        self._refresh_graph_state()
        if self._odom_publishers > 0:
            raise RuntimeError("external simulation already connected")
        command = [
            "ros2", "launch", "drone_bringup", "planned_sim.launch.py",
            f"rviz:={'true' if rviz else 'false'}",
        ]
        if map_file:
            command.append(f"map_file:={map_file}")
        return self._owned_simulation.start(command)

    def stop_simulation(self) -> None:
        """Stop only a simulation created by this web server."""
        self._owned_simulation.stop()

    def start_rviz(self) -> int:
        """Open one owned planning RViz process."""
        if self._owned_rviz.running:
            raise RuntimeError("owned RViz is already running")
        return self._owned_rviz.start([
            "ros2", "run", "rviz2", "rviz2", "-d",
            str(self._rviz_config_path()),
        ])

    def stop_rviz(self) -> None:
        """Stop only the RViz process created by this web server."""
        self._owned_rviz.stop()

    def _guard_task_publication(self) -> None:
        self._refresh_graph_state()
        if self._duplicate:
            raise RuntimeError("DUPLICATE_SIMULATION_DETECTED")
        if self._odom_publishers == 0:
            raise RuntimeError("SIMULATION_NOT_CONNECTED")

    def _refresh_graph_state(self) -> None:
        odom = len(self.get_publishers_info_by_topic("/drone/odom"))
        rpm = len(self.get_publishers_info_by_topic("/drone/motor_rpm"))
        with self._lock:
            self._odom_publishers = odom
            self._rpm_publishers = rpm
            self._duplicate = odom > 1 or rpm > 1

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        values = {
            "time": time.time(),
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "vx": twist.linear.x,
            "vy": twist.linear.y,
            "vz": twist.linear.z,
            "wx": twist.angular.x,
            "wy": twist.angular.y,
            "wz": twist.angular.z,
        }
        with self._lock:
            self._vehicle.update(values)
        self._recorder.update(values)

    def _on_imu(self, message: Imu) -> None:
        q = message.orientation
        roll, pitch, yaw = _euler(q.x, q.y, q.z, q.w)
        values = {"roll": roll, "pitch": pitch, "yaw": yaw}
        with self._lock:
            self._vehicle.update(values)
        self._recorder.update(values)

    def _on_rpm(self, message: Float32MultiArray) -> None:
        if len(message.data) != 4:
            return
        values = {
            f"rpm{index + 1}": float(value)
            for index, value in enumerate(message.data)
        }
        with self._lock:
            self._vehicle.update(values)
        self._recorder.update(values)

    def _on_planned_path(self, message: PathMessage) -> None:
        with self._lock:
            self._planned_path = _path_points(message)

    def _on_actual_path(self, message: PathMessage) -> None:
        with self._lock:
            self._actual_path = _path_points(message)

    def _on_planner_status(self, message: String) -> None:
        payload = _json_object(message.data)
        with self._lock:
            self._planner = payload
        self._recorder.mark_state(str(payload.get("state", "")))
        self._recorder.update(_numeric_values(payload, "planner_"))
        task_type = self._recorder.snapshot()["meta"].get("task_type")
        state = str(payload.get("state", ""))
        if task_type == "POINT" and state in TERMINAL_PLANNER_STATES:
            self._recorder.freeze(
                state,
                "planner",
                {"planner": payload, "mission": self._mission, "patrol": self._patrol},
            )

    def _on_mission_status(self, message: String) -> None:
        payload = _json_object(message.data)
        with self._lock:
            self._mission = payload
        self._recorder.mark_state(str(payload.get("state", "")))

    def _on_patrol_status(self, message: String) -> None:
        payload = _json_object(message.data)
        with self._lock:
            self._patrol = payload
        state = str(payload.get("state", ""))
        self._recorder.mark_state(state)
        self._recorder.update(_numeric_values(payload, "patrol_"))
        task_type = self._recorder.snapshot()["meta"].get("task_type")
        if task_type == "PATROL" and state in TERMINAL_PATROL_STATES:
            self._recorder.freeze(
                state,
                "patrol_manager",
                {"planner": self._planner, "mission": self._mission, "patrol": payload},
            )

    def _on_clearance(self, message: Float32) -> None:
        with self._lock:
            self._last_clearance = float(message.data)
        self._recorder.update({"minimum_clearance": float(message.data)})

    def _on_raw_obstacles(self, message: MarkerArray) -> None:
        boxes, bounds = _marker_geometry(message)
        with self._lock:
            self._raw_obstacles = boxes
            if bounds is not None:
                self._bounds = bounds

    def _on_inflated_obstacles(self, message: MarkerArray) -> None:
        boxes, _bounds = _marker_geometry(message)
        with self._lock:
            self._inflated_obstacles = boxes

    def _on_current_waypoint(self, message: Marker) -> None:
        with self._lock:
            self._current_waypoint = {
                "x": message.pose.position.x,
                "y": message.pose.position.y,
                "z": message.pose.position.z,
            }

    def _on_mission_goal_marker(self, message: Marker) -> None:
        with self._lock:
            self._mission_goal = {
                "x": message.pose.position.x,
                "y": message.pose.position.y,
                "z": message.pose.position.z,
            }

    def _pose(self, values: Sequence[float]) -> PoseStamped:
        x, y, z, yaw = parse_goal(values)
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = z
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        return message

    @staticmethod
    def _rviz_config_path() -> Path:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("drone_bringup"))
            / "rviz"
            / "planned_quadrotor.rviz"
        )


def _json_object(value: str) -> Dict[str, object]:
    try:
        payload = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return {"state": "INVALID", "raw": value}
    return payload if isinstance(payload, dict) else {"value": payload}


def _numeric_values(
    payload: Mapping[str, object], prefix: str
) -> dict[str, float]:
    return {
        prefix + str(key): float(value)
        for key, value in payload.items()
        if isinstance(value, (int, float)) and math.isfinite(float(value))
    }


def _path_points(message: PathMessage) -> list[tuple[float, float, float]]:
    return [
        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        for pose in message.poses
    ]


def _marker_geometry(
    message: MarkerArray,
) -> tuple[list[dict[str, object]], tuple[float, float, float, float, float, float] | None]:
    boxes: list[dict[str, object]] = []
    points: list[tuple[float, float, float]] = []
    for marker in message.markers:
        if marker.type == Marker.CUBE:
            boxes.append({
                "id": marker.text or f"{marker.ns}_{marker.id}",
                "x": marker.pose.position.x,
                "y": marker.pose.position.y,
                "z": marker.pose.position.z,
                "sx": marker.scale.x,
                "sy": marker.scale.y,
                "sz": marker.scale.z,
                "namespace": marker.ns,
            })
        elif marker.type == Marker.LINE_LIST:
            points.extend((point.x, point.y, point.z) for point in marker.points)
    if not points:
        return boxes, None
    xs, ys, zs = zip(*points)
    return boxes, (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


def _euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        return 0.0, 0.0, 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw
