"""Background ROS2 executor and Qt signal bridge."""

from __future__ import annotations

import json
import math
from queue import Empty
from queue import Queue
import time
from typing import Dict, Sequence

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from PyQt5.QtCore import QThread
from PyQt5.QtCore import pyqtSignal


class RosWorker(QThread):
    """Own all rclpy objects in a background thread."""

    telemetry = pyqtSignal(object)
    status = pyqtSignal(object)
    mission_status = pyqtSignal(object)
    planned_path = pyqtSignal(object)
    actual_path = pyqtSignal(object)
    obstacles = pyqtSignal(object)
    connection = pyqtSignal(str)
    log = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self._commands: Queue[tuple[str, object]] = Queue()

    def enqueue(self, command: str, payload: object = None) -> None:
        """Queue a ROS publication for the worker thread."""
        self._commands.put((command, payload))

    def stop(self) -> None:
        """Request executor shutdown and wait for the thread."""
        self.requestInterruption()
        self.wait(5000)

    def run(self) -> None:
        """Create, spin, and destroy all ROS entities."""
        node = None
        executor = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = Node("ground_station_bridge")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            context = self._create_entities(node)
            self.connection.emit("CONNECTED")
            while not self.isInterruptionRequested() and rclpy.ok():
                executor.spin_once(timeout_sec=0.05)
                self._drain_commands(node, context)
        except Exception as error:  # GUI boundary must report cleanly.
            self.log.emit(f"ROS 线程异常：{error}\n")
            self.connection.emit("ERROR")
        finally:
            if executor is not None and node is not None:
                executor.remove_node(node)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.connection.emit("DISCONNECTED")

    def _create_entities(self, node: Node) -> Dict[str, object]:
        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(
            Odometry, "/drone/odom", self._odom_callback, 10
        )
        node.create_subscription(
            Imu, "/drone/imu", self._imu_callback, 10
        )
        node.create_subscription(
            Float32MultiArray,
            "/drone/motor_rpm",
            self._rpm_callback,
            10,
        )
        node.create_subscription(
            Path,
            "/drone/planned_path",
            lambda message: self.planned_path.emit(_path_points(message)),
            durable,
        )
        node.create_subscription(
            Path,
            "/drone/path",
            lambda message: self.actual_path.emit(_path_points(message)),
            10,
        )
        node.create_subscription(
            String,
            "/drone/planner_status",
            lambda message: self.status.emit(_json(message.data)),
            durable,
        )
        node.create_subscription(
            String,
            "/drone/mission_status",
            lambda message: self.mission_status.emit(_json(message.data)),
            durable,
        )
        node.create_subscription(
            MarkerArray,
            "/map/obstacles",
            self._obstacle_callback,
            durable,
        )
        return {
            "goal": node.create_publisher(
                PoseStamped, "/drone/mission_goal", 10
            ),
            "waypoints": node.create_publisher(
                Path, "/drone/mission_waypoints", 10
            ),
            "command": node.create_publisher(
                String, "/drone/mission_command", 10
            ),
        }

    def _drain_commands(
        self,
        node: Node,
        publishers: Dict[str, object],
    ) -> None:
        while True:
            try:
                command, payload = self._commands.get_nowait()
            except Empty:
                return
            if command == "goal":
                message = _pose(node, payload)
                publishers["goal"].publish(message)
            elif command == "waypoints":
                message = Path()
                message.header.frame_id = "map"
                message.header.stamp = node.get_clock().now().to_msg()
                message.poses = [
                    _pose(node, point) for point in payload
                ]
                publishers["waypoints"].publish(message)
            elif command == "mission_command":
                message = String()
                message.data = str(payload).upper()
                publishers["command"].publish(message)

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self.telemetry.emit({
            "time": time.time(),
            "x": position.x,
            "y": position.y,
            "z": position.z,
            "vx": linear.x,
            "vy": linear.y,
            "vz": linear.z,
            "wx": angular.x,
            "wy": angular.y,
            "wz": angular.z,
        })

    def _imu_callback(self, message: Imu) -> None:
        orientation = message.orientation
        self.telemetry.emit({
            "time": time.time(),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": _yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        })

    def _rpm_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) == 4:
            self.telemetry.emit({
                "time": time.time(),
                **{
                    f"rpm{index + 1}": float(value)
                    for index, value in enumerate(message.data)
                },
            })

    def _obstacle_callback(self, message: MarkerArray) -> None:
        boxes = []
        for marker in message.markers:
            if marker.type != Marker.CUBE:
                continue
            boxes.append({
                "x": marker.pose.position.x,
                "y": marker.pose.position.y,
                "sx": marker.scale.x,
                "sy": marker.scale.y,
                "name": marker.text,
            })
        self.obstacles.emit(boxes)


def _pose(node: Node, values: Sequence[float]) -> PoseStamped:
    if len(values) < 3:
        raise ValueError("goal requires x, y, z")
    yaw = float(values[3]) if len(values) > 3 else 0.0
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = float(values[0])
    message.pose.position.y = float(values[1])
    message.pose.position.z = float(values[2])
    message.pose.orientation.z = math.sin(0.5 * yaw)
    message.pose.orientation.w = math.cos(0.5 * yaw)
    return message


def _path_points(message: Path) -> list[tuple[float, float, float]]:
    return [
        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        for pose in message.poses
    ]


def _json(value: str) -> Dict[str, object]:
    try:
        payload = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return {"state": "INVALID", "raw": value}
    return payload if isinstance(payload, dict) else {"value": payload}


def _yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        return 0.0
    return math.atan2(
        2.0 * (w * z + x * y) / (norm * norm),
        1.0 - 2.0 * (y * y + z * z) / (norm * norm),
    )
