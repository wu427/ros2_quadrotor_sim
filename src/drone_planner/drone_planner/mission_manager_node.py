"""Multi-waypoint mission sequencing over the single-goal planner."""

from __future__ import annotations

import json
import math
from typing import List

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_msgs.msg import Float32


class MissionManagerNode(Node):
    """Queue a ``nav_msgs/Path`` and dispatch each pose in order."""

    def __init__(self) -> None:
        super().__init__("mission_manager_node")
        durable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/drone/mission_goal", 10
        )
        self._status_publisher = self.create_publisher(
            String, "/drone/mission_status", durable_qos
        )
        self._progress_publisher = self.create_publisher(
            Float32, "/drone/mission_progress", durable_qos
        )
        self._path_subscription = self.create_subscription(
            Path,
            "/drone/mission_waypoints",
            self._path_callback,
            10,
        )
        self._command_subscription = self.create_subscription(
            String,
            "/drone/mission_command",
            self._command_callback,
            10,
        )
        self._planner_subscription = self.create_subscription(
            String,
            "/drone/planner_status",
            self._planner_callback,
            durable_qos,
        )
        self._waypoints: List[PoseStamped] = []
        self._index = 0
        self._state = "IDLE"
        self._planner_state = ""
        self._publish_status("waiting_for_path")

    def _path_callback(self, message: Path) -> None:
        if message.header.frame_id not in {"", "map"}:
            self._set_state("FAILED", "INVALID_PATH_FRAME")
            return
        waypoints = []
        for source in message.poses:
            values = (
                source.pose.position.x,
                source.pose.position.y,
                source.pose.position.z,
                source.pose.orientation.x,
                source.pose.orientation.y,
                source.pose.orientation.z,
                source.pose.orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                self._set_state("FAILED", "NON_FINITE_WAYPOINT")
                return
            target = PoseStamped()
            target.header.frame_id = "map"
            target.pose = source.pose
            waypoints.append(target)
        if not waypoints:
            self._set_state("FAILED", "EMPTY_MISSION")
            return
        self._waypoints = waypoints
        self._index = 0
        self._set_state("READY", "path_loaded")

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().upper()
        if command == "START":
            if not self._waypoints:
                self._set_state("FAILED", "NO_PATH_LOADED")
                return
            self._index = 0
            self._set_state("RUNNING", "mission_started")
            self._publish_current_goal()
        elif command == "PAUSE" and self._state == "RUNNING":
            self._set_state("PAUSED", "operator_pause")
        elif command == "RESUME" and self._state == "PAUSED":
            self._set_state("RUNNING", "operator_resume")
        elif command == "CANCEL" and self._state in {
            "READY",
            "RUNNING",
            "PAUSED",
        }:
            self._set_state("CANCELLED", "operator_cancel")
        elif command == "CLEAR":
            self._waypoints = []
            self._index = 0
            self._set_state("IDLE", "operator_clear")

    def _planner_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._set_state("FAILED", "INVALID_PLANNER_STATUS")
            return
        if not isinstance(payload, dict):
            self._set_state("FAILED", "INVALID_PLANNER_STATUS")
            return
        self._planner_state = str(payload.get("state", ""))
        if self._state != "RUNNING":
            return
        if self._planner_state == "FAILED":
            self._set_state(
                "FAILED",
                f"PLANNER:{payload.get('reason', 'UNKNOWN')}",
            )
        elif self._planner_state == "COMPLETED":
            if self._index + 1 >= len(self._waypoints):
                self._set_state("COMPLETED", "all_waypoints_completed")
            else:
                self._index += 1
                self._publish_current_goal()
                self._publish_status("waypoint_advanced")

    def _publish_current_goal(self) -> None:
        message = self._waypoints[self._index]
        message.header.stamp = self.get_clock().now().to_msg()
        self._goal_publisher.publish(message)
        self._publish_progress()

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._publish_status(reason)
        self._publish_progress()

    def _publish_status(self, reason: str) -> None:
        count = len(self._waypoints)
        completed = (
            count
            if self._state == "COMPLETED"
            else min(self._index, count)
        )
        message = String()
        message.data = json.dumps({
            "state": self._state,
            "reason": reason,
            "waypoint_index": self._index,
            "waypoint_count": count,
            "progress": completed / count if count else 0.0,
            "planner_state": self._planner_state,
        }, sort_keys=True, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _publish_progress(self) -> None:
        count = len(self._waypoints)
        completed = (
            count
            if self._state == "COMPLETED"
            else min(self._index, count)
        )
        message = Float32()
        message.data = completed / count if count else 0.0
        self._progress_publisher.publish(message)


def main(args=None) -> None:
    """Run the multi-waypoint mission manager."""
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
