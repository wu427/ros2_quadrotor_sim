"""Automated positive and negative acceptance for showcase missions."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import load_static_map


class ShowcaseAcceptanceNode(Node):
    """Drive one showcase scenario and evaluate terminal evidence."""

    def __init__(self) -> None:
        super().__init__("showcase_acceptance_node")
        self._scenario = str(
            self.declare_parameter("scenario", "default").value
        )
        self._map = load_static_map(
            str(self.declare_parameter("map_file", "").value)
        )
        self._target = (
            float(self.declare_parameter("target_x", 5.0).value),
            float(self.declare_parameter("target_y", 0.0).value),
            float(self.declare_parameter("target_z", 1.5).value),
        )
        self._timeout = float(
            self.declare_parameter("timeout_sec", 150.0).value
        )
        self._position_tolerance = float(
            self.declare_parameter("position_tolerance", 0.15).value
        )
        self._minimum_flight_z = float(
            self.declare_parameter("minimum_flight_z", 1.0).value
        )
        if (
            self._timeout <= 0.0
            or self._position_tolerance <= 0.0
            or self._minimum_flight_z < 0.0
        ):
            raise ValueError("acceptance time and position limits must be positive")
        durable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/drone/mission_goal", 10
        )
        self._path_publisher = self.create_publisher(
            Path, "/drone/mission_waypoints", 10
        )
        self._command_publisher = self.create_publisher(
            String, "/drone/mission_command", 10
        )
        self.create_subscription(
            Odometry, "/drone/odom", self._odom_callback, 10
        )
        self.create_subscription(
            Path,
            "/drone/planned_path",
            self._path_callback,
            durable,
        )
        self.create_subscription(
            String,
            "/drone/planner_status",
            self._planner_callback,
            durable,
        )
        self.create_subscription(
            String,
            "/drone/mission_status",
            self._mission_callback,
            durable,
        )
        self._timer = self.create_timer(0.1, self._tick)
        self._created = time.monotonic()
        self._started_at: Optional[float] = None
        self._latest_position: Optional[Point3] = None
        self._previous_position: Optional[Point3] = None
        self._actual_length = 0.0
        self._collision_count = 0
        self._in_collision = False
        self._planned_paths = 0
        self._path_collision = False
        self._minimum_clearance = math.inf
        self._minimum_actual_clearance = math.inf
        self._minimum_path_z = math.inf
        self._planner: Dict[str, object] = {}
        self._mission: Dict[str, object] = {}
        self._stage = "DISCOVERY"
        self._cancel_sent = False
        self.finished = False
        self.exit_code = 2

    def _odom_callback(self, message: Odometry) -> None:
        point = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        if not all(math.isfinite(value) for value in point):
            return
        self._latest_position = point
        if self._started_at is None:
            return
        if self._previous_position is not None:
            self._actual_length += math.dist(
                self._previous_position, point
            )
            if self._map.obstacles:
                self._minimum_actual_clearance = min(
                    self._minimum_actual_clearance,
                    *(
                        obstacle.segment_distance(
                            self._previous_position, point
                        )
                        for obstacle in self._map.obstacles
                    ),
                )
            colliding = any(
                obstacle.intersects_segment(
                    self._previous_position, point
                )
                for obstacle in self._map.obstacles
            )
            if colliding and not self._in_collision:
                self._collision_count += 1
            self._in_collision = colliding
        self._previous_position = point

    def _path_callback(self, message: Path) -> None:
        if self._started_at is None or not message.poses:
            return
        points = tuple(
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in message.poses
        )
        self._planned_paths += 1
        self._path_collision = (
            self._path_collision or self._map.path_collides(points)
        )
        self._minimum_clearance = min(
            self._minimum_clearance,
            self._map.minimum_clearance(points),
        )
        self._minimum_path_z = min(
            self._minimum_path_z,
            *(point[2] for point in points),
        )

    def _planner_callback(self, message: String) -> None:
        self._planner = _payload(message.data)

    def _mission_callback(self, message: String) -> None:
        self._mission = _payload(message.data)

    def _tick(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if self._started_at is None:
            ready = (
                self._latest_position is not None
                and self._planner.get("state") == "IDLE"
                and self._goal_publisher.get_subscription_count() > 0
            )
            if ready:
                self._started_at = now
                self._previous_position = self._latest_position
                if self._scenario in {"multi_segment", "square"}:
                    self._publish_multi_path()
                    self._stage = "PATH_SENT"
                else:
                    self._publish_goal()
                    self._stage = "RUNNING"
                return
            if now - self._created > 20.0:
                self._finish(False, "discovery timeout")
            return
        if self._stage == "PATH_SENT" and now - self._started_at > 0.5:
            self._publish_command("START")
            self._stage = "RUNNING"
        if (
            self._scenario == "cancel"
            and not self._cancel_sent
            and self._planner.get("state") in {
                "TAKEOFF",
                "EXECUTING",
                "FINAL_APPROACH",
            }
        ):
            self._publish_command("CANCEL")
            self._cancel_sent = True

        planner_state = str(self._planner.get("state", ""))
        mission_state = str(self._mission.get("state", ""))
        if self._scenario in {"invalid_goal", "no_path"}:
            if planner_state == "FAILED":
                self._finish(False, f"expected {planner_state}")
                return
        elif self._scenario == "cancel":
            if planner_state == "CANCELLED":
                self._finish(False, "expected cancellation")
                return
        elif self._scenario in {"multi_segment", "square"}:
            if mission_state == "FAILED" or planner_state == "FAILED":
                self._finish(False, "multi-segment mission failed")
                return
            if mission_state == "COMPLETED":
                self._evaluate_positive()
                return
        else:
            if planner_state == "FAILED":
                self._finish(False, "planner failed")
                return
            if planner_state == "COMPLETED":
                self._evaluate_positive()
                return
        if now - self._started_at > self._timeout:
            self._finish(False, "scenario timeout")

    def _publish_goal(self) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self._target[0]
        message.pose.position.y = self._target[1]
        message.pose.position.z = self._target[2]
        message.pose.orientation.w = 1.0
        self._goal_publisher.publish(message)

    def _publish_multi_path(self) -> None:
        if self._scenario == "square":
            points = (
                (1.0, -1.0, 1.5),
                (3.0, -1.0, 1.5),
                (3.0, 1.0, 1.5),
                (1.0, 1.0, 1.5),
                (1.0, -1.0, 1.5),
            )
            self._target = points[-1]
        else:
            points = (
                (1.0, 1.0, 1.4),
                (2.5, -1.0, 1.6),
                self._target,
            )
        message = Path()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self._path_publisher.publish(message)

    def _publish_command(self, command: str) -> None:
        message = String()
        message.data = command
        self._command_publisher.publish(message)

    def _evaluate_positive(self) -> None:
        failures: List[str] = []
        position_error = (
            math.dist(self._latest_position, self._target)
            if self._latest_position is not None
            else math.inf
        )
        if self._planned_paths == 0:
            failures.append("no planned path")
        if self._path_collision:
            failures.append("planned path collision")
        if self._minimum_path_z < self._minimum_flight_z - 1.0e-9:
            failures.append("planned path below minimum flight height")
        if self._collision_count:
            failures.append("actual collision")
        if position_error > self._position_tolerance:
            failures.append("final position error")
        self._finish(not failures, "; ".join(failures) or "completed")

    def _finish(self, success: bool, reason: str) -> None:
        elapsed = (
            time.monotonic() - self._started_at
            if self._started_at is not None
            else 0.0
        )
        position_error = (
            math.dist(self._latest_position, self._target)
            if self._latest_position is not None
            else math.inf
        )
        metrics = {
            "scenario": self._scenario,
            "reason": reason,
            "elapsed_sec": elapsed,
            "planning_time_sec": self._planner.get(
                "planning_time_sec", 0.0
            ),
            "expanded_nodes": self._planner.get("expanded_nodes", 0),
            "planned_paths": self._planned_paths,
            "minimum_clearance": (
                self._minimum_clearance
                if math.isfinite(self._minimum_clearance)
                else None
            ),
            "minimum_actual_clearance": (
                self._minimum_actual_clearance
                if math.isfinite(self._minimum_actual_clearance)
                else None
            ),
            "minimum_path_z": (
                self._minimum_path_z
                if math.isfinite(self._minimum_path_z)
                else None
            ),
            "actual_path_length": self._actual_length,
            "final_position_error": position_error,
            "collision_count": self._collision_count,
            "planner_state": self._planner.get("state", ""),
            "mission_state": self._mission.get("state", ""),
        }
        marker = "PASS" if success else "FAIL"
        logger = self.get_logger().info if success else self.get_logger().error
        logger(
            "\n========================================\n"
            f"    SHOWCASE ACCEPTANCE: {marker}\n"
            "========================================\n"
            f"SHOWCASE_METRICS_JSON={json.dumps(metrics, sort_keys=True)}\n"
            "========================================"
        )
        self.finished = True
        self.exit_code = 0 if success else 1
        self._timer.cancel()


def _payload(value: str) -> Dict[str, object]:
    try:
        result = json.loads(value)
    except (TypeError, json.JSONDecodeError):
        return {}
    return result if isinstance(result, dict) else {}


def main(args=None) -> int:
    """Run showcase acceptance and return a process exit code."""
    rclpy.init(args=args)
    node: Optional[ShowcaseAcceptanceNode] = None
    exit_code = 2
    try:
        node = ShowcaseAcceptanceNode()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
