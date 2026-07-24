"""End-to-end acceptance node for static-map planning and actual flight."""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

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


def _path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(
        math.dist(first, second)
        for first, second in zip(path, path[1:])
    )


def _yaw(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        return 0.0
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PlanningAcceptanceNode(Node):
    """Publish a mission and verify planned path plus actual odometry."""

    def __init__(self) -> None:
        super().__init__("planning_acceptance_node")
        self._scenario = str(
            self.declare_parameter("scenario", "multi").value
        )
        map_file = str(self.declare_parameter("map_file", "").value)
        self._map = load_static_map(map_file)
        self._target = (
            self._finite_parameter("target_x", 5.0),
            self._finite_parameter("target_y", 0.0),
            self._finite_parameter("target_z", 1.5),
        )
        self._target_yaw = math.radians(
            self._finite_parameter("target_yaw_deg", 20.0)
        )
        self._discovery_timeout = self._positive_parameter(
            "discovery_timeout_sec", 15.0
        )
        self._mission_timeout = self._positive_parameter(
            "mission_timeout_sec", 90.0
        )
        self._position_tolerance = self._positive_parameter(
            "position_tolerance", 0.12
        )
        self._yaw_tolerance = math.radians(
            self._positive_parameter("yaw_tolerance_deg", 6.0)
        )

        durable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mission_publisher = self.create_publisher(
            PoseStamped, "/drone/mission_goal", 10
        )
        self._odom_subscription = self.create_subscription(
            Odometry, "/drone/odom", self._odom_callback, 10
        )
        self._path_subscription = self.create_subscription(
            Path,
            "/drone/planned_path",
            self._path_callback,
            durable_qos,
        )
        self._status_subscription = self.create_subscription(
            String,
            "/drone/planner_status",
            self._status_callback,
            durable_qos,
        )
        self._timer = self.create_timer(0.1, self._timer_callback)

        self._created = time.monotonic()
        self._mission_sent_at: Optional[float] = None
        self._latest_odom: Optional[Odometry] = None
        self._start: Optional[Point3] = None
        self._actual_points: List[Point3] = []
        self._actual_length = 0.0
        self._actual_clearance = math.inf
        self._collision_count = 0
        self._in_collision = False
        self._path_points: Tuple[Point3, ...] = ()
        self._planned_length = 0.0
        self._planned_clearance = math.inf
        self._planned_collision = True
        self._direct_blocked = False
        self._planner_state = ""
        self._planner_reason = ""
        self._planner_metrics: Dict[str, object] = {}
        self._completed = False
        self.finished = False
        self.exit_code = 2
        self.get_logger().info(
            "Planning acceptance scenario '%s', target=(%.2f, %.2f, %.2f)"
            % (self._scenario, *self._target)
        )

    def _finite_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _positive_parameter(self, name: str, default: float) -> float:
        value = self._finite_parameter(name, default)
        if value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _odom_callback(self, message: Odometry) -> None:
        point = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        quaternion = (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in point + quaternion):
            return
        self._latest_odom = message
        if self._start is None:
            self._start = point
        if self._mission_sent_at is None:
            return
        if self._actual_points and math.dist(
            self._actual_points[-1], point
        ) <= 1.0e-6:
            return
        if self._actual_points:
            previous = self._actual_points[-1]
            self._actual_length += math.dist(previous, point)
            segment_collision = any(
                obstacle.intersects_segment(previous, point)
                for obstacle in self._map.obstacles
            )
            segment_clearance = min(
                (
                    obstacle.segment_distance(previous, point)
                    for obstacle in self._map.obstacles
                ),
                default=math.inf,
            )
            self._actual_clearance = min(
                self._actual_clearance, segment_clearance
            )
        else:
            segment_collision = any(
                obstacle.contains(point)
                for obstacle in self._map.obstacles
            )
            self._actual_clearance = min(
                (
                    obstacle.point_distance(point)
                    for obstacle in self._map.obstacles
                ),
                default=math.inf,
            )
        if segment_collision and not self._in_collision:
            self._collision_count += 1
        self._in_collision = segment_collision
        self._actual_points.append(point)

    def _path_callback(self, message: Path) -> None:
        if self._mission_sent_at is None or not message.poses:
            return
        points = tuple(
            (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )
            for pose in message.poses
        )
        if not all(
            math.isfinite(value)
            for point in points
            for value in point
        ):
            self._finish_failure("planned path contains non-finite values")
            return
        self._path_points = points
        self._planned_length = _path_length(points)
        self._planned_collision = self._map.path_collides(points)
        self._planned_clearance = self._map.minimum_clearance(points)
        if self._start is not None:
            self._direct_blocked = not self._map.segment_is_collision_free(
                self._start, self._target
            )

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._finish_failure("planner status is not valid JSON")
            return
        if not isinstance(payload, dict):
            self._finish_failure("planner status is not a JSON object")
            return
        self._planner_state = str(payload.get("state", ""))
        self._planner_reason = str(payload.get("reason", ""))
        self._planner_metrics = payload
        if self._mission_sent_at is None:
            return
        if self._planner_state == "FAILED":
            self._finish_failure(
                f"planner returned FAILED: {self._planner_reason}"
            )
        elif self._planner_state == "COMPLETED":
            self._completed = True

    def _timer_callback(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if self._mission_sent_at is None:
            ready = (
                self._latest_odom is not None
                and self._planner_state == "IDLE"
                and self._planner_reason == "odometry_ready"
                and self._mission_publisher.get_subscription_count() > 0
            )
            if ready:
                self._publish_mission()
                self._mission_sent_at = now
                if self._start is not None:
                    self._actual_points.append(self._start)
                return
            if now - self._created >= self._discovery_timeout:
                self._finish_failure(
                    "discovery timeout waiting for planner and odometry"
                )
            return
        if self._completed:
            self._evaluate_success()
            return
        if now - self._mission_sent_at >= self._mission_timeout:
            self._finish_failure(
                f"mission timeout in planner state {self._planner_state}"
            )

    def _publish_mission(self) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self._target[0]
        message.pose.position.y = self._target[1]
        message.pose.position.z = self._target[2]
        message.pose.orientation.z = math.sin(0.5 * self._target_yaw)
        message.pose.orientation.w = math.cos(0.5 * self._target_yaw)
        self._mission_publisher.publish(message)
        self.get_logger().info("Mission goal published")

    def _final_errors(self) -> Tuple[float, float]:
        if self._latest_odom is None:
            return math.inf, math.inf
        pose = self._latest_odom.pose.pose
        position = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
        )
        yaw = _yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        yaw_error = abs(math.atan2(
            math.sin(yaw - self._target_yaw),
            math.cos(yaw - self._target_yaw),
        ))
        return math.dist(position, self._target), yaw_error

    def _evaluate_success(self) -> None:
        position_error, yaw_error = self._final_errors()
        failures = []
        if not self._path_points:
            failures.append("no planned path received")
        if not self._direct_blocked:
            failures.append("direct start-goal segment was not blocked")
        if self._planned_collision:
            failures.append("planned path collides with inflated obstacles")
        if self._collision_count:
            failures.append("actual odometry trajectory collided")
        if self._actual_clearance < self._map.drone_radius:
            failures.append("actual trajectory violated drone-radius clearance")
        if position_error > self._position_tolerance:
            failures.append("final position error exceeds tolerance")
        if yaw_error > self._yaw_tolerance:
            failures.append("final yaw error exceeds tolerance")
        if failures:
            self._finish_failure("; ".join(failures))
            return
        self._finish_success(position_error, yaw_error)

    def _metric_lines(
        self,
        position_error: float,
        yaw_error: float,
    ) -> str:
        elapsed = (
            time.monotonic() - self._mission_sent_at
            if self._mission_sent_at is not None
            else 0.0
        )
        return (
            f"Scenario: {self._scenario}\n"
            f"Planning time: "
            f"{float(self._planner_metrics.get('planning_time_sec', 0.0)):.6f} s\n"
            f"Expanded nodes: "
            f"{int(self._planner_metrics.get('expanded_nodes', 0))}\n"
            f"Raw path points: "
            f"{int(self._planner_metrics.get('raw_path_points', 0))}\n"
            f"Simplified waypoints: {len(self._path_points)}\n"
            f"Planned path length: {self._planned_length:.6f} m\n"
            f"Actual trajectory length: {self._actual_length:.6f} m\n"
            f"Minimum planned-path clearance: "
            f"{self._planned_clearance:.6f} m\n"
            f"Minimum actual-trajectory clearance: "
            f"{self._actual_clearance:.6f} m\n"
            f"Final position error: {position_error:.6f} m\n"
            f"Final yaw error: {math.degrees(yaw_error):.6f} deg\n"
            f"Mission completion time: {elapsed:.6f} s\n"
            f"Collision count: {self._collision_count}"
        )

    def _finish_success(
        self,
        position_error: float,
        yaw_error: float,
    ) -> None:
        self.get_logger().info(
            "\n========================================\n"
            "    PLANNING ACCEPTANCE: PASS\n"
            "========================================\n"
            + self._metric_lines(position_error, yaw_error)
            + "\n========================================"
        )
        self._finish(0)

    def _finish_failure(self, reason: str) -> None:
        if self.finished:
            return
        position_error, yaw_error = self._final_errors()
        self.get_logger().error(
            "\n========================================\n"
            "    PLANNING ACCEPTANCE: FAIL\n"
            "========================================\n"
            f"Reason: {reason}\n"
            f"Planner state: {self._planner_state}\n"
            f"Planner reason: {self._planner_reason}\n"
            + self._metric_lines(position_error, yaw_error)
            + "\n========================================"
        )
        self._finish(1)

    def _finish(self, code: int) -> None:
        self.finished = True
        self.exit_code = code
        self._timer.cancel()


def main(args=None) -> int:
    """Run planning acceptance and return a process-compatible exit code."""
    rclpy.init(args=args)
    node: Optional[PlanningAcceptanceNode] = None
    exit_code = 2
    try:
        node = PlanningAcceptanceNode()
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
