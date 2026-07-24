"""ROS2 static-map planner and sequential waypoint executor."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time
from typing import Dict, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import load_static_map
from drone_planner.astar_3d import AStar3D
from drone_planner.astar_3d import PlanningResult
from drone_planner.astar_3d import path_length
from drone_planner.path_simplifier import simplify_path


def _yaw_from_quaternion(values: Sequence[float]) -> float:
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-9:
        return 0.0
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _yaw_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def _angle_error(first: float, second: float) -> float:
    return abs(math.atan2(
        math.sin(first - second),
        math.cos(first - second),
    ))


class PlannerNode(Node):
    """Plan on mission goals and feed safe waypoints to the controller."""

    def __init__(self) -> None:
        super().__init__("planner_node")
        default_map = (
            Path(get_package_share_directory("drone_map"))
            / "config"
            / "static_map.yaml"
        )
        map_file = self.declare_parameter(
            "map_file", str(default_map)
        ).value
        self._waypoint_tolerance = self._positive_parameter(
            "waypoint_tolerance", 0.18
        )
        self._final_position_tolerance = self._positive_parameter(
            "final_position_tolerance", 0.08
        )
        self._final_yaw_tolerance = math.radians(
            self._positive_parameter("final_yaw_tolerance_deg", 5.0)
        )
        self._final_linear_speed = self._positive_parameter(
            "final_linear_speed_tolerance", 0.08
        )
        self._final_angular_speed = self._positive_parameter(
            "final_angular_speed_tolerance", 0.08
        )
        self._completion_stable_sec = self._positive_parameter(
            "completion_stable_sec", 1.0
        )
        max_expanded = int(self.declare_parameter(
            "max_expanded_nodes", 100000
        ).value)
        timeout = self._positive_parameter(
            "planning_timeout_sec", 2.0
        )
        if max_expanded <= 0:
            raise ValueError("max_expanded_nodes must be positive")

        self._map = load_static_map(str(map_file))
        self._planner = AStar3D(self._map, max_expanded, timeout)
        durable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_publisher = self.create_publisher(
            PoseStamped, "/drone/goal", 10
        )
        self._path_publisher = self.create_publisher(
            PathMessage, "/drone/planned_path", durable_qos
        )
        self._waypoint_publisher = self.create_publisher(
            Marker, "/drone/current_waypoint", durable_qos
        )
        self._mission_marker_publisher = self.create_publisher(
            Marker, "/drone/mission_goal_marker", durable_qos
        )
        self._status_publisher = self.create_publisher(
            String, "/drone/planner_status", durable_qos
        )
        self._mission_subscription = self.create_subscription(
            PoseStamped,
            "/drone/mission_goal",
            self._mission_callback,
            10,
        )
        self._odometry_subscription = self.create_subscription(
            Odometry,
            "/drone/odom",
            self._odometry_callback,
            10,
        )

        self._state = "IDLE"
        self._latest_position: Optional[Point3] = None
        self._latest_yaw = 0.0
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        self._waypoints: Tuple[Point3, ...] = ()
        self._waypoint_index = 0
        self._mission_yaw = 0.0
        self._mission_quaternion = (0.0, 0.0, 0.0, 1.0)
        self._stable_since: Optional[float] = None
        self._mission_started: Optional[float] = None
        self._metrics: Dict[str, object] = {}
        self._publish_status("IDLE", "waiting_for_mission")
        self.get_logger().info(
            "Planner ready: resolution %.3f m, %d obstacles"
            % (self._map.grid_resolution, len(self._map.obstacles))
        )

    def _positive_parameter(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _mission_callback(self, message: PoseStamped) -> None:
        if self._latest_position is None:
            self._fail("ODOMETRY_NOT_READY")
            return
        if message.header.frame_id != "map":
            self._fail("INVALID_FRAME")
            return
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        quaternion = (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in position + quaternion):
            self._fail("NON_FINITE_MISSION_GOAL")
            return
        quaternion_norm = math.sqrt(
            sum(value * value for value in quaternion)
        )
        if quaternion_norm <= 1.0e-9:
            quaternion = (0.0, 0.0, 0.0, 1.0)
        else:
            quaternion = tuple(
                value / quaternion_norm for value in quaternion
            )

        self._state = "PLANNING"
        self._waypoints = ()
        self._stable_since = None
        self._metrics = {}
        self._publish_status("PLANNING", "new_mission")
        result = self._planner.plan(self._latest_position, position)
        if not result.success:
            self._record_planning_metrics(result)
            self._fail(result.reason)
            return
        try:
            simplified = simplify_path(result.path, self._map)
        except (ValueError, RuntimeError) as error:
            self._record_planning_metrics(result)
            self._fail(f"SIMPLIFICATION_ERROR:{error}")
            return

        self._waypoints = simplified
        self._waypoint_index = 1 if len(simplified) > 1 else 0
        self._mission_yaw = _yaw_from_quaternion(quaternion)
        self._mission_quaternion = quaternion
        self._mission_started = time.monotonic()
        self._record_planning_metrics(result)
        self._metrics.update({
            "simplified_waypoints": len(simplified),
            "planned_path_length": path_length(simplified),
            "planned_path_clearance": self._map.minimum_clearance(
                simplified
            ),
        })
        self._publish_path(simplified)
        self._publish_mission_marker(position)
        self._state = "EXECUTING"
        self._publish_current_waypoint()
        self._publish_control_goal()
        self._publish_status("EXECUTING", "path_ready")
        self.get_logger().info(
            "Plan ready: raw=%d simplified=%d expanded=%d time=%.4f s"
            % (
                result.raw_path_points,
                len(simplified),
                result.expanded_nodes,
                result.planning_time_sec,
            )
        )

    def _odometry_callback(self, message: Odometry) -> None:
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning("Rejected non-finite odometry")
            return
        first_valid_odometry = self._latest_position is None
        self._latest_position = values[0], values[1], values[2]
        self._latest_yaw = _yaw_from_quaternion(values[3:7])
        self._linear_speed = math.sqrt(
            sum(value * value for value in values[7:10])
        )
        self._angular_speed = math.sqrt(
            sum(value * value for value in values[10:13])
        )
        if first_valid_odometry and self._state == "IDLE":
            self._publish_status("IDLE", "odometry_ready")
        if self._state != "EXECUTING" or not self._waypoints:
            return

        current_goal = self._waypoints[self._waypoint_index]
        position_error = math.dist(self._latest_position, current_goal)
        if (
            self._waypoint_index < len(self._waypoints) - 1
            and position_error <= self._waypoint_tolerance
        ):
            self._waypoint_index += 1
            self._stable_since = None
            self._publish_current_waypoint()
            self._publish_control_goal()
            self._publish_status("EXECUTING", "waypoint_advanced")
            return

        if self._waypoint_index != len(self._waypoints) - 1:
            return
        yaw_error = _angle_error(self._latest_yaw, self._mission_yaw)
        stable = (
            position_error <= self._final_position_tolerance
            and yaw_error <= self._final_yaw_tolerance
            and self._linear_speed <= self._final_linear_speed
            and self._angular_speed <= self._final_angular_speed
        )
        now = time.monotonic()
        if not stable:
            self._stable_since = None
            return
        if self._stable_since is None:
            self._stable_since = now
            return
        if now - self._stable_since < self._completion_stable_sec:
            return
        self._state = "COMPLETED"
        self._metrics["mission_completion_time_sec"] = (
            now - self._mission_started
            if self._mission_started is not None
            else 0.0
        )
        self._metrics["final_position_error"] = position_error
        self._metrics["final_yaw_error_deg"] = math.degrees(yaw_error)
        self._publish_status("COMPLETED", "goal_stable")
        self.get_logger().info("Mission completed")

    def _record_planning_metrics(self, result: PlanningResult) -> None:
        self._metrics = {
            "planning_time_sec": result.planning_time_sec,
            "expanded_nodes": result.expanded_nodes,
            "raw_path_points": result.raw_path_points,
            "raw_path_length": result.raw_path_length,
        }

    def _publish_path(self, path: Sequence[Point3]) -> None:
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            yaw = self._waypoint_yaw(index)
            quaternion = _yaw_quaternion(yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            message.poses.append(pose)
        self._path_publisher.publish(message)

    def _waypoint_yaw(self, index: int) -> float:
        if index >= len(self._waypoints) - 1:
            return self._mission_yaw
        current = self._waypoints[index]
        following = self._waypoints[index + 1]
        return math.atan2(
            following[1] - current[1],
            following[0] - current[0],
        )

    def _publish_control_goal(self) -> None:
        point = self._waypoints[self._waypoint_index]
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = point[0]
        message.pose.position.y = point[1]
        message.pose.position.z = point[2]
        if self._waypoint_index == len(self._waypoints) - 1:
            quaternion = self._mission_quaternion
        else:
            quaternion = _yaw_quaternion(
                self._waypoint_yaw(self._waypoint_index)
            )
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        self._goal_publisher.publish(message)

    def _publish_current_waypoint(self) -> None:
        point = self._waypoints[self._waypoint_index]
        marker = self._point_marker(
            point,
            "current_waypoint",
            0,
            (1.0, 0.65, 0.05, 0.95),
            0.24,
        )
        self._waypoint_publisher.publish(marker)

    def _publish_mission_marker(self, point: Point3) -> None:
        marker = self._point_marker(
            point,
            "mission_goal",
            0,
            (0.15, 0.95, 0.25, 0.95),
            0.32,
        )
        self._mission_marker_publisher.publish(marker)

    def _point_marker(
        self,
        point: Point3,
        namespace: str,
        marker_id: int,
        colour: Tuple[float, float, float, float],
        scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = colour[0]
        marker.color.g = colour[1]
        marker.color.b = colour[2]
        marker.color.a = colour[3]
        return marker

    def _fail(self, reason: str) -> None:
        self._state = "FAILED"
        self._waypoints = ()
        self._stable_since = None
        self._publish_status("FAILED", reason)
        self.get_logger().error(f"Planning mission failed: {reason}")

    def _publish_status(self, state: str, reason: str) -> None:
        payload: Dict[str, object] = {
            "state": state,
            "reason": reason,
            "waypoint_index": self._waypoint_index,
            "waypoint_count": len(self._waypoints),
        }
        payload.update(self._metrics)
        message = String()
        message.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
        )
        self._status_publisher.publish(message)


def main(args=None) -> None:
    """Run the static-map planner node."""
    rclpy.init(args=args)
    node = PlannerNode()
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
