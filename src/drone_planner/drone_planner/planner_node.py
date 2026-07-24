"""ROS2 static-map planner with takeoff and continuous path following."""

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
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from drone_map.collision_geometry import Point3
from drone_map.collision_geometry import load_static_map
from drone_planner.astar_3d import AStar3D
from drone_planner.astar_3d import PlanningResult
from drone_planner.astar_3d import path_length
from drone_planner.path_processing import densely_collision_free
from drone_planner.path_processing import process_path
from drone_planner.path_processing import resample_by_arc_length
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
    """Plan collision-free missions and continuously feed controller goals."""

    _ACTIVE_STATES = {
        "TAKEOFF",
        "EXECUTING",
        "FINAL_APPROACH",
    }

    def __init__(self) -> None:
        super().__init__("planner_node")
        default_map = (
            Path(get_package_share_directory("drone_map"))
            / "config"
            / "static_map.yaml"
        )
        map_file = str(
            self.declare_parameter("map_file", str(default_map)).value
        )
        self._waypoint_tolerance = self._positive_parameter(
            "waypoint_pass_radius", 0.22
        )
        self._lookahead_distance = self._positive_parameter(
            "lookahead_distance", 0.55
        )
        self._takeoff_altitude = self._positive_parameter(
            "cruise_altitude", 1.50
        )
        self._minimum_flight_altitude = self._positive_parameter(
            "minimum_flight_z", 1.00
        )
        self._takeoff_required = bool(
            self.declare_parameter("takeoff_required", True).value
        )
        if self._takeoff_altitude < self._minimum_flight_altitude:
            raise ValueError(
                "takeoff_altitude must be >= minimum_flight_altitude"
            )
        self._takeoff_tolerance = self._positive_parameter(
            "takeoff_tolerance", 0.12
        )
        self._path_sample_spacing = self._positive_parameter(
            "path_sample_spacing", 0.25
        )
        self._collision_check_step = self._positive_parameter(
            "collision_check_step", 0.05
        )
        self._smoothing_iterations = int(
            self.declare_parameter("smoothing_iterations", 2).value
        )
        self._smoothing_enabled = bool(
            self.declare_parameter("smoothing_enabled", True).value
        )
        if self._smoothing_iterations < 0:
            raise ValueError("smoothing_iterations must be non-negative")
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
            "final_settle_duration", 1.0
        )
        self._reference_update_rate = self._positive_parameter(
            "reference_update_rate", 20.0
        )
        max_expanded = int(
            self.declare_parameter("max_expanded_nodes", 100000).value
        )
        timeout = self._positive_parameter(
            "planning_timeout_sec", 2.0
        )
        if max_expanded <= 0:
            raise ValueError("max_expanded_nodes must be positive")

        self._map = load_static_map(map_file)
        if self._takeoff_altitude > self._map.maximum[2]:
            raise ValueError("takeoff_altitude is above map bounds")
        self._planner = AStar3D(
            self._map,
            max_expanded,
            timeout,
            minimum_z=self._minimum_flight_altitude,
        )
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
        self._progress_publisher = self.create_publisher(
            String, "/drone/path_progress", durable_qos
        )
        self._avoidance_publisher = self.create_publisher(
            Bool, "/drone/avoidance_active", durable_qos
        )
        self._clearance_publisher = self.create_publisher(
            Float32, "/drone/min_obstacle_clearance", durable_qos
        )
        self._waypoint_index_publisher = self.create_publisher(
            Int32, "/drone/current_waypoint_index", durable_qos
        )
        self._planning_metrics_publisher = self.create_publisher(
            String, "/drone/planning_metrics", durable_qos
        )
        self._float_metric_publishers = {
            name: self.create_publisher(
                Float64, f"/drone/metrics/{name}", durable_qos
            )
            for name in (
                "planning_time",
                "path_length",
                "minimum_clearance",
                "actual_path_length",
                "mission_completion_time",
                "final_position_error",
            )
        }
        self._int_metric_publishers = {
            name: self.create_publisher(
                Int32, f"/drone/metrics/{name}", durable_qos
            )
            for name in ("expanded_nodes", "collision_count")
        }
        self._mission_subscription = self.create_subscription(
            PoseStamped,
            "/drone/mission_goal",
            self._mission_callback,
            10,
        )
        self._command_subscription = self.create_subscription(
            String,
            "/drone/mission_command",
            self._command_callback,
            10,
        )
        self._odometry_subscription = self.create_subscription(
            Odometry,
            "/drone/odom",
            self._odometry_callback,
            10,
        )
        self._reference_timer = self.create_timer(
            1.0 / self._reference_update_rate,
            self._reference_callback,
        )

        self._state = "IDLE"
        self._state_before_pause = "IDLE"
        self._latest_position: Optional[Point3] = None
        self._previous_actual_position: Optional[Point3] = None
        self._latest_yaw = 0.0
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        self._path: Tuple[Point3, ...] = ()
        self._progress_index = 0
        self._target_index = 0
        self._takeoff_point: Optional[Point3] = None
        self._mission_position: Optional[Point3] = None
        self._mission_yaw = 0.0
        self._mission_quaternion = (0.0, 0.0, 0.0, 1.0)
        self._stable_since: Optional[float] = None
        self._mission_started: Optional[float] = None
        self._metrics: Dict[str, object] = {}
        self._actual_in_collision = False
        self._avoidance_active = False
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
        if not self._map.contains_point(position):
            self._fail("GOAL_OUT_OF_BOUNDS")
            return
        if self._map.is_occupied(position):
            self._fail("GOAL_OCCUPIED")
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

        self._mission_position = position
        self._mission_quaternion = quaternion
        self._mission_yaw = _yaw_from_quaternion(quaternion)
        self._mission_started = time.monotonic()
        self._path = ()
        self._progress_index = 0
        self._target_index = 0
        self._stable_since = None
        self._previous_actual_position = self._latest_position
        self._actual_in_collision = False
        self._metrics = {
            "actual_path_length": 0.0,
            "collision_count": 0,
        }
        self._publish_mission_marker(position)
        if (
            self._takeoff_required
            and
            self._latest_position[2]
            < self._takeoff_altitude - self._takeoff_tolerance
        ):
            self._takeoff_point = (
                self._latest_position[0],
                self._latest_position[1],
                self._takeoff_altitude,
            )
            if (
                not self._map.is_traversable(self._takeoff_point)
                or not self._map.segment_is_collision_free(
                    self._latest_position, self._takeoff_point
                )
            ):
                self._fail("TAKEOFF_COLUMN_BLOCKED")
                return
            self._set_state("TAKEOFF", "climb_to_cruise_altitude")
            self._publish_pose_goal(
                self._takeoff_point,
                _yaw_quaternion(self._latest_yaw),
            )
            return
        if self._latest_position[2] < self._minimum_flight_altitude:
            self._fail("BELOW_MINIMUM_Z_WITHOUT_TAKEOFF")
            return
        self._takeoff_point = None
        self._begin_planning()

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().upper()
        if command == "PAUSE" and self._state in self._ACTIVE_STATES:
            self._state_before_pause = self._state
            self._set_state("PAUSED", "operator_pause")
            self._publish_hover_goal()
        elif command == "RESUME" and self._state == "PAUSED":
            restored = self._state_before_pause
            self._set_state(restored, "operator_resume")
            if restored == "TAKEOFF" and self._takeoff_point is not None:
                self._publish_pose_goal(
                    self._takeoff_point,
                    _yaw_quaternion(self._latest_yaw),
                )
            elif self._path:
                self._publish_control_goal()
        elif command == "CANCEL" and self._state not in {
            "IDLE",
            "COMPLETED",
            "CANCELLED",
        }:
            self._path = ()
            self._set_state("CANCELLED", "operator_cancel")
            self._publish_hover_goal()
        elif command == "CLEAR" and self._state in {
            "COMPLETED",
            "FAILED",
            "CANCELLED",
        }:
            self._path = ()
            self._mission_position = None
            self._set_state("IDLE", "operator_clear")
        elif command not in {"START", "PAUSE", "RESUME", "CANCEL", "CLEAR"}:
            self.get_logger().warning(
                f"Ignoring unsupported mission command '{command}'"
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
        first_valid = self._latest_position is None
        self._latest_position = values[0], values[1], values[2]
        self._latest_yaw = _yaw_from_quaternion(values[3:7])
        self._linear_speed = math.sqrt(
            sum(value * value for value in values[7:10])
        )
        self._angular_speed = math.sqrt(
            sum(value * value for value in values[10:13])
        )
        if first_valid and self._state == "IDLE":
            self._publish_status("IDLE", "odometry_ready")
        self._update_actual_metrics()
        if self._state == "TAKEOFF" and self._takeoff_point is not None:
            if (
                math.dist(self._latest_position, self._takeoff_point)
                <= self._takeoff_tolerance
            ):
                self._begin_planning()
            return

    def _reference_callback(self) -> None:
        if self._state in {"EXECUTING", "FINAL_APPROACH"}:
            self._follow_path()

    def _begin_planning(self) -> None:
        if self._latest_position is None or self._mission_position is None:
            self._fail("MISSION_CONTEXT_MISSING")
            return
        planning_goal = (
            self._mission_position[0],
            self._mission_position[1],
            max(
                self._mission_position[2],
                self._minimum_flight_altitude,
            ),
        )
        self._set_state("PLANNING", "astar_search")
        direct_start = self._latest_position
        self._avoidance_active = not self._map.segment_is_collision_free(
            direct_start, planning_goal
        )
        result = self._planner.plan(self._latest_position, planning_goal)
        self._record_planning_metrics(result)
        if not result.success:
            self._fail(result.reason)
            return
        try:
            simplified = simplify_path(result.path, self._map)
            smoothing_started = time.monotonic()
            processed = process_path(
                simplified,
                self._map,
                (
                    self._smoothing_iterations
                    if self._smoothing_enabled
                    else 0
                ),
                self._collision_check_step,
                self._path_sample_spacing,
                self._minimum_flight_altitude,
            )
            smoothing_time = time.monotonic() - smoothing_started
            points = processed.points
            if self._mission_position[2] < self._minimum_flight_altitude:
                if not self._map.segment_is_collision_free(
                    points[-1], self._mission_position
                ):
                    self._fail("FINAL_DESCENT_BLOCKED")
                    return
                descent = resample_by_arc_length(
                    (points[-1], self._mission_position),
                    self._path_sample_spacing,
                )
                points = points + descent[1:]
            if not densely_collision_free(
                points,
                self._map,
                self._collision_check_step,
            ):
                self._fail("POST_PROCESSING_COLLISION")
                return
        except (ValueError, RuntimeError) as error:
            self._fail(f"PATH_PROCESSING_ERROR:{error}")
            return

        self._path = points
        self._progress_index = 0
        self._target_index = 0
        self._metrics.update({
            "simplified_waypoints": len(simplified),
            "path_points": len(points),
            "smoothing_used": processed.smoothing_used,
            "smoothing_fallback": processed.smoothing_fallback,
            "smoothed_points": processed.smoothed_points,
            "smoothed_length": processed.smoothed_length,
            "simplified_length": path_length(simplified),
            "smoothing_time_sec": smoothing_time,
            "path_length": path_length(points),
            "minimum_clearance": self._map.minimum_clearance(points),
        })
        self._metrics["planned_path_length"] = self._metrics["path_length"]
        self._metrics["planned_path_clearance"] = self._metrics[
            "minimum_clearance"
        ]
        self._publish_path(points)
        self._set_state("EXECUTING", "path_ready")
        self._follow_path()
        self.get_logger().info(
            "Plan ready: raw=%d simplified=%d smoothed=%d "
            "execution=%d expanded=%d fallback=%s"
            % (
                result.raw_path_points,
                len(simplified),
                processed.smoothed_points,
                len(points),
                result.expanded_nodes,
                processed.smoothing_fallback,
            )
        )

    def _follow_path(self) -> None:
        if self._latest_position is None or not self._path:
            return
        search_end = min(len(self._path), self._progress_index + 20)
        nearest = min(
            range(self._progress_index, search_end),
            key=lambda index: math.dist(
                self._latest_position, self._path[index]
            ),
        )
        self._progress_index = max(self._progress_index, nearest)
        while (
            self._progress_index < len(self._path) - 1
            and math.dist(
                self._latest_position,
                self._path[self._progress_index],
            ) <= self._waypoint_tolerance
        ):
            self._progress_index += 1

        target = self._progress_index
        lookahead = 0.0
        while target < len(self._path) - 1:
            lookahead += math.dist(
                self._path[target], self._path[target + 1]
            )
            target += 1
            if lookahead >= self._lookahead_distance:
                break
        if target != self._target_index:
            self._target_index = target
            self._publish_current_waypoint()
            self._publish_control_goal()
        if (
            self._target_index == len(self._path) - 1
            and self._state == "EXECUTING"
        ):
            self._set_state("FINAL_APPROACH", "final_target_selected")
        self._publish_progress()
        if self._target_index == len(self._path) - 1:
            self._check_completion()

    def _check_completion(self) -> None:
        if self._latest_position is None or self._mission_position is None:
            return
        position_error = math.dist(
            self._latest_position, self._mission_position
        )
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
        self._metrics["mission_completion_time"] = (
            now - self._mission_started
            if self._mission_started is not None
            else 0.0
        )
        self._metrics["mission_completion_time_sec"] = self._metrics[
            "mission_completion_time"
        ]
        self._metrics["final_position_error"] = position_error
        self._metrics["final_yaw_error_deg"] = math.degrees(yaw_error)
        self._set_state("COMPLETED", "goal_stable")
        self.get_logger().info("Mission completed")

    def _update_actual_metrics(self) -> None:
        if (
            self._latest_position is None
            or self._previous_actual_position is None
            or self._mission_started is None
            or self._state in {"IDLE", "COMPLETED", "FAILED", "CANCELLED"}
        ):
            return
        previous = self._previous_actual_position
        current = self._latest_position
        self._metrics["actual_path_length"] = float(
            self._metrics.get("actual_path_length", 0.0)
        ) + math.dist(previous, current)
        colliding = any(
            obstacle.intersects_segment(previous, current)
            for obstacle in self._map.obstacles
        )
        if colliding and not self._actual_in_collision:
            self._metrics["collision_count"] = int(
                self._metrics.get("collision_count", 0)
            ) + 1
        self._actual_in_collision = colliding
        self._previous_actual_position = current
        self._publish_metrics()

    def _record_planning_metrics(self, result: PlanningResult) -> None:
        self._metrics.update({
            "planning_time": result.planning_time_sec,
            "planning_time_sec": result.planning_time_sec,
            "expanded_nodes": result.expanded_nodes,
            "raw_path_points": result.raw_path_points,
            "raw_path_length": result.raw_path_length,
        })

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
            quaternion = _yaw_quaternion(self._path_yaw(index))
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            message.poses.append(pose)
        self._path_publisher.publish(message)

    def _path_yaw(self, index: int) -> float:
        if index >= len(self._path) - 1:
            return self._mission_yaw
        current = self._path[index]
        following = self._path[index + 1]
        return math.atan2(
            following[1] - current[1],
            following[0] - current[0],
        )

    def _publish_control_goal(self) -> None:
        point = self._path[self._target_index]
        quaternion = (
            self._mission_quaternion
            if self._target_index == len(self._path) - 1
            else _yaw_quaternion(self._path_yaw(self._target_index))
        )
        self._publish_pose_goal(point, quaternion)

    def _publish_pose_goal(
        self,
        point: Point3,
        quaternion: Sequence[float],
    ) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = point[0]
        message.pose.position.y = point[1]
        message.pose.position.z = point[2]
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        self._goal_publisher.publish(message)

    def _publish_hover_goal(self) -> None:
        if self._latest_position is not None:
            self._publish_pose_goal(
                self._latest_position,
                _yaw_quaternion(self._latest_yaw),
            )

    def _publish_current_waypoint(self) -> None:
        if not self._path:
            return
        marker = self._point_marker(
            self._path[self._target_index],
            "current_waypoint",
            (1.0, 0.65, 0.05, 0.95),
            0.24,
        )
        self._waypoint_publisher.publish(marker)

    def _publish_mission_marker(self, point: Point3) -> None:
        marker = self._point_marker(
            point,
            "mission_goal",
            (0.15, 0.95, 0.25, 0.95),
            0.32,
        )
        self._mission_marker_publisher.publish(marker)

    def _point_marker(
        self,
        point: Point3,
        namespace: str,
        colour: Tuple[float, float, float, float],
        scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
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

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._publish_status(state, reason)
        self._publish_metrics()

    def _fail(self, reason: str) -> None:
        self._path = ()
        self._stable_since = None
        self._set_state("FAILED", reason)
        self.get_logger().error(f"Planning mission failed: {reason}")

    def _publish_status(self, state: str, reason: str) -> None:
        payload: Dict[str, object] = {
            "state": state,
            "reason": reason,
            "waypoint_index": self._target_index,
            "waypoint_count": len(self._path),
            "minimum_flight_altitude": self._minimum_flight_altitude,
            "takeoff_altitude": self._takeoff_altitude,
        }
        payload.update(self._metrics)
        message = String()
        message.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
        )
        self._status_publisher.publish(message)

    def _publish_progress(self) -> None:
        count = len(self._path)
        progress = (
            self._progress_index / max(1, count - 1)
            if count
            else 0.0
        )
        message = String()
        message.data = json.dumps({
            "state": self._state,
            "path_index": self._progress_index,
            "path_count": count,
            "progress": min(1.0, progress),
        }, sort_keys=True, separators=(",", ":"))
        self._progress_publisher.publish(message)

    def _publish_metrics(self) -> None:
        for name, publisher in self._float_metric_publishers.items():
            value = self._metrics.get(name)
            if value is not None:
                message = Float64()
                message.data = float(value)
                publisher.publish(message)
        avoidance = Bool()
        avoidance.data = self._avoidance_active
        self._avoidance_publisher.publish(avoidance)
        clearance = self._metrics.get("minimum_clearance")
        if clearance is not None and math.isfinite(float(clearance)):
            clearance_message = Float32()
            clearance_message.data = float(clearance)
            self._clearance_publisher.publish(clearance_message)
        index_message = Int32()
        index_message.data = self._target_index
        self._waypoint_index_publisher.publish(index_message)
        planning_message = String()
        planning_message.data = json.dumps({
            "planning_time": self._metrics.get("planning_time", 0.0),
            "expanded_nodes": self._metrics.get("expanded_nodes", 0),
            "raw_points": self._metrics.get("raw_path_points", 0),
            "simplified_points": self._metrics.get(
                "simplified_waypoints", 0
            ),
            "smoothed_points": self._metrics.get("smoothed_points", 0),
            "execution_points": self._metrics.get("path_points", 0),
            "planned_path_length": self._metrics.get("path_length", 0.0),
            "avoidance_active": self._avoidance_active,
            "smoothing_fallback": self._metrics.get(
                "smoothing_fallback", False
            ),
        }, sort_keys=True, separators=(",", ":"))
        self._planning_metrics_publisher.publish(planning_message)
        for name, publisher in self._int_metric_publishers.items():
            value = self._metrics.get(name)
            if value is not None:
                message = Int32()
                message.data = int(value)
                publisher.publish(message)


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
