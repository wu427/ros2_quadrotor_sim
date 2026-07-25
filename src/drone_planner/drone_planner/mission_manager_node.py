"""Multi-waypoint and repeatable patrol mission sequencing."""

from __future__ import annotations

import json
import math
import time
from typing import List

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Float32
from std_msgs.msg import String

from drone_planner.patrol import PatrolConfig
from drone_planner.patrol import completed_fraction
from drone_planner.patrol import patrol_cycle_indices
from drone_planner.patrol import should_continue_after_cycle


class MissionManagerNode(Node):
    """Dispatch waypoint missions and optional repeatable patrol patterns."""

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
        self._patrol_status_publisher = self.create_publisher(
            String, "/drone/patrol_status", durable_qos
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
        self._config_subscription = self.create_subscription(
            String,
            "/drone/patrol_config",
            self._config_callback,
            10,
        )
        self._planner_subscription = self.create_subscription(
            String,
            "/drone/planner_status",
            self._planner_callback,
            durable_qos,
        )
        self._timer = self.create_timer(0.1, self._timer_callback)

        self._waypoints: List[PoseStamped] = []
        self._config = PatrolConfig()
        self._cycle: tuple[int, ...] = ()
        self._cycle_position = 0
        self._lap = 0
        self._completed_segments = 0
        self._state = "IDLE"
        self._state_before_pause = "IDLE"
        self._planner_state = ""
        self._reason = "waiting_for_path"
        self._started_at: float | None = None
        self._dwell_deadline: float | None = None
        self._remaining_dwell = 0.0
        self._pending_start = False
        self._planner_started_for_goal = False
        self._publish_status(self._reason)

    def _path_callback(self, message: Path) -> None:
        if message.header.frame_id not in {"", "map"}:
            self._set_state("FAILED", "INVALID_PATH_FRAME")
            return
        waypoints: List[PoseStamped] = []
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
        self._cycle = ()
        self._cycle_position = 0
        self._lap = 0
        self._completed_segments = 0
        self._set_state("READY", "path_loaded")
        if self._pending_start:
            self._pending_start = False
            self._start()

    def _config_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._config = PatrolConfig.from_mapping(payload)
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self._set_state("FAILED", f"INVALID_PATROL_CONFIG:{error}")
            return
        if self._state in {"IDLE", "READY", "COMPLETED", "STOPPED"}:
            self._publish_status("patrol_config_updated")

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().upper()
        if command == "START":
            if not self._waypoints:
                self._pending_start = True
                self._set_state("WAITING_FOR_PATH", "start_waiting_for_path")
            else:
                self._start()
        elif command == "PAUSE" and self._state in {
            "RUNNING",
            "DWELL",
            "RETURNING",
        }:
            self._state_before_pause = self._state
            if self._state == "DWELL" and self._dwell_deadline is not None:
                self._remaining_dwell = max(
                    0.0, self._dwell_deadline - time.monotonic()
                )
            self._set_state("PAUSED", "operator_pause")
        elif command == "RESUME" and self._state == "PAUSED":
            restored = self._state_before_pause
            if restored == "DWELL":
                self._dwell_deadline = (
                    time.monotonic() + self._remaining_dwell
                )
            self._set_state(restored, "operator_resume")
        elif command in {"CANCEL", "STOP"} and self._state not in {
            "IDLE",
            "COMPLETED",
            "STOPPED",
            "CANCELLED",
        }:
            self._dwell_deadline = None
            if command == "CANCEL":
                self._set_state("CANCELLED", "operator_cancel")
            else:
                self._set_state("STOPPED", "operator_stop")
        elif command == "SKIP" and self._state in {
            "RUNNING",
            "DWELL",
            "PAUSED",
        }:
            self._dwell_deadline = None
            self._advance("operator_skip")
        elif command == "RETURN_HOME":
            self._return_home("operator_return_home")
        elif command == "CLEAR":
            self._waypoints = []
            self._cycle = ()
            self._cycle_position = 0
            self._lap = 0
            self._completed_segments = 0
            self._started_at = None
            self._dwell_deadline = None
            self._pending_start = False
            self._planner_started_for_goal = False
            self._set_state("IDLE", "operator_clear")

    def _start(self) -> None:
        if not self._waypoints:
            self._set_state("FAILED", "NO_PATH_LOADED")
            return
        if self._config.mode != "ONCE" and len(self._waypoints) < 2:
            self._set_state("FAILED", "PATROL_REQUIRES_TWO_WAYPOINTS")
            return
        try:
            self._cycle = (
                (0,)
                if len(self._waypoints) == 1
                else patrol_cycle_indices(
                    len(self._waypoints), self._config.mode
                )
            )
        except ValueError as error:
            self._set_state("FAILED", f"INVALID_PATROL:{error}")
            return
        self._cycle_position = 0
        self._lap = 1
        self._completed_segments = 0
        self._started_at = time.monotonic()
        self._dwell_deadline = None
        self._set_state("RUNNING", "mission_started")
        self._publish_current_goal()

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
        if self._planner_state in {
            "TAKEOFF",
            "PLANNING",
            "EXECUTING",
            "FINAL_APPROACH",
            "PAUSED",
        }:
            self._planner_started_for_goal = True
        if (
            self._planner_state == "COMPLETED"
            and not self._planner_started_for_goal
            and self._state in {"RUNNING", "RETURNING"}
        ):
            return
        if self._state == "RETURNING":
            if self._planner_state == "FAILED":
                self._set_state(
                    "FAILED", f"RETURN_HOME:{payload.get('reason', 'UNKNOWN')}"
                )
            elif self._planner_state == "COMPLETED":
                self._set_state("COMPLETED", "returned_home")
            return
        if self._state != "RUNNING":
            return
        if self._planner_state == "FAILED":
            self._set_state(
                "FAILED",
                f"PLANNER:{payload.get('reason', 'UNKNOWN')}",
            )
        elif self._planner_state == "COMPLETED":
            if self._config.dwell_sec > 0.0:
                self._dwell_deadline = (
                    time.monotonic() + self._config.dwell_sec
                )
                self._set_state("DWELL", "waypoint_dwell")
            else:
                self._advance("waypoint_completed")

    def _timer_callback(self) -> None:
        if self._state == "DWELL" and self._dwell_deadline is not None:
            if time.monotonic() >= self._dwell_deadline:
                self._dwell_deadline = None
                self._advance("dwell_completed")

    def _advance(self, reason: str) -> None:
        if not self._cycle:
            self._set_state("FAILED", "PATROL_SEQUENCE_MISSING")
            return
        self._completed_segments += 1
        if (
            self._config.mode == "TIMED"
            and self._elapsed() >= self._config.duration_sec
        ):
            self._finish("timed_patrol_completed")
            return
        self._cycle_position += 1
        if self._cycle_position < len(self._cycle):
            self._set_state("RUNNING", reason)
            self._publish_current_goal()
            return

        completed_laps = self._lap
        elapsed = self._elapsed()
        if should_continue_after_cycle(
            self._config, completed_laps, elapsed
        ):
            self._lap += 1
            self._cycle_position = (
                1 if self._config.mode == "PING_PONG" else 0
            )
            self._set_state("RUNNING", "next_patrol_lap")
            self._publish_current_goal()
            return
        self._finish("all_waypoints_completed")

    def _finish(self, reason: str) -> None:
        self._dwell_deadline = None
        if self._config.final_action == "RETURN_HOME":
            self._return_home(reason)
        else:
            self._set_state("COMPLETED", reason)

    def _return_home(self, reason: str) -> None:
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = self._config.home[0]
        target.pose.position.y = self._config.home[1]
        target.pose.position.z = self._config.home[2]
        yaw = self._config.home[3]
        target.pose.orientation.z = math.sin(0.5 * yaw)
        target.pose.orientation.w = math.cos(0.5 * yaw)
        target.header.stamp = self.get_clock().now().to_msg()
        self._planner_started_for_goal = False
        self._planner_state = ""
        self._set_state("RETURNING", reason)
        self._goal_publisher.publish(target)

    def _publish_current_goal(self) -> None:
        waypoint_index = self._cycle[self._cycle_position]
        message = self._waypoints[waypoint_index]
        message.header.stamp = self.get_clock().now().to_msg()
        self._planner_started_for_goal = False
        self._planner_state = ""
        self._goal_publisher.publish(message)
        self._publish_status("goal_dispatched")

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._reason = reason
        self._publish_status(reason)

    def _elapsed(self) -> float:
        return (
            max(0.0, time.monotonic() - self._started_at)
            if self._started_at is not None
            else 0.0
        )

    def _progress(self) -> float:
        if self._state == "COMPLETED":
            return 1.0
        return completed_fraction(
            self._completed_segments,
            len(self._cycle),
            self._lap,
            self._config,
        )

    def _publish_status(self, reason: str) -> None:
        current_index = (
            self._cycle[self._cycle_position]
            if self._cycle and self._cycle_position < len(self._cycle)
            else 0
        )
        payload = {
            "state": self._state,
            "reason": reason,
            "waypoint_index": current_index,
            "waypoint_count": len(self._waypoints),
            "cycle_position": self._cycle_position,
            "cycle_count": len(self._cycle),
            "lap": self._lap,
            "completed_segments": self._completed_segments,
            "progress": self._progress(),
            "planner_state": self._planner_state,
            "patrol_mode": self._config.mode,
            "patrol_laps": self._config.laps,
            "duration_sec": self._config.duration_sec,
            "dwell_sec": self._config.dwell_sec,
            "final_action": self._config.final_action,
            "elapsed_sec": self._elapsed(),
        }
        message = String()
        message.data = json.dumps(
            payload, sort_keys=True, separators=(",", ":")
        )
        self._status_publisher.publish(message)
        self._patrol_status_publisher.publish(message)
        progress = Float32()
        progress.data = float(payload["progress"])
        self._progress_publisher.publish(progress)


def main(args=None) -> None:
    """Run the waypoint and patrol mission manager."""
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
