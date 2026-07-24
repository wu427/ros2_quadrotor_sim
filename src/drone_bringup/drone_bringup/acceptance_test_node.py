import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AcceptanceTestNode(Node):
    """Send a target pose and automatically evaluate tracking accuracy."""

    def __init__(self) -> None:
        super().__init__("acceptance_test_node")

        self.target_x = float(
            self.declare_parameter("target_x", 2.0).value
        )
        self.target_y = float(
            self.declare_parameter("target_y", 1.0).value
        )
        self.target_z = float(
            self.declare_parameter("target_z", 1.5).value
        )
        self.target_yaw_deg = float(
            self.declare_parameter(
                "target_yaw_deg",
                90.0,
            ).value
        )

        self.position_tolerance = float(
            self.declare_parameter(
                "position_tolerance",
                0.05,
            ).value
        )
        self.yaw_tolerance_deg = float(
            self.declare_parameter(
                "yaw_tolerance_deg",
                3.0,
            ).value
        )
        self.linear_speed_tolerance = float(
            self.declare_parameter(
                "linear_speed_tolerance",
                0.05,
            ).value
        )
        self.angular_speed_tolerance = float(
            self.declare_parameter(
                "angular_speed_tolerance",
                0.05,
            ).value
        )
        required_stable_time = float(
            self.declare_parameter(
                "required_stable_time",
                1.0,
            ).value
        )
        self.stable_duration_sec = float(
            self.declare_parameter(
                "stable_duration_sec",
                required_stable_time,
            ).value
        )
        self.discovery_timeout_sec = float(
            self.declare_parameter(
                "discovery_timeout_sec",
                15.0,
            ).value
        )
        self.timeout_sec = float(
            self.declare_parameter(
                "timeout_sec",
                20.0,
            ).value
        )

        self.validate_parameters()

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/drone/goal",
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
        )

        self.discovery_start_time = time.monotonic()
        self.discovery_elapsed_time = 0.0
        self.goal_publish_time: Optional[float] = None
        self.stable_since: Optional[float] = None
        self.latest_odom: Optional[Odometry] = None
        self.finished = False
        self.exit_code = 2
        self.log_counter = 0
        self.goal_sent = False
        self.invalid_odom_warned = False

        self.get_logger().info(
            "Acceptance test target: "
            f"({self.target_x:.2f}, "
            f"{self.target_y:.2f}, "
            f"{self.target_z:.2f}), "
            f"yaw={self.target_yaw_deg:.1f} deg"
        )

    def validate_parameters(self) -> None:
        positive_parameters = {
            "stable_duration_sec": self.stable_duration_sec,
            "discovery_timeout_sec": self.discovery_timeout_sec,
            "timeout_sec": self.timeout_sec,
        }
        nonnegative_parameters = {
            "position_tolerance": self.position_tolerance,
            "yaw_tolerance_deg": self.yaw_tolerance_deg,
            "linear_speed_tolerance": (
                self.linear_speed_tolerance
            ),
            "angular_speed_tolerance": (
                self.angular_speed_tolerance
            ),
        }

        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
                )

        for name, value in nonnegative_parameters.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and nonnegative"
                )

    def odom_callback(self, message: Odometry) -> None:
        if not self.is_valid_odometry(message):
            self.latest_odom = None
            self.stable_since = None

            if not self.invalid_odom_warned:
                self.get_logger().warning(
                    "Ignoring odometry with non-finite state values"
                )
                self.invalid_odom_warned = True

            return

        self.latest_odom = message

    def timer_callback(self) -> None:
        if self.finished:
            return

        now = time.monotonic()

        if not self.goal_sent:
            self.discovery_elapsed_time = (
                now - self.discovery_start_time
            )
            controller_ready = (
                self.goal_publisher.get_subscription_count()
                > 0
            )
            odometry_ready = self.latest_odom is not None

            if controller_ready and odometry_ready:
                self.publish_goal()
                self.goal_sent = True
                self.goal_publish_time = time.monotonic()

                self.get_logger().info(
                    "Discovery completed"
                )
                self.get_logger().info(
                    "Discovery elapsed time: "
                    f"{self.discovery_elapsed_time:.2f} s"
                )
                self.get_logger().info("Goal published")
                self.get_logger().info(
                    "Convergence timer started"
                )
            elif (
                self.discovery_elapsed_time
                >= self.discovery_timeout_sec
            ):
                self.finish_failure(
                    self.discovery_failure_reason(
                        controller_ready,
                        odometry_ready,
                    )
                )

            return

        convergence_elapsed = self.convergence_elapsed(now)

        if self.latest_odom is None:
            self.stable_since = None

            if convergence_elapsed >= self.timeout_sec:
                self.finish_failure(
                    "Convergence timeout: "
                    "no valid odometry available",
                )
            return

        metrics = self.calculate_metrics(self.latest_odom)

        self.log_counter += 1
        if self.log_counter % 10 == 0:
            self.get_logger().info(
                "position_error="
                f"{metrics['position_error']:.4f} m, "
                "yaw_error="
                f"{metrics['yaw_error_deg']:.2f} deg, "
                "linear_speed="
                f"{metrics['linear_speed']:.4f} m/s, "
                "angular_speed="
                f"{metrics['angular_speed']:.4f} rad/s"
            )

        stable = (
            metrics["position_error"]
            <= self.position_tolerance
            and metrics["yaw_error_deg"]
            <= self.yaw_tolerance_deg
            and metrics["linear_speed"]
            <= self.linear_speed_tolerance
            and metrics["angular_speed"]
            <= self.angular_speed_tolerance
        )

        if stable:
            if self.stable_since is None:
                self.stable_since = now

            stable_duration = now - self.stable_since

            if (
                stable_duration
                >= self.stable_duration_sec
            ):
                self.finish_success(
                    metrics,
                    convergence_elapsed,
                )
                return
        else:
            self.stable_since = None

        if convergence_elapsed >= self.timeout_sec:
            self.finish_failure(
                "Convergence timeout: "
                "tracking did not meet all thresholds",
                metrics,
            )

    def discovery_failure_reason(
        self,
        controller_ready: bool,
        odometry_ready: bool,
    ) -> str:
        if not controller_ready and not odometry_ready:
            return (
                "Discovery timeout: missing controller "
                "subscription and valid odometry"
            )

        if not controller_ready:
            return (
                "Discovery timeout: missing controller "
                "subscription"
            )

        return "Discovery timeout: missing valid odometry"

    def convergence_elapsed(
        self,
        now: Optional[float] = None,
    ) -> float:
        if self.goal_publish_time is None:
            return 0.0

        if now is None:
            now = time.monotonic()

        return now - self.goal_publish_time

    @staticmethod
    def is_valid_odometry(odom: Odometry) -> bool:
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        linear = odom.twist.twist.linear
        angular = odom.twist.twist.angular

        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        )
        return all(math.isfinite(value) for value in values)

    def publish_goal(self) -> None:
        yaw_rad = math.radians(self.target_yaw_deg)

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"

        message.pose.position.x = self.target_x
        message.pose.position.y = self.target_y
        message.pose.position.z = self.target_z

        message.pose.orientation.z = math.sin(
            yaw_rad / 2.0
        )
        message.pose.orientation.w = math.cos(
            yaw_rad / 2.0
        )

        self.goal_publisher.publish(message)

    def calculate_metrics(
        self,
        odom: Odometry,
    ) -> dict[str, float]:
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        linear = odom.twist.twist.linear
        angular = odom.twist.twist.angular

        dx = position.x - self.target_x
        dy = position.y - self.target_y
        dz = position.z - self.target_z

        position_error = math.sqrt(
            dx * dx + dy * dy + dz * dz
        )

        current_yaw = self.yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        target_yaw = math.radians(
            self.target_yaw_deg
        )

        yaw_error = self.wrap_angle(
            current_yaw - target_yaw
        )

        linear_speed = math.sqrt(
            linear.x * linear.x
            + linear.y * linear.y
            + linear.z * linear.z
        )

        angular_speed = math.sqrt(
            angular.x * angular.x
            + angular.y * angular.y
            + angular.z * angular.z
        )

        return {
            "position_error": position_error,
            "yaw_error_deg": abs(
                math.degrees(yaw_error)
            ),
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "x": position.x,
            "y": position.y,
            "z": position.z,
        }

    def finish_success(
        self,
        metrics: dict[str, float],
        elapsed: float,
    ) -> None:
        self.get_logger().info(
            "\n"
            "========================================\n"
            "       ACCEPTANCE TEST: PASS\n"
            "========================================\n"
            f"Final position: "
            f"({metrics['x']:.6f}, "
            f"{metrics['y']:.6f}, "
            f"{metrics['z']:.6f})\n"
            f"Position error: "
            f"{metrics['position_error']:.6f} m\n"
            f"Yaw error: "
            f"{metrics['yaw_error_deg']:.4f} deg\n"
            f"Linear speed: "
            f"{metrics['linear_speed']:.6f} m/s\n"
            f"Angular speed: "
            f"{metrics['angular_speed']:.6f} rad/s\n"
            f"Discovery time: "
            f"{self.discovery_elapsed_time:.2f} s\n"
            f"Convergence time: {elapsed:.2f} s\n"
            "========================================"
        )
        self.finish(0)

    def finish_failure(
        self,
        reason: str,
        metrics: Optional[dict[str, float]] = None,
    ) -> None:
        detail = ""

        if metrics is not None:
            detail = (
                f"\nPosition error: "
                f"{metrics['position_error']:.6f} m"
                f"\nYaw error: "
                f"{metrics['yaw_error_deg']:.4f} deg"
                f"\nLinear speed: "
                f"{metrics['linear_speed']:.6f} m/s"
                f"\nAngular speed: "
                f"{metrics['angular_speed']:.6f} rad/s"
            )

        self.get_logger().error(
            "\n"
            "========================================\n"
            "       ACCEPTANCE TEST: FAIL\n"
            "========================================\n"
            f"Reason: {reason}"
            f"{detail}\n"
            f"Discovery time: "
            f"{self.discovery_elapsed_time:.2f} s\n"
            f"Convergence time: "
            f"{self.convergence_elapsed():.2f} s\n"
            "========================================"
        )
        self.finish(1)

    def finish(self, exit_code: int) -> None:
        if self.finished:
            return

        self.finished = True
        self.exit_code = exit_code

        self.timer.cancel()

    @staticmethod
    def yaw_from_quaternion(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> float:
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[AcceptanceTestNode] = None
    exit_code = 2

    try:
        node = AcceptanceTestNode()

        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

        exit_code = node.exit_code
    except (
        KeyboardInterrupt,
        ExternalShutdownException,
    ):
        exit_code = 130
    except Exception:
        if rclpy.ok():
            raise

        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
