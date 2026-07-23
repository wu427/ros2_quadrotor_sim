import math
import time
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
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
        self.required_stable_time = float(
            self.declare_parameter(
                "required_stable_time",
                1.0,
            ).value
        )
        self.timeout_sec = float(
            self.declare_parameter(
                "timeout_sec",
                15.0,
            ).value
        )

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

        self.start_time = time.monotonic()
        self.stable_since: Optional[float] = None
        self.latest_odom: Optional[Odometry] = None
        self.finished = False
        self.exit_code = 2
        self.log_counter = 0
        self.goal_sent = False
        self.shutdown_timer = None

        self.get_logger().info(
            "Acceptance test target: "
            f"({self.target_x:.2f}, "
            f"{self.target_y:.2f}, "
            f"{self.target_z:.2f}), "
            f"yaw={self.target_yaw_deg:.1f} deg"
        )

    def odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def timer_callback(self) -> None:
        if self.finished:
            return

        elapsed = time.monotonic() - self.start_time

        if not self.goal_sent:
            if (
                self.goal_publisher.get_subscription_count()
                > 0
            ):
                self.publish_goal()
                self.goal_sent = True

                self.get_logger().info(
                    "Goal published to controller"
                )
            elif elapsed >= self.timeout_sec:
                self.finish_failure(
                    "No controller subscription "
                    "before timeout"
                )

            return

        if self.latest_odom is None:
            if elapsed >= self.timeout_sec:
                self.finish_failure(
                    "No odometry received before timeout"
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

        now = time.monotonic()

        if stable:
            if self.stable_since is None:
                self.stable_since = now

            stable_duration = now - self.stable_since

            if stable_duration >= self.required_stable_time:
                self.finish_success(metrics, elapsed)
                return
        else:
            self.stable_since = None

        if elapsed >= self.timeout_sec:
            self.finish_failure(
                "Tracking did not converge before timeout",
                metrics,
            )

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
            "========================================"
        )
        self.finish(1)

    def finish(self, exit_code: int) -> None:
        self.finished = True
        self.exit_code = exit_code

        self.shutdown_timer = self.create_timer(
            0.1,
            self.request_shutdown,
        )

    @staticmethod
    def request_shutdown() -> None:
        if rclpy.ok():
            rclpy.shutdown()

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
    node = AcceptanceTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
