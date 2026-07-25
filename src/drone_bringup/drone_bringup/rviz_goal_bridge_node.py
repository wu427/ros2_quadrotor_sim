"""Bridge standard RViz goal tools to the planner's safe 3D mission topic."""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class RvizGoalBridgeNode(Node):
    """Convert RViz 2D Goal and Publish Point messages into 3D goals."""

    def __init__(self) -> None:
        super().__init__("rviz_goal_bridge_node")
        self._target_altitude = float(
            self.declare_parameter("target_altitude", 1.5).value
        )
        self._minimum_altitude = float(
            self.declare_parameter("minimum_altitude", 1.0).value
        )
        self._maximum_altitude = float(
            self.declare_parameter("maximum_altitude", 3.5).value
        )
        self._validate_altitude(self._target_altitude)

        self._goal_publisher = self.create_publisher(
            PoseStamped, "/drone/mission_goal", 10
        )
        self.create_subscription(
            PoseStamped,
            "/drone/rviz_goal_2d",
            self._on_goal_2d,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/drone/rviz_goal_point",
            self._on_goal_point,
            10,
        )
        self.create_subscription(
            Float32,
            "/drone/rviz_target_altitude",
            self._on_altitude,
            10,
        )
        self.get_logger().info(
            "RViz goal bridge ready: 2D Goal and Publish Point use z=%.2f m"
            % self._target_altitude
        )

    def _validate_altitude(self, value: float) -> None:
        if (
            not math.isfinite(value)
            or value < self._minimum_altitude
            or value > self._maximum_altitude
        ):
            raise ValueError(
                "target altitude must be within "
                f"[{self._minimum_altitude:.2f}, "
                f"{self._maximum_altitude:.2f}]"
            )

    def _on_altitude(self, message: Float32) -> None:
        altitude = float(message.data)
        try:
            self._validate_altitude(altitude)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._target_altitude = altitude
        self.get_logger().info(
            "RViz target altitude updated to %.2f m" % altitude
        )

    def _on_goal_2d(self, message: PoseStamped) -> None:
        target = PoseStamped()
        target.header.frame_id = message.header.frame_id or "map"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = message.pose
        target.pose.position.z = self._target_altitude
        self._goal_publisher.publish(target)
        self.get_logger().info(
            "RViz 2D goal forwarded: [%.2f, %.2f, %.2f]"
            % (
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            )
        )

    def _on_goal_point(self, message: PointStamped) -> None:
        target = PoseStamped()
        target.header.frame_id = message.header.frame_id or "map"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = message.point.x
        target.pose.position.y = message.point.y
        clicked_z = float(message.point.z)
        target.pose.position.z = (
            clicked_z
            if self._minimum_altitude <= clicked_z <= self._maximum_altitude
            else self._target_altitude
        )
        target.pose.orientation.w = 1.0
        self._goal_publisher.publish(target)
        self.get_logger().info(
            "RViz point goal forwarded: [%.2f, %.2f, %.2f]"
            % (
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            )
        )


def main(args=None) -> None:
    """Run the RViz goal bridge node."""
    rclpy.init(args=args)
    node = RvizGoalBridgeNode()
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
