from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import rclpy
from visualization_msgs.msg import Marker


class GoalMarkerNode(Node):
    """Publish a persistent RViz marker for the current drone goal."""

    def __init__(self) -> None:
        super().__init__("goal_marker_node")

        self._publisher = self.create_publisher(
            Marker,
            "/drone/goal_marker",
            10,
        )

        self._subscription = self.create_subscription(
            PoseStamped,
            "/drone/goal",
            self._goal_callback,
            10,
        )

        self._latest_marker = None

        self._timer = self.create_timer(
            0.2,
            self._publish_latest_marker,
        )

        self.get_logger().info(
            "Goal marker node started"
        )

    def _goal_callback(self, message: PoseStamped) -> None:
        marker = Marker()

        marker.header.frame_id = (
            message.header.frame_id
            if message.header.frame_id
            else "map"
        )

        # A zero timestamp asks RViz to use the latest transform.
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        marker.ns = "drone_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = message.pose.position
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22

        marker.color.r = 0.15
        marker.color.g = 0.95
        marker.color.b = 0.20
        marker.color.a = 0.90

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self._latest_marker = marker
        self._publisher.publish(marker)

    def _publish_latest_marker(self) -> None:
        if self._latest_marker is not None:
            self._publisher.publish(
                self._latest_marker
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
