from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from visualization_msgs.msg import Marker


class GoalMarkerNode(Node):
    """Publish a persistent RViz marker for the current drone goal."""

    def __init__(self) -> None:
        super().__init__("goal_marker_node")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(
            Marker,
            "/drone/goal_marker",
            qos,
        )

        self._subscription = self.create_subscription(
            PoseStamped,
            "/drone/goal",
            self._goal_callback,
            10,
        )

        self.get_logger().info(
            "Goal marker node started"
        )

    def _goal_callback(self, message: PoseStamped) -> None:
        marker = Marker()

        marker.header = message.header
        if not marker.header.frame_id:
            marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "drone_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = message.pose
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
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

        self._publisher.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
