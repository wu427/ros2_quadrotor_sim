"""ROS2 publisher for the configured static three-dimensional map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from drone_map.collision_geometry import StaticMap
from drone_map.collision_geometry import load_static_map


class StaticMapNode(Node):
    """Load collision geometry once and publish durable RViz markers."""

    def __init__(self) -> None:
        super().__init__("static_map_node")
        default_path = (
            Path(get_package_share_directory("drone_map"))
            / "config"
            / "static_map.yaml"
        )
        map_file = self.declare_parameter(
            "map_file", str(default_path)
        ).get_parameter_value().string_value
        self._static_map = load_static_map(map_file)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            MarkerArray, "/map/obstacles", qos
        )
        self._markers = self._build_markers(self._static_map)
        self._publisher.publish(self._markers)
        self._timer = self.create_timer(0.5, self._publish_once)
        self.get_logger().info(
            "Static map loaded: %d obstacles, inflation %.3f m"
            % (
                len(self._static_map.obstacles),
                self._static_map.inflation_distance,
            )
        )

    def _publish_once(self) -> None:
        self._publisher.publish(self._markers)
        self._timer.cancel()

    def _build_markers(self, static_map: StaticMap) -> MarkerArray:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for marker_id, obstacle in enumerate(static_map.obstacles):
            marker = Marker()
            marker.header.frame_id = static_map.frame_id
            marker.header.stamp = stamp
            marker.ns = "static_obstacles"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle.center[0]
            marker.pose.position.y = obstacle.center[1]
            marker.pose.position.z = obstacle.center[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = obstacle.size[0]
            marker.scale.y = obstacle.size[1]
            marker.scale.z = obstacle.size[2]
            marker.color.r = 0.75
            marker.color.g = 0.18
            marker.color.b = 0.12
            marker.color.a = 0.82
            marker.text = obstacle.obstacle_id
            markers.markers.append(marker)

        boundary = Marker()
        boundary.header.frame_id = static_map.frame_id
        boundary.header.stamp = stamp
        boundary.ns = "map_bounds"
        boundary.id = len(static_map.obstacles)
        boundary.type = Marker.LINE_LIST
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 0.025
        boundary.color.r = 0.15
        boundary.color.g = 0.55
        boundary.color.b = 0.95
        boundary.color.a = 0.9
        minimum = static_map.minimum
        maximum = static_map.maximum
        corners = [
            (x, y, z)
            for x in (minimum[0], maximum[0])
            for y in (minimum[1], maximum[1])
            for z in (minimum[2], maximum[2])
        ]
        edges = (
            (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
            (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
        )
        for first, second in edges:
            for corner in (corners[first], corners[second]):
                point = Point()
                point.x, point.y, point.z = corner
                boundary.points.append(point)
        markers.markers.append(boundary)
        return markers


def main(args=None) -> None:
    """Run the static-map publisher."""
    rclpy.init(args=args)
    node = StaticMapNode()
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
