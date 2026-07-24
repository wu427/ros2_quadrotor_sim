"""ROS2 publishers for raw, inflated, and sampled static-map geometry."""

from pathlib import Path
import struct

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from drone_map.collision_geometry import AABB
from drone_map.collision_geometry import StaticMap
from drone_map.collision_geometry import load_static_map
from drone_map.surface_sampling import sample_box_surfaces


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
        publish_point_cloud = bool(
            self.declare_parameter("publish_point_cloud", False).value
        )
        point_spacing = float(
            self.declare_parameter("point_cloud_spacing", 0.20).value
        )
        if point_spacing <= 0.0:
            raise ValueError("point_cloud_spacing must be positive")
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
        self._inflated_publisher = self.create_publisher(
            MarkerArray, "/map/inflated_obstacles", qos
        )
        self._cloud_publisher = self.create_publisher(
            PointCloud2, "/map/obstacle_points", qos
        )
        self._markers = self._build_raw_markers(self._static_map)
        self._inflated_markers = self._build_obstacle_markers(
            self._static_map.inflated_obstacles,
            self._static_map.frame_id,
            "inflated_obstacles",
            (0.12, 0.45, 0.95, 0.22),
        )
        self._cloud = (
            self._build_cloud(point_spacing)
            if publish_point_cloud
            else None
        )
        self._publisher.publish(self._markers)
        self._inflated_publisher.publish(self._inflated_markers)
        if self._cloud is not None:
            self._cloud_publisher.publish(self._cloud)
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
        self._inflated_publisher.publish(self._inflated_markers)
        if self._cloud is not None:
            self._cloud_publisher.publish(self._cloud)
        self._timer.cancel()

    def _build_obstacle_markers(
        self,
        obstacles: tuple[AABB, ...],
        frame_id: str,
        namespace: str,
        colour: tuple[float, float, float, float],
    ) -> MarkerArray:
        result = MarkerArray()
        for marker_id, obstacle in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
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
            marker.color.r = colour[0]
            marker.color.g = colour[1]
            marker.color.b = colour[2]
            marker.color.a = colour[3]
            marker.text = obstacle.obstacle_id
            result.markers.append(marker)
        return result

    def _build_raw_markers(self, static_map: StaticMap) -> MarkerArray:
        markers = self._build_obstacle_markers(
            static_map.obstacles,
            static_map.frame_id,
            "static_obstacles",
            (0.75, 0.18, 0.12, 0.82),
        )
        boundary = Marker()
        boundary.header.frame_id = static_map.frame_id
        boundary.header.stamp = self.get_clock().now().to_msg()
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

    def _build_cloud(self, spacing: float) -> PointCloud2:
        points = sample_box_surfaces(
            self._static_map.obstacles, spacing
        )
        message = PointCloud2()
        message.header.frame_id = self._static_map.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(
            struct.pack("<fff", *point) for point in points
        )
        return message


def main(args=None) -> None:
    """Run the static map node."""
    rclpy.init(args=args)
    node = None

    try:
        node = StaticMapNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal termination through Ctrl+C or launch shutdown.
        pass
    except RCLError as exc:
        # ROS 2 Humble may invalidate the context before spin() returns.
        message = str(exc)
        normal_shutdown = (
            "context is not valid" in message
            or "rcl_shutdown already called" in message
        )
        if not normal_shutdown:
            raise
    finally:
        if node is not None:
            node.destroy_node()

        # Avoid calling shutdown twice when launch already closed the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
