"""Launch the PyQt5 ground station as a ROS2 node process."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the standalone ground-station launch description."""
    return LaunchDescription([
        Node(
            package="drone_ground_station",
            executable="ground_station",
            name="ground_station",
            output="screen",
        )
    ])
