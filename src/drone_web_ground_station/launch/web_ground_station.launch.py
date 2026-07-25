"""Launch the single-page web ground station."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the web dashboard node with configurable address."""
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("open_browser", default_value="true"),
        DeclareLaunchArgument(
            "history_duration_sec", default_value="60.0"
        ),
        DeclareLaunchArgument(
            "max_history_points", default_value="10000"
        ),
        Node(
            package="drone_web_ground_station",
            executable="web_ground_station",
            name="web_ground_station",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": ParameterValue(
                    LaunchConfiguration("port"), value_type=int
                ),
                "open_browser": ParameterValue(
                    LaunchConfiguration("open_browser"), value_type=bool
                ),
                "history_duration_sec": ParameterValue(
                    LaunchConfiguration("history_duration_sec"),
                    value_type=float,
                ),
                "max_history_points": ParameterValue(
                    LaunchConfiguration("max_history_points"),
                    value_type=int,
                ),
            }],
        ),
    ])
