"""Launch the full planned simulator and optional PyQt5 ground station."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the one-command showcase launch description."""
    bringup_share = Path(
        get_package_share_directory("drone_bringup")
    )
    map_share = Path(get_package_share_directory("drone_map"))
    planned = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "planned_sim.launch.py")
        ),
        launch_arguments={
            "map_file": LaunchConfiguration("map_file"),
            "rviz": LaunchConfiguration("rviz"),
            "publish_point_cloud": LaunchConfiguration(
                "publish_point_cloud"
            ),
        }.items(),
    )
    ground_station = Node(
        package="drone_ground_station",
        executable="ground_station",
        name="ground_station",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value=str(
                map_share / "config" / "static_map.yaml"
            ),
            description="Static map YAML for the showcase.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch the planning RViz view.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch the PyQt5 ground station.",
        ),
        DeclareLaunchArgument(
            "publish_point_cloud",
            default_value="false",
            description="Publish the optional obstacle PointCloud2.",
        ),
        planned,
        ground_station,
    ])
