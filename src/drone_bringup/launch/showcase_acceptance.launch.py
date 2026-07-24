"""Launch one automated showcase acceptance scenario."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    """Build the showcase acceptance launch graph."""
    share = Path(get_package_share_directory("drone_bringup"))
    map_share = Path(get_package_share_directory("drone_map"))
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / "launch" / "showcase.launch.py")
        ),
        launch_arguments={
            "map_file": LaunchConfiguration("map_file"),
            "rviz": "false",
            "gui": "false",
            "publish_point_cloud": "false",
        }.items(),
    )
    acceptance = Node(
        package="drone_bringup",
        executable="showcase_acceptance_node",
        name="showcase_acceptance_node",
        output="screen",
        parameters=[{
            "scenario": LaunchConfiguration("scenario"),
            "map_file": LaunchConfiguration("map_file"),
            "target_x": _float("target_x"),
            "target_y": _float("target_y"),
            "target_z": _float("target_z"),
            "timeout_sec": _float("timeout_sec"),
            "position_tolerance": _float("position_tolerance"),
            "minimum_flight_z": _float("minimum_flight_z"),
        }],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=acceptance,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Showcase acceptance completed"
                    )
                )
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="default",
            description="Showcase acceptance scenario name.",
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value=str(map_share / "config" / "static_map.yaml"),
            description="Scenario static map.",
        ),
        DeclareLaunchArgument("target_x", default_value="5.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_z", default_value="1.5"),
        DeclareLaunchArgument("timeout_sec", default_value="150.0"),
        DeclareLaunchArgument(
            "position_tolerance", default_value="0.15"
        ),
        DeclareLaunchArgument("minimum_flight_z", default_value="1.0"),
        simulator,
        acceptance,
        shutdown,
    ])
