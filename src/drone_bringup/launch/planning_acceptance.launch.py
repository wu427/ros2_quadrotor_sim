"""Launch an end-to-end planning acceptance scenario."""

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


def _float_parameter(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    """Build the planning acceptance launch description."""
    bringup_share = Path(
        get_package_share_directory("drone_bringup")
    )
    map_share = Path(get_package_share_directory("drone_map"))
    map_file = LaunchConfiguration("map_file")

    planned_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(bringup_share / "launch" / "planned_sim.launch.py")
        ),
        launch_arguments={
            "map_file": map_file,
            "rviz": "false",
        }.items(),
    )
    test_node = Node(
        package="drone_bringup",
        executable="planning_acceptance_node",
        name="planning_acceptance_node",
        output="screen",
        parameters=[{
            "scenario": LaunchConfiguration("scenario"),
            "map_file": map_file,
            "target_x": _float_parameter("target_x"),
            "target_y": _float_parameter("target_y"),
            "target_z": _float_parameter("target_z"),
            "target_yaw_deg": _float_parameter("target_yaw_deg"),
            "discovery_timeout_sec": _float_parameter(
                "discovery_timeout_sec"
            ),
            "mission_timeout_sec": _float_parameter(
                "mission_timeout_sec"
            ),
            "position_tolerance": _float_parameter(
                "position_tolerance"
            ),
            "yaw_tolerance_deg": _float_parameter(
                "yaw_tolerance_deg"
            ),
        }],
    )
    shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=test_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="Planning acceptance completed"
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value="multi",
            description=(
                "Evidence scenario label: single, multi, "
                "invalid_goal, or no_path."
            ),
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value=str(map_share / "config" / "static_map.yaml"),
            description="Static-map YAML for this acceptance scenario.",
        ),
        DeclareLaunchArgument(
            "target_x",
            default_value="5.0",
            description="Mission target x in metres.",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="0.0",
            description="Mission target y in metres.",
        ),
        DeclareLaunchArgument(
            "target_z",
            default_value="1.5",
            description="Mission target z in metres.",
        ),
        DeclareLaunchArgument(
            "target_yaw_deg",
            default_value="20.0",
            description="Mission target yaw in degrees.",
        ),
        DeclareLaunchArgument(
            "discovery_timeout_sec",
            default_value="15.0",
            description="Planner and odometry discovery timeout.",
        ),
        DeclareLaunchArgument(
            "mission_timeout_sec",
            default_value="90.0",
            description="End-to-end mission timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "position_tolerance",
            default_value="0.12",
            description="Acceptance final position tolerance in metres.",
        ),
        DeclareLaunchArgument(
            "yaw_tolerance_deg",
            default_value="6.0",
            description="Acceptance final yaw tolerance in degrees.",
        ),
        planned_simulator,
        test_node,
        shutdown,
    ])
