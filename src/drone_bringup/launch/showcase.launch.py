"""Launch the planned simulator with Qt, Web, or no user interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the one-command showcase launch description."""
    bringup_share = Path(get_package_share_directory("drone_bringup"))
    map_share = Path(get_package_share_directory("drone_map"))
    ui = LaunchConfiguration("ui")
    gui = LaunchConfiguration("gui")

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
    qt_condition = IfCondition(PythonExpression([
        "('", ui, "' == 'qt') or ('", ui,
        "' == 'auto' and '", gui, "'.lower() in ['true','1','yes'])",
    ]))
    web_condition = IfCondition(PythonExpression([
        "'", ui, "' == 'web'",
    ]))
    ground_station = Node(
        package="drone_ground_station",
        executable="ground_station",
        name="ground_station",
        output="screen",
        condition=qt_condition,
    )
    web_ground_station = Node(
        package="drone_web_ground_station",
        executable="web_ground_station",
        name="web_ground_station",
        output="screen",
        parameters=[{
            "host": LaunchConfiguration("web_host"),
            "port": ParameterValue(
                LaunchConfiguration("web_port"), value_type=int
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
        condition=web_condition,
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
            description="Backward-compatible Qt enable flag when ui:=auto.",
        ),
        DeclareLaunchArgument(
            "ui",
            default_value="auto",
            description="User interface: auto, qt, web, or none.",
        ),
        DeclareLaunchArgument(
            "web_host",
            default_value="127.0.0.1",
            description="Web dashboard bind host.",
        ),
        DeclareLaunchArgument(
            "web_port",
            default_value="8765",
            description="Web dashboard port.",
        ),
        DeclareLaunchArgument(
            "open_browser",
            default_value="true",
            description="Open the web dashboard in the default browser.",
        ),
        DeclareLaunchArgument(
            "history_duration_sec",
            default_value="60.0",
            description="Complete task history duration for web plots.",
        ),
        DeclareLaunchArgument(
            "max_history_points",
            default_value="10000",
            description="Maximum full-task telemetry samples.",
        ),
        DeclareLaunchArgument(
            "publish_point_cloud",
            default_value="false",
            description="Publish the optional obstacle PointCloud2.",
        ),
        planned,
        ground_station,
        web_ground_station,
    ])
