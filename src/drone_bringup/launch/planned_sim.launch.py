"""Launch the core simulator with static-map planning and RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_parameter(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description() -> LaunchDescription:
    """Build the planned-simulation launch description."""
    bringup_share = Path(
        get_package_share_directory("drone_bringup")
    )
    map_share = Path(get_package_share_directory("drone_map"))
    map_file = LaunchConfiguration("map_file")
    rviz_enabled = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    core_simulator = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup_share / "launch" / "sim.launch.py")
                ),
                launch_arguments={"rviz": "false"}.items(),
            )
        ],
    )
    static_map = Node(
        package="drone_map",
        executable="static_map_node",
        name="static_map_node",
        output="screen",
        parameters=[{"map_file": map_file}],
    )
    planner = Node(
        package="drone_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[{
            "map_file": map_file,
            "waypoint_tolerance": _float_parameter(
                "waypoint_tolerance"
            ),
            "final_position_tolerance": _float_parameter(
                "final_position_tolerance"
            ),
            "final_yaw_tolerance_deg": _float_parameter(
                "final_yaw_tolerance_deg"
            ),
            "final_linear_speed_tolerance": _float_parameter(
                "final_linear_speed_tolerance"
            ),
            "final_angular_speed_tolerance": _float_parameter(
                "final_angular_speed_tolerance"
            ),
            "completion_stable_sec": _float_parameter(
                "completion_stable_sec"
            ),
            "max_expanded_nodes": _int_parameter(
                "max_expanded_nodes"
            ),
            "planning_timeout_sec": _float_parameter(
                "planning_timeout_sec"
            ),
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="planned_rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_file",
            default_value=str(map_share / "config" / "static_map.yaml"),
            description="Static-map YAML used by map and planner nodes.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch the planning-specific RViz2 view.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(
                bringup_share / "rviz" / "planned_quadrotor.rviz"
            ),
            description="RViz2 configuration for planned simulation.",
        ),
        DeclareLaunchArgument(
            "waypoint_tolerance",
            default_value="0.18",
            description="3D waypoint switch distance in metres.",
        ),
        DeclareLaunchArgument(
            "final_position_tolerance",
            default_value="0.08",
            description="Final 3D position tolerance in metres.",
        ),
        DeclareLaunchArgument(
            "final_yaw_tolerance_deg",
            default_value="5.0",
            description="Final absolute yaw tolerance in degrees.",
        ),
        DeclareLaunchArgument(
            "final_linear_speed_tolerance",
            default_value="0.08",
            description="Final linear-speed tolerance in metres per second.",
        ),
        DeclareLaunchArgument(
            "final_angular_speed_tolerance",
            default_value="0.08",
            description="Final angular-speed tolerance in radians per second.",
        ),
        DeclareLaunchArgument(
            "completion_stable_sec",
            default_value="1.0",
            description="Duration all final conditions must remain true.",
        ),
        DeclareLaunchArgument(
            "max_expanded_nodes",
            default_value="100000",
            description="Maximum A* node expansions per mission.",
        ),
        DeclareLaunchArgument(
            "planning_timeout_sec",
            default_value="2.0",
            description="Maximum A* wall-clock planning time in seconds.",
        ),
        core_simulator,
        static_map,
        planner,
        rviz,
    ])
