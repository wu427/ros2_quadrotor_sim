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


def _bool_parameter(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


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
        parameters=[{
            "map_file": map_file,
            "publish_point_cloud": _bool_parameter(
                "publish_point_cloud"
            ),
            "point_cloud_spacing": _float_parameter(
                "point_cloud_spacing"
            ),
        }],
    )
    planner = Node(
        package="drone_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[{
            "map_file": map_file,
            "waypoint_pass_radius": _float_parameter(
                "waypoint_pass_radius"
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
            "final_settle_duration": _float_parameter(
                "final_settle_duration"
            ),
            "max_expanded_nodes": _int_parameter(
                "max_expanded_nodes"
            ),
            "planning_timeout_sec": _float_parameter(
                "planning_timeout_sec"
            ),
            "cruise_altitude": _float_parameter(
                "cruise_altitude"
            ),
            "minimum_flight_z": _float_parameter(
                "minimum_flight_z"
            ),
            "takeoff_required": _bool_parameter("takeoff_required"),
            "takeoff_tolerance": _float_parameter(
                "takeoff_tolerance"
            ),
            "path_sample_spacing": _float_parameter(
                "path_sample_spacing"
            ),
            "collision_check_step": _float_parameter(
                "collision_check_step"
            ),
            "lookahead_distance": _float_parameter(
                "lookahead_distance"
            ),
            "smoothing_iterations": _int_parameter(
                "smoothing_iterations"
            ),
            "smoothing_enabled": _bool_parameter(
                "smoothing_enabled"
            ),
            "reference_update_rate": _float_parameter(
                "reference_update_rate"
            ),
        }],
    )
    mission_manager = Node(
        package="drone_planner",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
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
            "publish_point_cloud",
            default_value="false",
            description="Publish sampled raw obstacle surfaces as PointCloud2.",
        ),
        DeclareLaunchArgument(
            "point_cloud_spacing",
            default_value="0.20",
            description="Obstacle surface point spacing in metres.",
        ),
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
            "waypoint_pass_radius",
            default_value="0.22",
            description="Path progress pass radius in metres.",
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
            "final_settle_duration",
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
        DeclareLaunchArgument(
            "cruise_altitude",
            default_value="1.50",
            description="Vertical takeoff target altitude in metres.",
        ),
        DeclareLaunchArgument(
            "minimum_flight_z",
            default_value="1.00",
            description="Minimum non-final flight altitude in metres.",
        ),
        DeclareLaunchArgument(
            "takeoff_required",
            default_value="true",
            description="Require vertical takeoff before path planning.",
        ),
        DeclareLaunchArgument(
            "takeoff_tolerance",
            default_value="0.12",
            description="Takeoff altitude transition tolerance in metres.",
        ),
        DeclareLaunchArgument(
            "path_sample_spacing",
            default_value="0.25",
            description="Arc-length path sample spacing in metres.",
        ),
        DeclareLaunchArgument(
            "collision_check_step",
            default_value="0.05",
            description="Maximum dense collision-check interval in metres.",
        ),
        DeclareLaunchArgument(
            "lookahead_distance",
            default_value="0.55",
            description="Continuous path-following lookahead in metres.",
        ),
        DeclareLaunchArgument(
            "smoothing_iterations",
            default_value="2",
            description="Maximum collision-aware Chaikin iterations.",
        ),
        DeclareLaunchArgument(
            "smoothing_enabled",
            default_value="true",
            description="Enable collision-aware Chaikin smoothing.",
        ),
        DeclareLaunchArgument(
            "reference_update_rate",
            default_value="20.0",
            description="Lookahead reference update rate in hertz.",
        ),
        core_simulator,
        static_map,
        planner,
        mission_manager,
        rviz,
    ])
