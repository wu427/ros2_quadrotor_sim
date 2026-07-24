from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_parameter(name: str) -> ParameterValue:
    return ParameterValue(
        LaunchConfiguration(name),
        value_type=float,
    )


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("drone_bringup")
    )

    parameter_file = (
        package_share / "config" / "vertical_sim.yaml"
    )

    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    target_yaw_deg = LaunchConfiguration(
        "target_yaw_deg"
    )

    dynamics_node = Node(
        package="drone_dynamics",
        executable="quadrotor_dynamics_node",
        name="quadrotor_dynamics_node",
        output="screen",
        parameters=[str(parameter_file)],
    )

    controller_node = Node(
        package="drone_controller",
        executable="position_controller_node",
        name="position_controller_node",
        output="screen",
        parameters=[str(parameter_file)],
    )

    test_node = Node(
        package="drone_bringup",
        executable="acceptance_test_node",
        name="acceptance_test_node",
        output="screen",
        parameters=[{
            "target_x": ParameterValue(
                target_x,
                value_type=float,
            ),
            "target_y": ParameterValue(
                target_y,
                value_type=float,
            ),
            "target_z": ParameterValue(
                target_z,
                value_type=float,
            ),
            "target_yaw_deg": ParameterValue(
                target_yaw_deg,
                value_type=float,
            ),
            "position_tolerance": float_parameter(
                "position_tolerance"
            ),
            "yaw_tolerance_deg": float_parameter(
                "yaw_tolerance_deg"
            ),
            "linear_speed_tolerance": float_parameter(
                "linear_speed_tolerance"
            ),
            "angular_speed_tolerance": float_parameter(
                "angular_speed_tolerance"
            ),
            "stable_duration_sec": float_parameter(
                "stable_duration_sec"
            ),
            "discovery_timeout_sec": float_parameter(
                "discovery_timeout_sec"
            ),
            "timeout_sec": float_parameter(
                "timeout_sec"
            ),
        }],
    )

    shutdown_when_test_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=test_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason=(
                            "Acceptance test completed"
                        )
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_x",
            default_value="2.0",
            description="Target x position in metres.",
        ),
        DeclareLaunchArgument(
            "target_y",
            default_value="1.0",
            description="Target y position in metres.",
        ),
        DeclareLaunchArgument(
            "target_z",
            default_value="1.5",
            description="Target z position in metres.",
        ),
        DeclareLaunchArgument(
            "target_yaw_deg",
            default_value="90.0",
            description="Target yaw angle in degrees.",
        ),
        DeclareLaunchArgument(
            "position_tolerance",
            default_value="0.05",
            description=(
                "Maximum three-dimensional position "
                "error in metres."
            ),
        ),
        DeclareLaunchArgument(
            "yaw_tolerance_deg",
            default_value="3.0",
            description="Maximum yaw error in degrees.",
        ),
        DeclareLaunchArgument(
            "linear_speed_tolerance",
            default_value="0.05",
            description=(
                "Maximum linear speed in metres per second."
            ),
        ),
        DeclareLaunchArgument(
            "angular_speed_tolerance",
            default_value="0.05",
            description=(
                "Maximum angular speed in radians per second."
            ),
        ),
        DeclareLaunchArgument(
            "stable_duration_sec",
            default_value="1.0",
            description=(
                "Duration for which all acceptance thresholds "
                "must remain satisfied, in seconds."
            ),
        ),
        DeclareLaunchArgument(
            "discovery_timeout_sec",
            default_value="15.0",
            description=(
                "Maximum time to wait for the controller "
                "subscription and valid odometry, in seconds."
            ),
        ),
        DeclareLaunchArgument(
            "timeout_sec",
            default_value="20.0",
            description=(
                "Maximum convergence time after goal "
                "publication, in seconds."
            ),
        ),
        dynamics_node,
        controller_node,
        test_node,
        shutdown_when_test_finishes,
    ])
