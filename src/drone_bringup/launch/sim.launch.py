from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("drone_bringup")
    )

    parameter_file = (
        package_share / "config" / "vertical_sim.yaml"
    )

    urdf_file = (
        package_share / "urdf" / "quadrotor.urdf"
    )

    rviz_config = (
        package_share / "rviz" / "quadrotor.rviz"
    )

    robot_description = urdf_file.read_text(
        encoding="utf-8"
    )

    rviz_enabled = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz2 visualization",
        ),

        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            name="quadrotor_dynamics_node",
            output="screen",
            parameters=[str(parameter_file)],
        ),

        Node(
            package="drone_controller",
            executable="position_controller_node",
            name="position_controller_node",
            output="screen",
            parameters=[str(parameter_file)],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
            }],
        ),

        Node(
            package="drone_bringup",
            executable="goal_marker_node",
            name="goal_marker_node",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d",
                str(rviz_config),
            ],
            condition=IfCondition(rviz_enabled),
        ),
    ])
