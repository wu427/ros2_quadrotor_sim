from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("drone_bringup")

    parameter_file = os.path.join(
        package_share,
        "config",
        "vertical_sim.yaml",
    )

    dynamics_node = Node(
        package="drone_dynamics",
        executable="quadrotor_dynamics_node",
        name="quadrotor_dynamics_node",
        output="screen",
        parameters=[parameter_file],
    )

    controller_node = Node(
        package="drone_controller",
        executable="position_controller_node",
        name="position_controller_node",
        output="screen",
        parameters=[parameter_file],
    )

    return LaunchDescription([
        dynamics_node,
        controller_node,
    ])
