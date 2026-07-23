from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("drone_bringup")
    params = os.path.join(share, "config", "vertical_sim.yaml")

    return LaunchDescription([
        Node(
            package="drone_dynamics",
            executable="quadrotor_dynamics_node",
            name="quadrotor_dynamics_node",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="drone_controller",
            executable="position_controller_node",
            name="position_controller_node",
            output="screen",
            parameters=[params],
        ),
    ])
