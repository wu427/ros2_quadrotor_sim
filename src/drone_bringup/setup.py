from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = "drone_bringup"


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "urdf",
            ),
            glob("urdf/*.urdf"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "rviz",
            ),
            glob("rviz/*.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="wu427",
    maintainer_email="1973669388@qq.com",
    description=(
        "Launch, visualization, and automated acceptance "
        "tests for the ROS2 quadrotor simulator"
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            (
                "goal_marker_node = "
                "drone_bringup.goal_marker_node:main"
            ),
            (
                "acceptance_test_node = "
                "drone_bringup.acceptance_test_node:main"
            ),
        ],
    },
)
