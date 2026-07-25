from setuptools import find_packages
from setuptools import setup

package_name = "drone_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="u9736",
    maintainer_email="u9736@todo.todo",
    description="Static-map 3D A* planning and waypoint execution",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "mission_manager_node = "
                "drone_planner.mission_manager_node:main"
            ),
            "planner_node = drone_planner.planner_node:main",
        ],
    },
)
