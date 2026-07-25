from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = "drone_map"

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
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="u9736",
    maintainer_email="u9736@todo.todo",
    description="Static 3D AABB map and collision geometry",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "static_map_node = drone_map.static_map_node:main",
        ],
    },
)
