from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = "drone_web_ground_station"

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
            os.path.join("share", package_name, "static"),
            glob("static/*"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="u9736",
    maintainer_email="1973669388@qq.com",
    description="Single-page HTTP/SSE ground station with clicked goals and patrol missions",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "web_ground_station = drone_web_ground_station.web_server_node:main",
        ],
    },
)
