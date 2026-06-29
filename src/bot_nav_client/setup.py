import os
from glob import glob

from setuptools import find_packages, setup

package_name = "bot_nav_client"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        # CHANGED/ADDED: Install waypoint YAML config into package share directory.
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Duckie",
    maintainer_email="duckie@example.com",
    description="Bot navigation client package for sending goals to Nav2.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Existing raw coordinate client.
            "navigate_to_pose_client = bot_nav_client.navigate_to_pose_client:main",

            # CHANGED/ADDED: Named waypoint client.
            "named_waypoint_client = bot_nav_client.named_waypoint_client:main",
        ],
    },
)