from setuptools import find_packages, setup
import os
from glob import glob

package_name = "fleet_robotics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ivymahncke",
    maintainer_email="imahncke@olin.edu",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "network_startup = fleet_robotics.network_startup:main",
            "visual_odom = fleet_robotics.visual_odometry:main",
            "odom_adapter = fleet_robotics.odom_adapter:main",
            "sensor_fusion = fleet_robotics.extended_kalman_filter:main",
            "crash_handling = fleet_robotics.crash_handling:main",
            "path_planning = fleet_robotics.path_planningpath_no_obstacle:main",
            "motion_execution = fleet_robotics.motion_execution:main",
        ],
    },
)
