from setuptools import find_packages, setup
import glob, os

package_name = "fleet_robotics"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "visual_odom = fleet_robotics.visual_odometry:main",
            "sensor_fusion = fleet_robotics.sensor_fusion:main",
        ],
    },
)
