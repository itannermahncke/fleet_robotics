from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("fleet_robotics"), "test_config.yaml"
    )

    # launch argument for robot name: ...
    # save as param and pass to all nodes

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="visual_odom",
                parameters=[config],
            ),
            Node(
                package="pie_waiterbot",
                executable="crash_handling",
            ),
            Node(
                package="pie_waiterbot",
                executable="path_planning",
                parameters=[config],
            ),
        ]
    )
