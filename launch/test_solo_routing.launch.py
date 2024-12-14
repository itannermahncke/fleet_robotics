from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    fleet_info = os.path.join(
        get_package_share_directory("fleet_robotics"), "fleet_info.yaml"
    )
    map_config = os.path.join(
        get_package_share_directory("fleet_robotics"), "map_config.yaml"
    )
    robot_name = {robot_name: "robot1"}

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="path_planning",
                parameters=[fleet_info, map_config, robot_name],
            ),
            Node(
                package="pie_waiterbot",
                executable="crash_handling",
                parameters=[fleet_info, robot_name],
            ),
            Node(
                package="pie_waiterbot",
                executable="motion_execution",
            ),
        ]
    )
