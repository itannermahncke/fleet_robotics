from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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

    # launch argument for robot name
    robot_name = DeclareLaunchArgument("robot_name", default_value="")  # for remapping
    robot_name_param = {"robot_name": robot_name}  # for param

    return LaunchDescription(
        [
            Node(
                package="pie_waiterbot",
                executable="network_startup",
                parameters=[fleet_info, robot_name_param],
            ),
            Node(
                package="pie_waiterbot",
                executable="visual_odom",
            ),
            Node(
                package="pie_waiterbot",
                executable="path_planning",
                parameters=[fleet_info, map_config, robot_name_param],
            ),
            Node(
                package="pie_waiterbot",
                executable="crash_handling",
                parameters=[fleet_info, robot_name_param],
            ),
            Node(
                package="pie_waiterbot",
                executable="motion_execution",
            ),
        ]
    )
