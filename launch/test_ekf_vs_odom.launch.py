from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    name_str = "robot1"
    robot_name = {"robot_name": name_str}

    fleet_info = os.path.join(
        get_package_share_directory("fleet_robotics"), "fleet_info.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="fleet_robotics",
                executable="odom_adapter",
                parameters=[robot_name, fleet_info],
            ),
            Node(
                package="fleet_robotics",
                executable="sensor_fusion",
                parameters=[robot_name],
            ),
            Node(
                package="fleet_robotics",
                executable="pose_plotter",
                parameters=[robot_name],
            ),
        ]
    )
