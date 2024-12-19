from launch_ros.actions import Node
from launch import LaunchDescription

import os


def generate_launch_description():

    name_str = "robot1"
    robot_name = {"robot_name": name_str}

    return LaunchDescription(
        [
            Node(
                package="fleet_robotics",
                executable="odom_adapter",
                parameters=[robot_name],
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
