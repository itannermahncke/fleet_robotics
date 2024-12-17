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

    name_str = "robot1"
    robot_name = {"robot_name": name_str}

    return LaunchDescription(
        [
            Node(
                package="fleet_robotics",
                executable="odom_adapter",
                parameters=[robot_name],
                remappings=[("pose_estimate", f"{name_str}/pose_estimate")],
            ),
            Node(
                package="fleet_robotics",
                executable="path_planning",
                parameters=[fleet_info, map_config, robot_name],
                remappings=[
                    ("pose_estimate", f"{name_str}/pose_estimate"),
                    ("next_step", f"{name_str}/next_step"),
                    ("next_step", f"{name_str}/step_status"),
                ],
            ),
            Node(
                package="fleet_robotics",
                executable="crash_handling",
                parameters=[fleet_info, robot_name],
                remappings=[
                    ("step_clearance", f"{name_str}/step_clearance"),
                ],
            ),
            Node(
                package="fleet_robotics",
                executable="motion_execution",
                remappings=[
                    ("step_clearance", f"{name_str}/step_clearance"),
                    ("pose_estimate", f"{name_str}/pose_estimate"),
                    ("next_step", f"{name_str}/next_step"),
                    ("next_step", f"{name_str}/step_status"),
                ],
            ),
        ]
    )
