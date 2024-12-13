from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # this contains the number of robots
    fleet_info = os.path.join(
        get_package_share_directory("fleet_robotics"), "fleet_info.yaml"
    )

    # launch argument for robot name: ...
    robot_name_param = None  # save as param and pass to all nodes
    robot_name_str = None  # also save as string, for remappign

    return LaunchDescription(
        [
            # for each robot, do this.
            Node(
                package="pie_waiterbot",
                executable="network_startup",
                # also add the robot name param
                parameters=[fleet_info],
                # remap every topic here
                remappings=[("example_topic", f"{robot_name_str}/example_topic")],
            ),
        ]
    )
