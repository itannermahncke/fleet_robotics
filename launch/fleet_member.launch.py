from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("fleet_robotics"), "test_config.yaml"
    )

    # launch argument for robot name: ...
    # save as param and pass to all nodes
    robot_name = DeclareLaunchArgument("robot_name", default_value="")
    return LaunchDescription(
        [
            Node(
                package="fleet_robotics",
                executable="send_message_node",
                remappings=[
                    ("/cmd_vel", f"{robot_name}/cmd_vel"),
                ],
            ),
        ]
    )
