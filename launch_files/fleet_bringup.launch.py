from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # first, declare launch arguments: Neato count and IP addresses
    num_neatos = DeclareLaunchArgument("num_neatos", default_value="2")
    ip_addresses = DeclareLaunchArgument(
        "ip_addresses", default_value="192.168.16.50,192.168.16.51"
    )

    # next, source the Neato bringup file
    action_list = []
    for i in range(0, num_neatos):
        robot_name = f"robot_{i}"
        host = ip_addresses[i]
        neato_launch = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("neato_node2"), "launch"),
                    "bringup_multi.py",
                ]
            ),
            launch_arguments={
                "robot_name": TextSubstitution(text=robot_name),
                "host": TextSubstitution(text=host),
                "udp_video_port": TextSubstitution(text=str(5002 + i)),
                "udp_sensor_port": TextSubstitution(text=str(7777 + i)),
            }.items(),
        )
        action_list.append(neato_launch)

    return LaunchDescription(action_list)
