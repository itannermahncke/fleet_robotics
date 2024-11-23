from launch_ros.actions import Node
from launch import LaunchDescription
import sys
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # # first, declare launch arguments: Neato count and IP addresses
    # num_neatos = DeclareLaunchArgument("num_neatos", default_value="2")
    # ip_addresses = DeclareLaunchArgument(
    #     "ip_addresses", default_value="192.168.16.50 , 192.168.16.51"
    # )

    # next, source the Neato bringup file
    num_neatos = None
    ip_addresses = None
    for arg in sys.argv:
        if arg.startswith("num_neatos:="):
            num_neatos = int(arg.split(":=")[1])
        if arg.startswith("ip_addresses:="):
            ip_addresses = arg.split(":=")[1]
            ip_split = ip_addresses.split(",")

    action_list = []
    for i in range(num_neatos):
        robot_name = f"robot_{i}"
        host = ip_split[i]
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
if __name__ == "__main__":
    generate_launch_description()

