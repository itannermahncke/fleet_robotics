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
    robot_names = ["robot1", "robot2", "robot3", "robot4"]
    # TODO get robot names from launch args in bash file
    robot_name_param = [
        {"robot_name": robot_names[0]},
        {"robot_name": robot_names[1]},
        {"robot_name": robot_names[2]},
        {"robot_name": robot_names[3]},
    ]  # save as param and pass to all nodes

    topic_names = [
        "speak",
        "heard",
        "comm_check",
        "current_time",
        "start_timer",
        "start_node",
        "time_offset",
    ]
    remap_list = []
    robot_remap = []
    for robot in robot_names:
        for topic in topic_names:
            remap = (topic, f"{robot}/{topic}")
            robot_remap.append(remap)
        remap_list.append(robot_remap)
        robot_remap = []

    # [("example_topic", f"{robot_names[num]}/example_topic")]
    launch_description = []
    for num in range(len(robot_names)):
        launch_description.append(
            Node(
                package="fleet_robotics",
                executable="network_startup",
                parameters=[fleet_info, robot_name_param[num]],
                remappings=remap_list[num],
                name=f"{robot_names[num]}_network_startup",
            )
        )
    return LaunchDescription(launch_description)
