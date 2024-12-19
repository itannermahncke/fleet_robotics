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

    # launch argument for robot name and num robots
    robot_name = "robot2"

    net_start_node = Node(
        package="fleet_robotics",
        executable="network_startup",
        parameters=[fleet_info, {"robot_name": robot_name}],
    )
    wheel_odom_node = Node(
        package="fleet_robotics",
        executable="odom_adapter",
        parameters=[fleet_info, {"robot_name": robot_name}],
    )
    path_plan_node = Node(
        package="fleet_robotics",
        executable="path_planning",
        parameters=[fleet_info, map_config, {"robot_name": robot_name}],
    )
    crash_handle_node = Node(
        package="fleet_robotics",
        executable="crash_handling",
        parameters=[fleet_info, {"robot_name": robot_name}],
    )
    motion_exe_node = Node(
        package="fleet_robotics",
        executable="motion_execution",
        namespace=robot_name,
    )
    # Implement if we have time
    # update_config_file = ExecuteProcess(
    #     cmd=["python3", "../config/update_config.py", num_robots], shell=True
    # )

    return LaunchDescription(
        [
            net_start_node,
            wheel_odom_node,
            path_plan_node,
            crash_handle_node,
            motion_exe_node,
        ]
    )
