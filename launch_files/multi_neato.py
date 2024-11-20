from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the number of Neatos and a comma-separated list of IP addresses
    num_neatos = DeclareLaunchArgument('num_neatos', default_value='2')
    ip_addresses = DeclareLaunchArgument('ip_addresses', default_value='192.168.16.50,192.168.16.51')

    # Directory of the bringup_multi.py file
    bringup_multi_launch_dir = os.path.join(
        get_package_share_directory('neato_2_gazebo'),'launch'
    )

    def create_robot_launch(context, *args, **kwargs):
        num_robots = int(LaunchConfiguration('num_neatos').perform(context))
        ips = LaunchConfiguration('ip_addresses').perform(context).split(',')

        actions = []
        for i in range(num_robots):
            robot_name = f"robot_{i}"
            host = ips[i]
            actions.append(
                GroupAction([
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([bringup_multi_launch_dir, '/bringup_multi.py']),
                        launch_arguments={
                            'robot_name': TextSubstitution(text=robot_name),
                            'host': TextSubstitution(text=host),
                            'udp_video_port': TextSubstitution(text=str(5002 + i)),
                            'udp_sensor_port': TextSubstitution(text=str(7777 + i)),
                        }.items()
                    )
                ])
            )
        return actions

    return LaunchDescription([
        num_neatos,
        ip_addresses,
        OpaqueFunction(function=create_robot_launch)
    ])
