from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

# Helper function to create nodes for each robot
def create_neato_nodes(robot_index, robot_name, udp_video_port, udp_sensor_port, host):
    interfaces_launch_file_dir = os.path.join(get_package_share_directory('neato2_gazebo'), 'launch')

    return GroupAction(actions=[
        PushRosNamespace(namespace=robot_name),  # Use namespace for robot isolation
        Node(
            package='neato_node2',
            executable='neato_node',
            name='neato_driver',
            parameters=[
                {"use_udp": True},
                {"udp_port": udp_sensor_port},
                {"robot_name": robot_name},
                {"host": host}
            ],
            output='screen'
        ),
        Node(
            package='fix_scan',
            executable='fix_scan',
            name='fix_scan',
            parameters=[{"robot_name": robot_name}]
        ),
        Node(
            package='neato_node2',
            executable='setup_udp_stream',
            name='udp_stream_setup',
            parameters=[
                {"receive_port": udp_video_port},
                {"width": 1024},
                {"height": 768},
                {"fps": 30},
                {"host": host}
            ],
            output='screen'
        ),
        Node(
            package='gscam',
            executable='gscam_node',
            parameters=[
                {'preroll': True},
                {'camera_name': 'camera'},
                {'use_gst_timestamps': False},
                {'frame_id': 'camera'},
                {'gscam_config': f'udpsrc port={udp_video_port} ! application/x-rtp, payload=96 ! '
                                 f'rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'}
            ]
        )
    ])

# Main launch description
def generate_launch_description():
    # Declare number of robots
    declare_robot_count = DeclareLaunchArgument(
        'robot_count', default_value='2', description='Number of robots to launch'
    )

    robot_count = LaunchConfiguration('robot_count')

    # Declare arguments common for all robots
    robots = []
    for i in range(10):  # Support up to 10 robots for this example
        robots.append(
            DeclareLaunchArgument(
                f'robot{i}_name', default_value=f'robot{i}', description=f'Namespace for robot {i}'
            )
        )
        robots.append(
            DeclareLaunchArgument(
                f'robot{i}_host', default_value=f'192.168.16.{50 + i}', description=f'Host IP for robot {i}'
            )
        )
        robots.append(
            DeclareLaunchArgument(
                f'robot{i}_udp_video_port', default_value=str(5000 + i), description=f'Video port for robot {i}'
            )
        )
        robots.append(
            DeclareLaunchArgument(
                f'robot{i}_udp_sensor_port', default_value=str(7000 + i), description=f'Sensor port for robot {i}'
            )
        )

    # Dynamically add robot nodes based on robot_count
    robot_nodes = []
    for i in range(10):  # Up to 10 robots
        robot_nodes.append(
            GroupAction(
                actions=create_neato_nodes(
                    robot_index=i,
                    robot_name=LaunchConfiguration(f'robot{i}_name'),
                    udp_video_port=LaunchConfiguration(f'robot{i}_udp_video_port'),
                    udp_sensor_port=LaunchConfiguration(f'robot{i}_udp_sensor_port'),
                    host=LaunchConfiguration(f'robot{i}_host'),
                ),
                condition=IfCondition(f'{robot_count} > {i}'),  # Only include robots up to `robot_count`
            )
        )

    return LaunchDescription([declare_robot_count] + robots + robot_nodes)
