import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped
import rclpy.time


class CrashHandlingNode(Node):
    """
    This node prevents Neatos from crashing into one another. Two types of
    crashes are expected:
    - Body crashes, in which a local Neato's path collides with another Neato's body
    - Path crashes, in which a local Neato's path collides with another Neato's crash

    In the event of an anticipated body crash, the local Neato will not
    move until the body crash is no longer a concern, and the partner Neato
    has moved out of the local Neato's path. Note that this algorithm will
    only trigger on the local Neato.

    In the event of an anticipated path crash, the local Neato will determine
    its priority relevant to the partner Neato, confirm that both agree on this
    priority, and then act accordingly. Note that this algorithm will trigger
    on all Neatos involved in the path crash.
    """

    def __init__(self):
        """
        Initialize an instance of the CrashHandlingNode.
        """
        super.__init__("crash_handling")

        # access number of robots
        self.declare_parameter("num_robots", rclpy.Parameter.Type.INTEGER)
        self.num_robots = (
            self.get_parameter("num_robots").get_parameter_value().integer_value
        )

        # access this robot's name
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.robot_num = int(self.robot_name[-1])

        # add as many subscribers as is necessary from other robots
        for num in range(0, self.num_robots):
            # do not subscribe to yourself
            if num == self.robot_num:
                continue
            else:
                # note that subscribers all have the same callback
                self.create_subscription(
                    PoseStamped,
                    f"/robot{num}/pose_estimate",
                    self.generic_pose_callback,
                    10,
                )
                self.create_subscription(
                    PoseStamped,
                    f"/robot{num}/next_request",
                    self.generic_request_callback,
                    10,
                )

        # subscribe to local pose and next request
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            f"/{self.robot_name}/pose_estimate",
            self.local_pose_callback,
            10,
        )
        self.local_request_sub = self.create_subscription(
            PoseStamped,
            f"/{self.robot_name}/next_request",
            self.local_request_callback,
            10,
        )

        # latest robot poses and requests
        self.latest_poses = [] * self.num_robots
        self.latest_requests = [] * self.num_robots

        # local pose and request
        self.declare_parameter("init_pose", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.my_latest_pose = (
            self.get_parameter("init_pose").get_parameter_value().double_array_value
        )
        self.my_latest_request = None

    def generic_pose_callback(self, pose_msg: PoseStamped):
        """
        Callback when any external pose is received.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.latest_poses[int(pose_msg.header.frame_id[-1])] = pose_msg

    def generic_request_callback(self, pose_msg: PoseStamped):
        """
        Callback when any external motion request is received.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.latest_requests[int(pose_msg.header.frame_id[-1])] = pose_msg

    def local_pose_callback(self, pose_msg: PoseStamped):
        """
        Callback when any local pose is received.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.my_latest_pose = pose_msg

    def local_request_callback(self, pose_msg: PoseStamped):
        """
        Callback when any local motion request is received. This is when crash
        handling is performed.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.my_latest_request = pose_msg

        # if any remote poses are in local trajectory -- execute body crash protocol

        # if any remote trajectories are in local trajectory -- execute path crash protocol

    def determine_crash_radius(self, local_pose: Pose, remote_pose: Pose):
        """
        Determine if there is a risk of crash between two poses. This function
        can be used to compare any combination of poses and trajectories.

        Returns:
            True if the poses are within a spatial threshold indicating a crash.
            False if this is not the case.
        """

    def pose_is_recent(self, pose: PoseStamped):
        """
        Determine if a pose is recent enough to be considered valid.

        Returns:
            True if a pose is within a certain recency threshold.
            False if this is not the case.
        """
        rclpy.time.
        Time.from_msg(pose.header.stamp).seconds_nanoseconds()


def main(args=None):
    rclpy.init(args=args)
    node = CrashHandlingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
