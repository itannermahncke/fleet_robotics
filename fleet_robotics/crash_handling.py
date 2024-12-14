import rclpy
from rclpy.node import Node
from rclpy.time import Time

from fleet_robotics_msgs.msg import CrashDetection, PoseStampedSourced, TimeSourced

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
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

        # access local time and offsets
        self.create_subscription(TimeMsg, "local_time", self.time_callback, 10)
        self.local_time = None
        self.create_subscription(TimeMsg, "offsets", self.offset_callback, 10)
        self.time_offsets = {}

        # add as many subscribers as is necessary from other robots
        for num in range(1, self.num_robots):
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
        self.latest_poses = [] * self.num_robots
        self.latest_requests = [] * self.num_robots

        # subscribe to local next steps
        self.local_request_sub = self.create_subscription(
            PoseStamped,
            f"/{self.robot_name}/next_request",
            self.local_request_callback,
            10,
        )
        self.my_latest_request = None

        # publish any requests that could cause a crash if executed
        self.crash_pub = self.create_publisher(
            CrashDetection, "trajectory_clearance", 10
        )

        # other attributes for handling crashes
        self.CRASH_RADIUS = 1.0  # meters
        self.RECENCY_REQ = 5 * 10**9  # nanosec

    def time_callback(self, time: TimeMsg):
        """
        Save the latest local time.
        """
        self.local_time = Time.from_msg(time)

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

    def local_request_callback(self, my_pose_msg: PoseStamped):
        """
        Callback when any local motion request is received. This is when crash
        handling is performed.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.my_latest_request = my_pose_msg

        # path crashes, for handling mass crash events
        collision_agents = [self.robot_name]

        # check all robot poses/trajectories
        for robot in range(0, self.num_robots):
            remote_pose = self.latest_poses[robot]
            remote_trajectory = self.latest_requests[robot]

            # if any remote poses are in local trajectory -- execute body crash protocol
            if self.pose_is_recent(remote_pose) and self.determine_crash_radius(
                self, my_pose_msg, remote_pose
            ):
                self.crash_reported()
                return
            # if any remote trajectories are in local trajectory -- note the collision for later
            elif self.pose_is_recent(remote_trajectory) and self.determine_crash_radius(
                my_pose_msg, remote_trajectory
            ):
                collision_agents.append(robot)

        # evaluate the severity of the path crash
        if len(collision_agents) > 1:
            self.path_crash_reported(my_pose_msg, collision_agents)
            return

        # greenlight the trajectory
        self.no_crash_reported(my_pose_msg)

    def offset_callback(self, time: TimeSourced):
        """
        Callback when an offset between the clocks of a fleet member and the local
        robot is received. Save the value.

        Positive numbers are ahead of local time.
        Negative numbers are behind local time.
        """
        # adds new offsets while overwriting existing ones
        self.time_offsets[time.source_id] = time.nanosec

    def determine_crash_radius(self, local_pose: PoseStamped, remote_pose: PoseStamped):
        """
        Determine if there is a risk of crash between two poses. This function
        can be used to compare any combination of poses and trajectories.

        The threshold should be equivalent to the step size of the grid.

        Returns:
            True if the poses are within a spatial threshold indicating a crash.
            False if this is not the case.
        """
        if (
            abs(local_pose.pose.position.x - remote_pose.pose.position.x)
            < self.CRASH_RADIUS
            and abs(local_pose.pose.position.y - remote_pose.pose.position.y)
            < self.CRASH_RADIUS
        ):
            return True
        return False

    def pose_is_recent(self, pose: PoseStampedSourced):
        """
        Determine if a pose is recent enough to be considered valid.

        Returns:
            True if a pose is within a certain recency threshold.
            False if this is not the case.
        """
        # determine origin robot of pose stamped
        origin = pose.source_id

        # find offset between remote and local time
        offset = self.time_offsets[origin]

        # figure out when pose was actually sent using offset
        source_time = Time.from_msg(pose.stamp)
        actual_time = source_time + offset

        ## return true/false if over a threshold
        return abs(actual_time - self.local_time) < self.RECENCY_REQ

    def crash_reported(self, pose: PoseStamped):
        """
        In the event of an anticipated body crash, the local Neato will not
        move until the body crash is no longer a concern, and the partner Neato
        has moved out of the local Neato's path. Note that this algorithm will
        only trigger on the local Neato.

        This means that when a potential body crash is reported, this node
        publishes a crash detection immediately, preventing the Neato from
        taking that trajectory at that time.
        """
        self.crash_pub.publish(
            CrashDetection(header=pose.header, pose=pose.pose, clearance=False)
        )

    def path_crash_reported(self, pose: PoseStamped, robot_list: list[str]):
        """
        In the event of an anticipated path crash, the local Neato will determine
        its priority relevant to the partner Neato, confirm that both agree on this
        priority, and then act accordingly. Note that this algorithm will trigger
        on all Neatos involved in the path crash.
        """
        # currently takes predefined priority
        # no cross-robot comms b/c this logic evaluates predictably across the fleet
        if self.robot_name[-1] == min(robot_list):
            self.no_crash_reported(pose)
        else:
            self.crash_reported(pose)

    def no_crash_reported(self, pose: PoseStamped):
        """
        Greenlight the current pose trajectory.
        """
        self.crash_pub.publish(
            CrashDetection(header=pose.header, pose=pose.pose, clearance=True)
        )


def main(args=None):
    rclpy.init(args=args)
    node = CrashHandlingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
