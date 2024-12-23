import rclpy
from rclpy.node import Node
from rclpy.time import Time

from fleet_robotics_msgs.msg import CrashDetection, PoseStampedSourced, TimeSourced
from std_msgs.msg import Int32MultiArray
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
        super().__init__("crash_handling")

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
        self.create_subscription(TimeSourced, "current_time", self.time_callback, 10)
        self.local_time = None
        self.create_subscription(
            Int32MultiArray, "time_offset", self.offset_callback, 10
        )
        self.time_offsets = {}

        # add as many subscribers as is necessary from other robots
        for num in range(1, self.num_robots):
            # do not subscribe to yourself
            if num == self.robot_num:
                continue
            else:
                # note that subscribers all have the same callback
                self.create_subscription(
                    PoseStampedSourced,
                    f"/robot{num}/pose_estimate",
                    self.generic_pose_callback,
                    10,
                )
                self.create_subscription(
                    PoseStampedSourced,
                    f"/robot{num}/next_step",
                    self.generic_step_callback,
                    10,
                )
        self.remote_pose = None
        self.remote_step = None

        # subscribe to local next steps
        self.local_step_sub = self.create_subscription(
            PoseStampedSourced,
            f"/{self.robot_name}/next_step",
            self.local_step_callback,
            10,
        )
        self.my_latest_step = None

        # publish any steps that could cause a crash if executed
        self.crash_pub = self.create_publisher(CrashDetection, "step_clearance", 10)

        # other attributes for handling crashes
        self.CRASH_RADIUS = 1.0  # meters
        self.RECENCY_REQ = 5 * 10**9  # nanosec

    def time_callback(self, time: TimeSourced):
        """
        Save the latest local time.
        """
        self.local_time = time.nanosec

    def generic_pose_callback(self, pose_msg: PoseStampedSourced):
        """
        Callback when any external pose is received.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.remote_pose = pose_msg

    def generic_step_callback(self, pose_msg: PoseStampedSourced):
        """
        Callback when any external motion step is received.
        """
        # assumes frame_id is robot_name (formatted robotN)
        self.remote_step = pose_msg

    def local_step_callback(self, my_pose_msg: PoseStampedSourced):
        """
        Callback when any local motion step is received. This is when crash
        handling is performed.
        """
        self.get_logger().info(
            f"Crash handler heard {my_pose_msg.pose.position} from {my_pose_msg.source_id}"
        )
        # assumes frame_id is robot_name (formatted robotN)
        self.my_latest_step = my_pose_msg

        # path crashes, for handling mass crash events
        collision_agents = [self.robot_name]

        # check all robot poses/steps
        for robot in range(1, self.num_robots + 1):
            if robot != self.robot_num:

                if self.remote_pose is not None:

                    # if any remote poses are in local step -- execute body crash protocol
                    if self.pose_is_recent(
                        self.remote_pose
                    ) and self.determine_crash_radius(my_pose_msg, self.remote_pose):
                        self.get_logger().info(
                            f"{self.robot_name} detected crash with {self.remote_pose.source_id}'s body"
                        )
                        self.crash_reported(my_pose_msg)
                        return
                if self.remote_step is not None:
                    self.get_logger().info(
                        f"{self.robot_name} detected crash with {self.remote_pose.source_id}'s path"
                    )
                    # if any remote steps are in local step -- note the collision for later
                    if self.pose_is_recent(
                        self.remote_step
                    ) and self.determine_crash_radius(my_pose_msg, self.remote_step):
                        collision_agents.append(robot)

        # evaluate the severity of the path crash
        if len(collision_agents) > 1:
            self.path_crash_reported(my_pose_msg, collision_agents)
            return

        # greenlight the step
        self.no_crash_reported(my_pose_msg)

    def offset_callback(self, time: Int32MultiArray):
        """
        Callback when an offset between the clocks of a fleet member and the local
        robot is received. Save the value.

        Positive numbers are ahead of local time.
        Negative numbers are behind local time.
        """
        # adds new offsets while overwriting existing ones
        self.time_offsets = time

    def determine_crash_radius(self, local_pose: PoseStamped, remote_pose: PoseStamped):
        """
        Determine if there is a risk of crash between two poses. This function
        can be used to compare any combination of poses and steps.

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
        # # determine origin robot of pose stamped
        # origin = pose.source_id

        # # find offset between remote and local time
        # offset = self.time_offsets[origin]

        # # figure out when pose was actually sent using offset
        # source_time = Time.from_msg(pose.stamp)
        # actual_time = source_time + offset

        # ## return true/false if over a threshold
        # return abs(actual_time - self.local_time) < self.RECENCY_REQ
        return True

    def crash_reported(self, pose: PoseStampedSourced):
        """
        In the event of an anticipated body crash, the local Neato will not
        move until the body crash is no longer a concern, and the partner Neato
        has moved out of the local Neato's path. Note that this algorithm will
        only trigger on the local Neato.

        This means that when a potential body crash is reported, this node
        publishes a crash detection immediately, preventing the Neato from
        taking that step at that time.
        """
        self.get_logger().info(f"Pose {pose.pose.position} sounds BAD!!!")
        self.crash_pub.publish(
            CrashDetection(
                header=pose.header, msg_id=pose.msg_id, pose=pose.pose, clearance=False
            )
        )
        self.remote_pose = None
        self.remote_step = None

    def path_crash_reported(self, pose: PoseStampedSourced, robot_list: list[str]):
        """
        In the event of an anticipated path crash, the local Neato will determine
        its priority relevant to the partner Neato, confirm that both agree on this
        priority, and then act accordingly. Note that this algorithm will trigger
        on all Neatos involved in the path crash.
        """
        # currently takes predefined priority
        # no cross-robot comms b/c this logic evaluates predictably across the fleet
        if self.robot_name[-1] == str(min(robot_list)):
            self.no_crash_reported(pose)
        else:
            self.crash_reported(pose)

    def no_crash_reported(self, pose: PoseStampedSourced):
        """
        Greenlight the current step.
        """
        self.get_logger().info(f"Pose {pose.msg_id} sounds good!")
        self.crash_pub.publish(
            CrashDetection(
                header=pose.header, msg_id=pose.msg_id, pose=pose.pose, clearance=True
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = CrashHandlingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
