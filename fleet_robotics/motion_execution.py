import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist, Pose

from tf_transformations import euler_from_quaternion

from fleet_robotics_msgs.msg import CrashDetection, PoseStampedSourced

from collections import deque
import math


class MotionExecutionNode(Node):
    """
    This node rationalizes step plans from the path_planning node and
    crash priority from the crash_handling node. It outputs a Twist message
    that commands the robot to act or wait, depending on its situation.
    """

    def __init__(self):
        """
        Initialize an instance of the MotionExecutionNode.
        """
        super.__init__("motion_execution")

        # subscribe to the crash handler and the path planner
        self.crash_sub = self.create_subscription(
            CrashDetection, "step_clearance", self.clearance_callback, 10
        )
        self.step_sub = self.create_subscription(
            PoseStampedSourced, "next_step", self.step_callback, 10
        )

        # subscribe to current pose estimate
        self.pose_sub = self.create_subscription(
            PoseStampedSourced, "pose_estimate", self.pose_callback, 10
        )
        self.latest_pose = None
        self.latest_twist = None

        # publish cmd velocities to Neato
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # publish when a step has been completed
        self.goal_pub = self.create_publisher(Bool, "goal_status", 10)

        # attributes
        self.clearances: deque[CrashDetection] = deque(maxlen=10)
        self.max_ang_vel = 0.49
        self.max_lin_vel = 0.29
        self.tolerance = 0.05

    def pose_callback(self, pose_msg: PoseStampedSourced):
        """
        Save most recent pose estimate.
        """
        self.latest_pose = pose_msg.pose

    def clearance_callback(self, crash_msg: CrashDetection):
        """
        When the crash handler informs this node about the clearance for a
        particular pose/step, note it here.
        """
        self.clearances.append(crash_msg)

    def step_callback(self, pose_msg: PoseStampedSourced):
        """
        When a new pose step is received, make sure it is not at risk of
        crashing. Then, execute.
        """
        for crash in self.clearances:
            if crash.clearance and pose_msg.msg_id == crash.msg_id:
                self.execute_path()

    ## FOLLOWING CODE IS STOLEN FROM WALTER... MAKE SURE TO TEST
    def execute_path(self):
        """
        Calculate wheel speeds and publish.
        """
        twist = Twist()
        # calculate error
        lin_error, ang_error = self.calculate_error()

        # if angle error is significant, correct
        if ang_error > self.tolerance:
            twist.angular.z = self.max_ang_vel
        # if lin error is significant, correct
        elif lin_error > self.tolerance:
            twist.linear.x = self.max_lin_vel
        # if within tolerance
        else:
            self.goal_pub.publish(Bool(data=True))

        # publish OR skip if identical to latest
        if not (
            twist.linear.x == self.latest_twist.linear.x
            and twist.angular.z == self.latest_twist.angular.z
        ):
            self.vel_pub.publish(twist)
            self.latest_twist = twist

    def calculate_error(self, pose: Pose, dest: Pose):
        """
        Calculate error between current pose and desired pose.
        """
        delta_x = dest.position.x - pose.position.x
        delta_y = dest.position.y - pose.position.y
        heading = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        lin_error = math.sqrt(delta_x**2 + delta_y**2)
        ang_error = math.atan2(delta_y, delta_x) - heading

        return lin_error, ang_error


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
