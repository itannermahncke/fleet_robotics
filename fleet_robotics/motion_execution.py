import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist, Pose
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
        super().__init__("motion_execution")

        # bump stop
        self.create_subscription(Bump, "bump", self.bump_callback, 10)
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
        self.latest_twist = Twist()
        self.next_step = None

        # publish cmd velocities to Neato
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_timer = self.create_timer(0.05, self.execute_path)

        # publish when a step has been completed
        self.status_pub = self.create_publisher(Bool, "step_status", 10)

        # attributes
        self.steps: deque[PoseStampedSourced] = deque(maxlen=10)
        self.max_ang_vel = 0.2
        self.max_lin_vel = 0.299
        self.ang_tol = 0.075
        self.lin_tol = 0.1

    def bump_callback(self, bump: Bump):
        if bump.left_front or bump.left_side or bump.right_front or bump.right_side:
            pass
            # self.get_logger().info("BUMP STOP, SHUTTING DOWN MOTION")
            # self.vel_pub.publish(Twist())
            # self.pub_timer.destroy()

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
        self.get_logger().info(f"Analyzing a crash msg id: {crash_msg.msg_id}")
        for step in self.steps:
            # confirm clearance
            if crash_msg.clearance and step.msg_id == crash_msg.msg_id:
                # don't interrupt a current step
                if self.next_step is None:
                    self.get_logger().info(
                        f"Tests passed! Executing message {crash_msg.msg_id}"
                    )
                    self.next_step = step
                    self.steps.remove(step)
                    return

    def step_callback(self, pose_msg: PoseStampedSourced):
        """
        When a new pose step is received, make sure it is not at risk of
        crashing. Then, execute.
        """
        self.steps.append(pose_msg)

    ## FOLLOWING CODE IS STOLEN FROM WALTER... MAKE SURE TO TEST
    def execute_path(self):
        """
        Calculate wheel speeds and publish.
        """
        self.get_logger().info(f"CURRENT STEP GOAL: {self.next_step}")
        if self.next_step is not None:
            # setup
            step = self.next_step
            twist = Twist()

            # calculate error
            lin_error, ang_error = self.calculate_error(
                pose=self.latest_pose, dest=step.pose
            )

            # if angle error is significant, correct
            if abs(ang_error) > self.ang_tol:
                self.get_logger().info(f"Ang error: {ang_error}")
                twist.angular.z = self.max_ang_vel * ang_error / abs(ang_error)
            # if lin error is significant, correct
            elif abs(lin_error) > self.lin_tol:
                self.get_logger().info(f"Lin error: {lin_error}")
                twist.linear.x = self.max_lin_vel * lin_error / abs(lin_error)
            # if within tolerance
            else:
                self.get_logger().info("No error!")
                self.status_pub.publish(Bool(data=True))
                self.next_step = None

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
        delta_x = round(dest.position.x - pose.position.x, 4)
        delta_y = round(dest.position.y - pose.position.y, 4)
        heading = self.euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )[2]
        lin_error = round(math.sqrt(delta_x**2 + delta_y**2), 4)
        ang_error = round(math.atan2(delta_y, delta_x) - heading, 4)

        return lin_error, ang_error

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw). Credit to
        AutomaticAddison for this code lol
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return [roll_x, pitch_y, yaw_z]  # in radians


def main(args=None):
    rclpy.init(args=args)
    node = MotionExecutionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
