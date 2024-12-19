import rclpy
from rclpy.node import Node
from rclpy import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from fleet_robotics_msgs.msg import PoseStampedSourced

import matplotlib.pyplot as plt
import math


class PosePlotterNode(Node):
    """
    This node makes plots of the wheel odometry compared to the visual odometry
    compared to the EKF pose estimate.
    """

    def __init__(self):
        """
        Initialize an instance of the PosePlotterNode.
        """
        super().__init__("pose_plotter")
        # access this robot's name
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )

        # subscribers
        self.create_subscription(Odometry, "odom", self.odom, 10)
        # self.create_subscription(PoseStampedSourced, "visual_odom", self.visual, 10)
        self.create_subscription(PoseStampedSourced, "pose_estimate", self.ekf, 10)

        # pub a velocity
        # self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # msg = Twist()
        # msg.linear = 0.1
        # msg.angular = 0.0
        # self.vel_pub.publish()

        self.odom_dict = {
            "x": [],
            "y": [],
            "theta": [],
            "time": [],
        }

        self.visual_dict = {
            "x": [],
            "y": [],
            "theta": [],
            "time": [],
        }

        self.ekf_dict = {
            "x": [],
            "y": [],
            "theta": [],
            "time": [],
        }

        # timer for plotting
        self.plot_timer = self.create_timer(20, self.timer_callback)

    def timer_callback(self):
        """
        After gathering 20 seconds of data, close subscribers and plot.
        """
        self.plot_timer.destroy()
        fig, ax = plt.subplots()
        ax.plot(self.odom_dict["x"], self.odom_dict["y"], "k-")
        # ax.plot(self.visual_dict["x"], self.odom_dict["y"], "b-")
        ax.plot(self.odom_dict["x"], self.odom_dict["y"], "r-")
        plt.show()

    def odom(self, odom: Odometry):
        """
        Save the latest odom for plotting.
        """
        # cartesian
        self.odom_dict["x"].append(odom.pose.pose.position.x)
        self.odom_dict["y"].append(odom.pose.pose.position.y)

        # heading
        theta = self.euler_from_quaternion(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )[2]
        self.odom_dict["theta"].append(theta)

        # time
        self.odom_dict["time"].append(self.get_clock().now().nanoseconds)

    def visual(self, visual: PoseStampedSourced):
        """
        Save the latest visual for plotting.
        """
        # cartesian
        self.visual_dict["x"].append(visual.pose.position.x)
        self.visual_dict["y"].append(visual.pose.position.y)

        # heading
        theta = self.euler_from_quaternion(
            visual.pose.orientation.x,
            visual.pose.orientation.y,
            visual.pose.orientation.z,
            visual.pose.orientation.w,
        )[2]
        self.visual_dict["theta"].append(theta)

        # time
        self.visual_dict["time"].append(self.get_clock().now().nanoseconds)

    def ekf(self, ekf: PoseStampedSourced):
        """
        Save the latest ekf estimate for plotting.
        """
        # cartesian
        self.ekf_dict["x"].append(ekf.pose.position.x)
        self.ekf_dict["y"].append(ekf.pose.position.y)

        # heading
        theta = self.euler_from_quaternion(
            ekf.pose.orientation.x,
            ekf.pose.orientation.y,
            ekf.pose.orientation.z,
            ekf.pose.orientation.w,
        )[2]
        self.ekf_dict["theta"].append(theta)

        # time
        self.ekf_dict["time"].append(self.get_clock().now().nanoseconds)

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

        return roll_x, pitch_y, yaw_z  # in radians


def main(args=None):
    rclpy.init(args=args)
    node = PosePlotterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
