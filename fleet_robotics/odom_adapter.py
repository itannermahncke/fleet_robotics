import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from fleet_robotics_msgs.msg import PoseStampedSourced


class OdometryAdapterNode(Node):
    """
    This node reformats wheel odometry data as a pose estimate in the world
    frame.
    """

    def __init__(self):
        """
        Initialize an instance of the OdometryAdapterNode.
        """
        super().__init__("odom_adapter")

        # robot name# access this robot's name
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )

        # subscribe to pose estimates
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # publisher for improved pose estimates
        self.pose_pub = self.create_publisher(PoseStampedSourced, "pose_estimate", 10)
        self.msg_id_counter = 1

    def odom_callback(self, odom_msg: Odometry):
        """
        Refurbish odom message as PoseStampedSourced.
        """
        # set up message
        pose_msg = PoseStampedSourced()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose

        # maintain unique messages
        pose_msg.msg_id = str(self.msg_id_counter)
        self.msg_id_counter += 1

        # source the pose
        pose_msg._source_id = self.robot_name

        # publish to network
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryAdapterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
