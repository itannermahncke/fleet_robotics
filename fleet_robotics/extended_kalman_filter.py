import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose

from fleet_robotics_msgs.msg import PoseStampedSourced

from filterpy.kalman import ExtendedKalmanFilter


class ExtendedKalmanFilterNode(Node):
    """
    This node corrects the Neato's wheel odometry using visual odometry data
    and produces a higher-quality pose estimate than each in isolation. This
    node performs this correction by implementing an Extended Kalman Filter.
    """

    def __init__(self):
        """
        Initialize an instance of the SensorFusionNode.
        """
        super().__init__("sensor_fusion")

        # subscribe to pose estimates
        self.odom_sub = self.create_subscription(Pose, "odom", self.wheel_callback, 10)
        self.visual_sub = self.create_subscription(
            Pose, "visual_odom", self.visual_callback, 10
        )

        # publisher for improved pose estimates
        self.pose_pub = self.create_publisher(PoseStampedSourced, "pose_estimate", 10)

        # EKF attributes


def main(args=None):
    rclpy.init(args=args)
    node = ExtendedKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
