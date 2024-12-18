import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from fleet_robotics_msgs.msg import PoseStampedSourced


class ObstacleDetectionNode(Node):
    """
    This node uses LiDAR to detect obstacles near the Neato. This Neato makes
    sure not to count other Neatos as obstacles by comparing the obstacle's
    pose with fleet-supplied poses and filtering out those that overlap within
    a single grid space.
    """

    def __init__(self):
        """
        Initialize an instance of the ObstacleDetectionNode.
        """
        super().__init__("obstacle_detection")

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

        # subscribe to pose estimates
        self.odom_sub = self.create_subscription(
            LaserScan, "stable_scan", self.scan_callback, 10
        )

        # publisher for improved pose estimates
        self.pose_pub = self.create_publisher(PoseStampedSourced, "pose_estimate", 10)

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


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
