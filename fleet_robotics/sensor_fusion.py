from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose



class SensorFusionNode(Node):
    """
    Node that utilizes Kalman filtering to fuse pose estimates supplied by
    visual odometry and dead reckoning.
    """

    def __init__(self):
        """
        Initializes an instance of the SensorFusionNode.
        """
        super().__init__("sensor_fusion")
        # publishers and subscribers
        self.vo_sub = self.create_subscription(
            Pose, "visual_pose", self.vo_callback, 10
        )
        self.dr_sub = self.create_subscription(
            Pose, "dead_reckoning", self.dr_callback, 10
        )
        self.pose_publisher = self.create_publisher(Pose, "robot_pose", 10)


        self.vo_pose = None
        self.dr_pose = None
        self.fused_pose = np.empty(3)

        # parameter attributes with defaults
        self.prior = np.array(self.get_parameter("prior", [0.0, 0.0, 0.0]))
        self.timestep_size = self.get_parameter("timestep_size", 0.1)

        # attributes for Kalman Filter
        self.state_size = 3  # x, y, theta
        self.measurement_size = 1
        self.uncertainty = 1000.0
        self.dead_reckoning_noise = 5
        self.noise_variance = 0.13

        # set up Kalman filter
        filter = KalmanFilter(dim_x=self.state_size, dim_z=1)
        filter.x = self.prior  # initial state
        filter.F = np.array([[1.0, 1.0], [0.0, 1.0]])  # state transition matrix
        filter.H = np.array([[1.0, 0.0]])  # measurement function
        filter.P *= self.uncertainty  # uncertainty matrix
        filter.R = self.dead_reckoning_noise * np.ones(
            (self.measurement_size, self.measurement_size)
        )
        filter.Q = Q_discrete_white_noise(
            self.measurement_size, self.timestep_size, self.noise_variance
        )

    def vo_callback(self, vo_msg):
        """
        Callback function when latest visual odometry-based pose estimate is received.
        """
        self.vo_pose = vo_msg
        

    def dr_callback(self, dr_msg):
        """
        Callback function for when latest dead reckoning pose estimate is received.
        """
        self.dr_pose = dr_msg


    def sensor_fusion(self):
        """
        Fuse the two pose estimates into one
        """
        measured = self.dr_pose + self.vo_pose
        self.fused_pose = esimate + measured


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
