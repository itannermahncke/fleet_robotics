import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance
from fleet_robotics_msgs.msg import PoseStampedSourced

from scipy import linalg
import numpy as np
import math


class ExtendedKalmanFilterNode(Node):
    """
    This node maintains an estimate of the robot's position and velocity in
    the world that is procedurally corrected by visual odometry estimates.
    """

    def __init__(self):
        """
        Initialize an instance of the SensorFusionNode.
        """
        super().__init__("sensor_fusion")

        # robot name
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )

        # subscribe to odometry estimator
        self.visual_sub = self.create_subscription(Odometry, "odom", self.ekf_loop, 10)

        # publisher for improved pose estimates
        self.pose_pub = self.create_publisher(PoseStampedSourced, "pose_estimate", 10)
        self.msg_id_counter = 0

        # EKF
        self.x_state = np.zeros((6, 1))  # x, y, theta, dx, dy, dtheta
        self.P_covariance = np.eye(len(self.x_state)) * 0.1
        self.dt = 0.1
        self.F_linearize = np.array(
            [
                [1, self.dt, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, self.dt, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, self.dt],
                [0, 0, 0, 0, 0, 1],
            ]
        )
        self.H_measurement_function = np.eye(len(self.x_state))  # no hidden variables
        self.Q_process_noise = np.zeros((len(self.x_state), len(self.x_state))) + 0.1
        self.R_sensor_noise = np.eye(len(self.x_state)) * 10

    def ekf_loop(self, odom: Odometry):
        """
        Update the current state model with a new observation, and then predict
        the next state.
        """
        # predict the next step
        x_predict, P_predict = self.predict(
            self.x_state, self.P_covariance, self.F_linearize, self.Q_process_noise
        )

        # construct the observation as a vector
        z_observation = self.observation_from_message(odom.pose, odom.twist)

        # update the state and covariance
        self.x_state, self.P_covariance = self.update(
            x_predict,
            P_predict,
            z_observation,
            self.H_measurement_function,
            self.R_sensor_noise,
        )

        # publish the new state estimate
        self.pose_pub.publish(self.message_from_state_vector(self.x_state))

    def observation_from_message(
        self, observed_pose: PoseStampedSourced, observed_twist: TwistWithCovariance
    ):
        """
        Assemble the state vector using messages from sensors. This assumes
        that the sensor provides pose and twist data.
        """
        # construct vector
        state = np.zeros((6, 1))

        # position
        state[0] = observed_pose.pose.position.x
        state[1] = observed_pose.pose.position.y
        state[2] = self.euler_from_quaternion(
            observed_pose.pose.orientation.x,
            observed_pose.pose.orientation.y,
            observed_pose.pose.orientation.z,
            observed_pose.pose.orientation.w,
        )[2]

        # velocity
        state[3] = observed_twist.twist.linear.x * math.cos(state[2])
        state[4] = observed_twist.twist.linear.x * math.sin(state[2])
        state[5] = observed_twist.twist.angular.z

        return state

    def message_from_state_vector(self, state: np.ndarray):
        """
        Assemble a PoseStampedSourced message using the most recent state
        update.
        """
        # pose
        message = PoseStampedSourced()

        # cartesian
        message.pose.position.x = state[(0, 0)]
        message.pose.position.y = state[(1, 0)]

        # do the angle stuff
        normal_angle = self.normalize_angle(state[(2, 0)])
        quat = self.quaternion_from_euler(0, 0, normal_angle)
        message.pose.orientation.x = quat[0]
        message.pose.orientation.y = quat[1]
        message.pose.orientation.z = quat[2]
        message.pose.orientation.z = quat[3]

        # stamp
        message.msg_id = str(self.msg_id_counter)
        self.msg_id_counter += 1
        message.source_id = self.robot_name

        return message

    def predict(self, x, P, F: np.ndarray, Q):
        """
        Returns a prediction update for pose and pose covariance according to
        the process model.

        Arguments:
            x: Vector of state variables.
            P: Covariance of the pose, which represents the sensor's uncertainty.
            F: State transition matrix, which is where the linearizing happens.
            Q: Noise; in this case, discrete white noise.
        """
        xt = F @ x
        Pt = F @ P @ F.T + Q
        return xt, Pt

    def update(self, x, P, z, H: np.ndarray, R):
        """
        Performs an update to the state and covariance.

        Arguments:
            x: Vector of state variables.
            P: Covariance of the Kalman filter.
            z: Observation from the sensor; same structure as x.
            H: Measurement model, which quantifies hiddenness.
            R: Measurement noise.
        """
        # Kalman Gain
        S = H @ P @ H.T + R
        K = P @ H.T @ linalg.inv(S)

        # Residual, represents data that was lost
        y = z - H @ x

        # update state and filter covariance
        ky = K @ y
        x += ky
        P = P - K @ H @ P
        return x, P

    def make_white_noise(self, dt, var):
        """
        Returns a white noise model Q.
        """

        return np.array(
            [
                [(0.5 * var * dt**2) ** 2, 0.5 * var * dt**2],
                [0.5 * var * dt**2, var * dt**2],
            ]
        )

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

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion. Shoutout AutomaticAddison !!!
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def normalize_angle(self, angle):
        """
        Normalize an angle on the range of 0 to 2pi.
        """
        if abs(angle) > 2 * math.pi:
            angle = angle % 2 * math.pi

        if angle < 0:
            angle = angle + 360

        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ExtendedKalmanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
