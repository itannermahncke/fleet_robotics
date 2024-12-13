import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')

        # # access number of robots
        # self.declare_parameter("num_robots", rclpy.Parameter.Type.INTEGER)
        # self.num_robots = (
        #     self.get_parameter("num_robots").get_parameter_value().integer_value
        # )

        # # access this robot's name
        # self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        # self.robot_name = (
        #     self.get_parameter("robot_name").get_parameter_value().string_value
        # )
        # self.robot_num = int(self.robot_name[-1])


        self.current_pose = None  # Current robot pose (x, y, theta)
        self.goal_pose = None  # Goal pose (x, y)
        self.obstacle_data = None  # LiDAR obstacle data
        self.goal_received = False #initialize path planning only when this is true
        # Parameters
        self.step_size = 0.2  # Distance to move per step
        self.threshold_to_goal = 0.05  # Distance threshold to consider goal reached
        # self.obstacle_threshold = 0.4  # Distance threshold for obstacles

        self.linear_vel = 0.2
        self.angular_vel = 0.2

        self.turning = False
        self.turn_timer = None
        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.current_pose_callback, 10)
        # self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publishers
        self.next_pose_publisher = self.create_publisher(PoseStamped, '/next_pose', 10)
        self.heading_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.5, self.timer_callback)


    def goal_pose_callback(self, goal_msg : PoseStamped):
        """Callback to receive the goal pose."""
        self.goal_pose = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.goal_received = True 

    def current_pose_callback(self, pose_msg: PoseStamped):
        """Callback to receive the current pose."""
        self.current_pose = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            self.heading_to_global(pose_msg.pose.orientation)
        )


    def timer_callback(self):
        """Main planning loop triggered by the timer."""
        if not self.goal_received or self.goal_pose is None or self.current_pose is None or self.obstacle_data is None:
            return

        if self.turning:
            # If currently turning, wait for turn_timer to handle stopping
            return

        next_pose = self.plan_next_pose()

        if next_pose:

            angle_difference = next_pose[2] - self.current_pose[2]

            if abs(angle_difference) > 0.01:  # If there's an angle difference to adjust
                self.start_turn(angle_difference)
            else:
                # Move forward after adjusting angle
                twist_msg = Twist()
                twist_msg.linear.x = self.linear_vel
                self.heading_publisher.publish(twist_msg)


    def start_turn(self, angle_difference):
        """Initiate a turning motion."""
        turn_time = abs(angle_difference / self.angular_vel)

        # Set the angular velocity direction
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_vel if angle_difference > 0 else -self.angular_vel
        self.heading_publisher.publish(twist_msg)

        # Start a timer for the turn
        self.turning = True
        self.turn_timer = self.create_timer(turn_time, self.stop_turn)


    def stop_turn(self):
        """Stop the turning motion."""
        # Stop the timer
        if self.turn_timer:
            self.turn_timer.cancel()
            self.turn_timer = None

        # Stop the angular velocity
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        self.heading_publisher.publish(twist_msg)

        # Transition back to regular motion
        self.turning = False


    def plan_next_pose(self):
        """Path planning logic."""
        current_x, current_y, current_theta = self.current_pose
        goal_x, goal_y = self.goal_pose

        # Check if the goal is reached
        if math.dist((current_x, current_y), self.goal_pose) < self.threshold_to_goal:
            self.get_logger().info('Goal reached!')
            return None

        # Compute direction vector toward the goal
        vector_to_goal = np.array([goal_x - current_x, goal_y - current_y])
        norm_vector = vector_to_goal / np.linalg.norm(vector_to_goal)
        next_x = current_x + self.step_size * norm_vector[0]
        next_y = current_y + self.step_size * norm_vector[1]
        heading = math.atan2(norm_vector[1], norm_vector[0])

        return (next_x, next_y, heading)


def main(args=None):

    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
