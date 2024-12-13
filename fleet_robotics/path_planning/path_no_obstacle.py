import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  

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

        self.current_pose = None   # Current robot pose (x, y, theta)world frame
        self.goal_pose = None  # Goal pose (x, y) in world frame
        self.goal_received = False #initialize path planning only when this is true
        self.grid_size = 0.4
        self.map_width = 2.25
        self.map_height = 5
        # Parameters
        # self.step_size = 0.2  # Distance to move per step
        self.threshold_to_goal = 0.05  # Distance threshold to consider goal reached
        # self.obstacle_threshold = 0.4  # Distance threshold for obstacles

        # self.linear_vel = 0.2
        # self.angular_vel = 0.2

        # self.turning = False
        # self.turn_timer = None
        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.create_subscription(PoseStamped, '/current_pose', self.current_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Publishers
        self.next_pose_publisher = self.create_publisher(PoseStamped, '/next_pose', 10)
        # self.heading_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.5, self.timer_callback)

    def translate_world_to_discrete(self, world_pose):
        discrete_coord_x = math.floor(world_pose[0] / self.grid_size)
        discrete_coord_y = math.floor(world_pose[1] / self.grid_size)
        return [discrete_coord_x, discrete_coord_y]


    def translate_discrete_to_world(self, discrete_pose):

        world_pose_x = discrete_pose[0]*self.grid_size + (self.grid_size/2)
        world_pose_y = discrete_pose[1]*self.grid_size + (self.grid_size/2)
        return [world_pose_x, world_pose_y]

    def goal_pose_callback(self, goal_msg : PoseStamped):
        """Callback to receive the goal pose."""
        self.goal_pose = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.goal_received = True 

    def current_pose_callback(self, pose_msg: PoseStamped):
        """Callback to receive the current pose."""
        self.current_pose = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            # self.heading_to_global(pose_msg.pose.orientation)
        )
    def odom_callback(self, odom_msg: Odometry):
        """Callback to process odometry data."""
        position = odom_msg.pose.pose.position
        # orientation = odom_msg.pose.pose.orientation

        # Extract (x, y) from position
        x = position.x
        y = position.y

        # Convert quaternion to Euler angles to extract theta (yaw)
        # siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        # cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        # theta = math.atan2(siny_cosp, cosy_cosp)

        # Update current pose
        self.current_pose = (x, y)

        
    def generic_callback(self):
        if not self.goal_received or self.current_pose is None or self.goal_pose is None:
            return

        # Translate poses and calculate next step
        discrete_current = self.translate_world_to_discrete(self.current_pose)
        discrete_goal = self.translate_world_to_discrete(self.goal_pose)
        self.goal_pose = discrete_goal
        next_pose_discrete = self.plan_next_pose(discrete_current)

        # Publish the next pose
        if next_pose_discrete:
            world_next_pose = self.translate_discrete_to_world(next_pose_discrete)
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = world_next_pose[0]
            pose_msg.pose.position.y = world_next_pose[1]
            self.next_pose_publisher.publish(pose_msg)

    def plan_next_pose(self, current_pose_discrete):
        """Path planning logic."""

        # Check if the goal is reached
        if math.dist(self.current_pose, self.goal_pose) < self.threshold_to_goal:
            self.get_logger().info('Goal!')
            return None
        grids_around = []
        grids_around_add = [(-1,1), (0,1),(1,1),(-1,0),(1,0),(-1,-1),(0,-1),(1,-1)]
        for grids in grids_around_add:
            result = tuple(x + y for x, y in zip(current_pose_discrete, grids))
            grids_around.append(result)

        min_distance = float("inf")
        next_pose_discrete = None
        for grid in grids_around:
            distance = abs(self.goal_pose[0]- grid[0])+abs(self.goal_pose[1]- grid[1])
            if distance < min_distance:
                min_distance = distance
                next_pose_discrete = grid
        return next_pose_discrete
            



def main(args=None):

    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
