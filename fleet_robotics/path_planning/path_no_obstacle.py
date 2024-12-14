import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped

from fleet_robotics_msgs.msg import PoseStampedSourced


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__("path_planning")

        # robot info and status
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value
        )
        self.current_pose = None  # Current robot pose (x, y, theta)world frame

        # goal status
        self.declare_parameter(f"{self.robot_name}_goal", rclpy.Parameter.Type.STRING)
        self.goal_pose = (
            self.get_parameter(f"{self.robot_name}_goal")
            .get_parameter_value()
            .string_value
        )  # goal pose (x, y) in world frame

        # map status
        self.grid_size = 0.4
        self.map_width = 2.25
        self.map_height = 5

        # Parameters
        self.threshold_to_goal = 0.05  # Distance threshold to consider goal reached

        # Subscribers
        self.create_subscription(
            PoseStamped, "pose_estimate", self.current_pose_callback, 10
        )
        self.create_subscription(Bool, "step_status", self.new_step_callback, 10)

        # Publishers
        self.next_pose_publisher = self.create_publisher(
            PoseStampedSourced, "next_step", 10
        )
        self.msg_id_counter = 1

    def translate_world_to_discrete(self, world_pose: Pose):
        """
        Given a pose in the world, return the square in the discrete world
        that contains this pose.
        """
        discrete_coord_x = math.floor(world_pose.position.x / self.grid_size)
        discrete_coord_y = math.floor(world_pose.position.y / self.grid_size)
        return [discrete_coord_x, discrete_coord_y]

    def translate_discrete_to_world(self, discrete_pose: tuple):
        """
        Given a pose in the discrete world, return that same pose in the real
        (cartesian) world. This function will always return the world pose as
        the center of the square that contains it in the discrete world.
        """
        world_pose_x = discrete_pose[0] * self.grid_size + (self.grid_size / 2)
        world_pose_y = discrete_pose[1] * self.grid_size + (self.grid_size / 2)
        return [world_pose_x, world_pose_y]

    def current_pose_callback(self, pose_msg: PoseStamped):
        """
        Callback to receive the goal pose. This node is only interested in its
        xy coordinates, not its heading.
        """
        self.current_pose = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
        )

    def new_step_callback(self):
        """
        Callback function that occurs when the motion_execution node is ready
        to receive its next step.
        """
        if self.current_pose is None or self.goal_pose is None:
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

    def send_next_step(self, next_step: tuple):
        """
        When the path planner has decided what its next step will be, package
        and send this step to the Neato network.
        """
        pose_msg = PoseStampedSourced()

        # assign proper world coordinates
        world_next_pose = self.translate_discrete_to_world(next_step)
        pose_msg.pose.position.x = world_next_pose[0]
        pose_msg.pose.position.y = world_next_pose[1]

        # assign a unique message identifier
        pose_msg.msg_id = self.msg_id_counter
        self.msg_id_counter += 1

        # tag with the source robot
        pose_msg.source_id = self.robot_name

        # publish to Neato network
        self.next_pose_publisher.publish(pose_msg)

    def plan_next_pose(self, current_pose_discrete):
        """
        Path planning logic. Live A* algorithm.
        """

        # Check if the goal is reached
        if math.dist(self.current_pose, self.goal_pose) < self.threshold_to_goal:
            self.get_logger().info("Goal!")
            return None
        grids_around = []
        grids_around_add = [
            (-1, 1),
            (0, 1),
            (1, 1),
            (-1, 0),
            (1, 0),
            (-1, -1),
            (0, -1),
            (1, -1),
        ]
        for grids in grids_around_add:
            result = tuple(x + y for x, y in zip(current_pose_discrete, grids))
            grids_around.append(result)

        min_distance = float("inf")
        next_pose_discrete = None
        for grid in grids_around:
            distance = abs(self.goal_pose[0] - grid[0]) + abs(
                self.goal_pose[1] - grid[1]
            )
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


if __name__ == "__main__":
    main()
