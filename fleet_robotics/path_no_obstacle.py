import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty

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

        # map state
        self.declare_parameter("square_size", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("grid_width", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("grid_height", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("obstacle_x", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("obstacle_y", rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.grid_size = (
            self.get_parameter("square_size").get_parameter_value().double_value
        )
        self.grid_width = (
            self.get_parameter("grid_width").get_parameter_value().integer_value
        )
        self.grid_height = (
            self.get_parameter("grid_height").get_parameter_value().integer_value
        )

        self.obstacle_pose_x = (
            self.get_parameter("obstacle_x").get_parameter_value().double_array_value
        )

        self.obstacle_pose_y = (
            self.get_parameter("obstacle_y").get_parameter_value().double_array_value
        )

        self.obstacle_list = []
        for index, __ in enumerate(self.obstacle_pose_x):
            self.obstacle_list.append(
                (self.obstacle_pose_x[index], self.obstacle_pose_y[index])
            )
        self.obstacle_discrete = []
        for obstacle in self.obstacle_list:
            self.obstacle_discrete.append(self.translate_world_to_discrete(obstacle))

        # goal status
        self.declare_parameter(
            f"{self.robot_name}_goal", rclpy.Parameter.Type.DOUBLE_ARRAY
        )
        self.goal_pose = (
            self.get_parameter(f"{self.robot_name}_goal")
            .get_parameter_value()
            .double_array_value
        )  # goal pose (x, y) in world frame
        self.discrete_goal = self.translate_world_to_discrete(self.goal_pose)

        # Subscribers
        self.create_subscription(
            PoseStampedSourced, "pose_estimate", self.current_pose_callback, 10
        )
        self.create_subscription(Empty, "start_node", self.new_step_callback, 10)
        self.create_subscription(Bool, "step_status", self.new_step_callback, 10)

        # Publishers
        self.next_pose_publisher = self.create_publisher(
            PoseStampedSourced, "next_step", 10
        )
        self.msg_id_counter = 1

    def translate_world_to_discrete(self, world_pose: tuple):
        """
        Given a pose in the world, return the square in the discrete world
        that contains this pose.
        """
        discrete_coord_x = math.floor(world_pose[0] / self.grid_size)
        discrete_coord_y = math.floor(world_pose[1] / self.grid_size)
        return (discrete_coord_x, discrete_coord_y)

    def translate_discrete_to_world(self, discrete_pose: tuple):
        """
        Given a pose in the discrete world, return that same pose in the real
        (cartesian) world. This function will always return the world pose as
        the center of the square that contains it in the discrete world.
        """
        world_pose_x = discrete_pose[0] * self.grid_size + (self.grid_size / 2)
        world_pose_y = discrete_pose[1] * self.grid_size + (self.grid_size / 2)
        return (world_pose_x, world_pose_y)

    def current_pose_callback(self, pose_msg: PoseStampedSourced):
        """
        Callback to receive the goal pose. This node is only interested in its
        xy coordinates, not its heading.
        """
        self.current_pose = (
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
        )

    def new_step_callback(self, _):
        """
        Callback function that occurs when the motion_execution node is ready
        to receive its next step.
        """
        # Translate poses and calculate next step
        if self.current_pose is not None:
            discrete_current = self.translate_world_to_discrete(self.current_pose)

            if self.discrete_goal != discrete_current:
                next_pose_discrete = self.plan_next_pose(discrete_current)

                # Publish the next pose
                if next_pose_discrete:
                    self.send_next_step(next_pose_discrete)
            else:
                self.get_logger().info("Path planning finished!")

    def send_next_step(self, next_step: tuple):
        """
        When the path planner has decided what its next step will be, package
        and send this step to the Neato network.
        """
        pose_msg = PoseStampedSourced()

        # assign proper world coordinates
        self.get_logger().info(f"Sending next step: {next_step}")
        world_next_pose = self.translate_discrete_to_world(next_step)
        pose_msg.pose.position.x = world_next_pose[0]
        pose_msg.pose.position.y = world_next_pose[1]

        # assign a unique message identifier
        pose_msg.msg_id = str(self.msg_id_counter)
        self.msg_id_counter += 1

        # tag with the source robot
        pose_msg.source_id = self.robot_name

        # publish to Neato network
        self.get_logger().info(
            f"Next step at x: {pose_msg.pose.position.x} and y: {pose_msg.pose.position.y}"
        )
        self.next_pose_publisher.publish(pose_msg)

    def plan_next_pose(self, current_pose_discrete):
        """
        Path planning logic. Live A* algorithm.
        """
        grids_around = []
        grids_around_add = [
            # (-1, 1),
            (0, 1),
            # (1, 1),
            (-1, 0),
            (1, 0),
            # (-1, -1),
            (0, -1),
            # (1, -1),
        ]
        for grids in grids_around_add:
            result = tuple(x + y for x, y in zip(current_pose_discrete, grids))
            grids_around.append(result)

        min_distance = float("inf")
        next_pose_discrete = None

        for grid in grids_around:
            if grid not in self.obstacle_discrete:
                distance = math.sqrt(
                    (self.discrete_goal[0] - grid[0]) ** 2
                    + (self.discrete_goal[1] - grid[1]) ** 2
                )
                if distance < min_distance:
                    self.get_logger().info(
                        f"Potential step {grid} has smallest distance of {distance}"
                    )
                    min_distance = distance
                    next_pose_discrete = grid
            else:
                self.get_logger().info(f"obstacle in way at {grid}")
        return next_pose_discrete


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
