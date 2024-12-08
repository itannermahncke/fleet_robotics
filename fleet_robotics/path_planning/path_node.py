#subscribe and publish stuff. calling functions and main loop

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PathPlanningNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('path_planning_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(PoseStamped, 'my_point', 10)

    def run_loop(self):
        """Prints a message to the terminal."""
        print('Hi from in_class_day02.')


class ReceiveMessageNode(Node):
    """This is a message publishing node, which inherits from the rclpy Node class."""
    def __init__(self):
        """Initializes the SendMessageNode. No inputs."""
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(PoseStamped, 'my_point', 10)

    def run_loop(self):
        """Prints a message to the terminal."""
        print('Hi from in_class_day02.')

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = PathPlanningNode()   # Create our Node
    node = ReceiveMessageNode()
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup
  

if __name__ == '__main__':
    main()
