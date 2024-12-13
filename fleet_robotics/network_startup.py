import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NetworkStartupNode(Node):
    """
    This node verifies the integrity of the Neato network by confirming that
    the local Neato can talk and listen to every other Neato. This node also
    confirms that all Neatos agree on what time it is. Once the integrity of
    the network has been verified, path planning can begin.
    """

    def __init__(self):
        """
        Initialize an instance of the MotionExecutionNode.
        """
        super.__init__("network_startup")

        # robots go in order and send a message, then receive confirmation that
        # it was heard.
        # Robot 1: HELLO
        # Robot 2: HELLO
        # Robot 3: HELLO
        # Robot 1: HEARD 2
        # Robot 2: HEARD 2
        # Robot 3: HEARD 2
        # if anything is wrong here, the user should take the Neatos down
        # then the neatos must agree on the current time
        # Robot 1: START TIMERS
        # Robot 1: [starts timer]
        # Robot 2: [starts timer]
        # Robot 3: [starts timer]
        # Robot 1: MY TIME AFTER 1 CALLBACK
        # Robot 2: MY TIME AFTER 1 CALLBACK
        # Robot 3: MY TIME AFTER 1 CALLBACK

        # then the neatos must take note of everyone else's offset relative to them
        # Robot 1: [robot2_offset, robot3_offset]
        # Robot 2: [robot1_offset, robot3_offset]
        # Robot 3: [robot1_offset, robot2_offset]

        # Getting the number of robots
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
        self.messages = list(range(self.num_robots))
        # Creating publishers and subscribers for the robots
        self.create_publisher(String, "send_msg", self.send_msg_callback, 10)
        for num in range(0, self.num_robots):
            if num == self.robot_num:
                continue
            else:
                self.create_subscription(
                    String, f"/robot{num}/receive_msg", self.receive_msg_callback, 10
                )

        for num in range(0, self.num_robots):
            if num == self.robot_num:
                self.send_message()
            else:
                self.confirm_receive()

    def send_message(self):
        pass

    def confirm_receive(self):
        pass

    def send_msg_callback(self, msg: String):
        pass

    def receive_msg_callback(self, msg: String):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = NetworkStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
