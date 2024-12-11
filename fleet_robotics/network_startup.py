import rclpy
from rclpy.node import Node


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


def main(args=None):
    rclpy.init(args=args)
    node = NetworkStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
