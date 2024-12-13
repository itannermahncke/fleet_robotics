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
        self.good_comms = [False] * self.num_robots

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run)
        # Robot 1 publishes to speak: 'robot 1 speaking'
        # Other robots subscribed to speak: run callback func
        #   other robots publish to heard: 'robot{num} heard robot 1
        # Robot 1 subscribed to heard: if all heard, move on
        self.speaker = self.create_publisher(String, "speak", 10)
        self.listner = self.create_publisher(String, "heard", 10)
        for num in range(0, self.num_robots):
            if num == self.robot_num:
                continue
            else:
                self.create_subscription(
                    String, f"/robot{num}/speak", self.speak_callback, 10
                )
                self.create_subscription(
                    String, f"/robot{num}/heard", self.heard_callback, 10
                )
        self.speaker.publish(String(data=f"robot {self.robot_num} speaking"))

    def run(self):
        if all(x for x in self.good_comms):
            pass  # go on to do timings, Then publish message

    def speak_callback(self, msg: String):
        self.listner.publish(
            String(data=f"robot {self.robot_num} heard robot {msg[6]}")
        )

    def heard_callback(self, msg: String):
        good_comm_robot = int(msg[6]) - 1
        self.good_comms[good_comm_robot] = True


def main(args=None):
    rclpy.init(args=args)
    node = NetworkStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
