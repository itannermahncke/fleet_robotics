import rclpy
from rclpy.node import Node
from rclpy.time import Time
import time
import numpy as np
from std_msgs.msg import String, Empty
from builtin_interfaces.msg import Time as TimeMsg
from fleet_robotics_msgs.msg import TimeSourced


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
        super().__init__("network_startup")

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
        self.good_comms[self.robot_num - 1] = True
        self.good_comms_message = False
        self.robots_good_comms = [False] * self.num_robots
        self.robots_good_comms[self.robot_num - 1] = True
        self.robot_good_comms_message = False
        self.time_current = 0.0
        self.time_counter = 0

        # Test Publisher
        self.test_pub = self.create_publisher(String, "test_pub", 10)
        self.test_pub.publish(
            String(
                data=f"in init: {self.robot_name} {type(self.robot_num)} -- {self.num_robots}"
            )
        )

        self.speaker = self.create_publisher(String, "speak", 10)
        self.listner = self.create_publisher(String, "heard", 10)
        self.comm_check = self.create_publisher(String, "comm_check", 10)
        self.current_time = self.create_publisher(TimeSourced, "current_time", 10)
        self.start_nodes = self.create_publisher(Empty, "start_node", 10)

        for num in range(1, self.num_robots + 1):
            if self.robot_num == 1 and num == 1:
                self.timer_starter = self.create_publisher(String, "start_timer", 10)
            elif self.robot_num == 1 and num >= 1:
                self.test_pub.publish(
                    String(data=f"{self.robot_name} creating comm check subscription")
                )
                self.create_subscription(
                    String, f"/robot{num}/comm_check", self.comm_check_callback, 10
                )
            if num == self.robot_num:
                self.test_pub.publish(
                    String(data=f"{self.robot_name} passed num == own num")
                )
            else:
                self.test_pub.publish(
                    String(data=f"{self.robot_name} creating subscriptions to topics")
                )
                self.create_subscription(
                    String, f"/robot{num}/speak", self.speak_callback, 10
                )
                self.create_subscription(
                    String, f"/robot{num}/heard", self.heard_callback, 10
                )
                self.create_subscription(
                    String, "robot1/start_timer", self.start_timer_callback, 10
                )

        timer_period = 0.2
        self.run_timer = self.create_timer(timer_period, self.run)
        time.sleep(3)
        self.speaker.publish(String(data=f"robot {self.robot_num} speaking"))

    def run(self):

        if all(x for x in self.good_comms) and self.good_comms_message == False:
            self.test_pub.publish(
                String(data=f"{self.robot_name} passed first run loop if statement")
            )
            # now check with all other robots to see if they all have good comms, this action is only taken by robot 1
            self.good_comms_message = True
            self.comm_check.publish(
                String(data=f"robot {self.robot_num} comms are good")
            )

        if self.robot_num == 1 and self.good_comms_message == True:
            # self.test_pub.publish(
            #     String(
            #         data=f"passed robot_num == 1 -- {self.robots_good_comms} -- {self.robot_good_comms_message}"
            #     )
            # )

            if (
                all(x for x in self.robots_good_comms)
                and self.robot_good_comms_message == False
            ):
                # now move on to timers
                self.test_pub.publish(
                    String(data="passed robot_good_comms_message == False")
                )
                self.robot_good_comms_message = True
                self.timer_starter.publish(String(data="Start"))

    def speak_callback(self, msg: String):
        self.test_pub.publish(String(data=f"in speak_callback: {msg.data}"))
        self.listner.publish(
            String(data=f"robot {self.robot_num} heard robot {msg.data[6]}")
        )

    def heard_callback(self, msg: String):
        self.test_pub.publish(String(data=f"in heard_callback: {msg.data}"))
        good_comm_robot = int(msg.data[6]) - 1
        self.good_comms[good_comm_robot] = True

    def comm_check_callback(self, msg: String):

        robot_good_comm = int(msg.data[6]) - 1

        self.robots_good_comms[robot_good_comm] = True
        self.test_pub.publish(
            String(
                data=f"in comm_check_callback: {msg.data} -- {self.robots_good_comms}"
            )
        )

    def start_timer_callback(self, msg: String):
        self.test_pub.publish(String(data=msg.data))
        if msg.data == "Start":
            # start timers
            self.start_nodes.publish(Empty())
            self.timer = self.create_timer(0.05, self.current_time_callback)
            x = self.get_clock().now().seconds_nanoseconds()
            self.start_time = (x[0] * (10**9)) + x[1]

    def current_time_callback(self):
        x = self.get_clock().now().seconds_nanoseconds()
        self.time_current = ((x[0] * (10**9)) + x[1]) - self.start_time
        self.time_counter += 1
        send_time = TimeSourced()
        send_time._source_id = self.robot_name
        send_time.msg_id = str(self.time_counter)
        send_time.nanosec = int((self.time_current) / 10**3)
        self.test_pub.publish(String(data=f"current time {send_time.nanosec}"))
        self.current_time.publish(send_time)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkStartupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
