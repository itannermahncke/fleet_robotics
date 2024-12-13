import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SendMessageNode(Node):
    def __init__(self):
        super().__init__("send_message_node")
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def run_loop(self):
        vel = Twist()
        vel.linear.x = 0.2
        vel.angular.z = 0.1
        self.publisher.publish(vel)
        print(vel)


def main(args=None):
    rclpy.init(args=args)  # Initialize communication with ROS
    node = SendMessageNode()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
