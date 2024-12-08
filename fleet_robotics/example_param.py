import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class TestROSParamsNode(Node):
    def __init__(self):
        super.__init__("test_params")

        # declare a parameter to access it
        self.declare_parameter("num_robots", rclpy.Parameter.Type.INTEGER)
        self.num_robots = (
            self.get_parameter("num_robots").get_parameter_value().integer_value
        )

        # add as many subscribers as is necessary
        subscriber_list = (
            []
        )  # this is kind of sketchy since the subs are not attributes
        for num in range(0, self.num_robots):
            # note that subscribers all have the same callback
            subscriber_list.append(
                self.create_subscription(
                    PoseStamped,
                    f"/robot{num}/pose_estimate",
                    self.generic_pose_callback,
                    10,
                )
            )

    def generic_pose_callback(self, pose_msg: PoseStamped):
        # we should include in the message who the source robot is
        pass
