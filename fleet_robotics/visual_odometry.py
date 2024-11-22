import rclpy
from rclpy.node import Node

from collections import deque

from geometry_msgs.msg import Transform, Pose
from sensor_msgs.msg import Image, CompressedImage


class VisualOdometryNode(Node):
    """ """

    def __init__(self):
        """ """
        super.__init__("visual_odom")

        # attributes
        self.update_rate = 0.01  # sec
        self.latest_pose = None

        # manage incoming images
        self.image_sub = self.create_subscription("/image_raw", self.img_callback, 10)
        self.image_deque = deque(maxlen=30)

        # pose and transformation management
        self.transform_timer = self.create_timer(
            self.update_rate, self.update_transform
        )
        self.transform_pub = self.create_publisher(Transform, "visual_tf", 10)
        self.latest_pose_sub = self.create_subscription(
            Pose, "pose_estimate", self.update_pose, 10
        )

    def update_pose(self, pose_msg: Pose):
        """
        Update the latest robot pose estimate.
        """
        self.latest_pose = pose_msg

    def img_callback(self, img_msg: Image):
        """
        When a new image is received, add it to the image deque.
        """
        self.image_deque.append(img_msg)

    def update_transform(self):
        """
        Every [self.update_rate] sec, take a snapshot of the current image
        deque and perform visual odometry on the dataset. Then, publish the
        transformation between the latest pose and the current pose.

        Ivy note: I believe that Vivian's visual odom code provides a
        transformation from the earliest image in the dataset to the latest
        image (and/or possibly the tf between each sequential image). However,
        we need to return the transformation between the self.latest_pose
        and the latest image. This will require some thinking related to:
        - associating pose estimates with particular images
        - guranteeing the earliest deque image is from the latest pose
        - the rates at which we gather new images and make pose estimates
        """
        # prevent live deque updates from interrupting odom calculations
        snapshot = list(self.image_deque)


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
