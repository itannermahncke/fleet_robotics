import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation

from cv_bridge import CvBridge
from collections import deque

from geometry_msgs.msg import Transform, Pose
from sensor_msgs.msg import Image, CompressedImage

from fleet_robotics import visual_odometry_helper

from typing import List

class VisualOdometryNode(Node):
    """ 
    The neato will take an image at each set timestep. With the images, we will perform visual
    odometry with the main step: feature extration, feature matching, and estimating motion.

    Each image is appended to a deque and processed in the img_callback function. The output 
    of this function / visual odom is a translation and rotation vector. These vectors associated
    with each index of the taken images will be appended ot the translation and rotation list.

    Questions:
    - are we saving images to a folder? what is the location - im assuming just the list
    """

    def __init__(self):
        """ """
        super().__init__("visual_odom")

        self.dataset_handler = visual_odometry_helper.NeatoFleetDatasetHandler()

        # attributes
        self.update_rate = 0.01  # sec
        self.current_pose = None
        self.latest_pose = None
        # Pose as a matrix - extract if we need to
        self.pose_history: List[Pose] = [np.eye(4)] # need to set inital transform to inital pose

        self.kp_list = []
        self.des_list = []

        # Transform
        self.transform_history = [np.eye(4)] # set inital transform to none
        self.current_transform = None
        # From inital pose to current
        self.combined_transform = np.eye(4)

        
        

        # manage incoming images
        self.image_sub = self.create_subscription(Image, "/image_raw", self.img_callback, 10)
        self.image_deque = deque(maxlen=30)     # why deque?

        # pose and transformation management
        self.transform_timer = self.create_timer(
            self.update_rate, self.update_transform
        )
        self.transform_pub = self.create_publisher(Transform, "visual_tf", 10)

        # Publish the pose for sensor fusion
        self.pose_pub = self.create_publisher(Pose, "visual_pose", 10)

        self.latest_pose_sub = self.create_subscription(
            Pose, "pose_estimate", self.update_pose, 10
        )

    def update_pose(self, pose_msg: Pose):
        """
        Update the latest robot pose estimate.
        """
        # self.latest_pose = pose_msg

        # get previous pose
        self.latest_pose = self.pose_history[-1]

        # new pose = (R * current pose) + translation
        self.current_pose = self.transform_history[-1] @ self.latest_pose
        self.pose_history.append(self.current_pose)

    def img_callback(self, img_msg: Image):
        """
        When a new image is received, add it to the image deque and process it
        """
        # Append image to deque
        self.image_deque.append(img_msg)

        # Convert the ROS image message to an OpenCV image
        cv_image = CvBridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        kp, des = self.dataset_handler.extract_features(cv_image)
        self.kp_list.append(kp)
        self.des_list.append(des)

        # Feature matching
        if self.image_deque is None or len(self.image_deque) == 1:
            pass
        else:
            # check length of image list
            # compare most revent and previous one
            filtered_matches = self.dataset_handler.match_features(self.des_list[-1], self.des_list[-2])
        
        ##### Depth Map #####################################
        depth_map = []

        #####################################

        # Rotation and Translation
        rmat, tvec, _, _ = self.dataset_handler.estimate_motion(filtered_matches, self.kp_list[-2], self.kp_list[-1], depth_map)

        # Create a 4x4 homogeneous transformation matrix
        self.current_transform = np.eye(4)  # Reset with an identity matrix
        self.current_pose[:3, :3] = rmat  # Insert the rotation matrix
        self.current_transform[:3, 3] = tvec.flatten()  # Insert the translation vector
        self.transform_history.append(self.current_transform)

    def update_transform(self, num_frame):
        """
        ONLY CALL IF WE NEED TRANSFORMATION FROM FIRST IMAGE TO WHATEVER SEQUENCE
        
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

        self.combined_transform = np.eye(4)

        # ideally not restaritng every time
        for transform in self.transform_history[: num_frame]:
            self.combined_transform = np.dot(self.combined_transform, transform)
        
    def publish_visual_pose(self):
        """
        Publish current pose
        """
        self.current_pose

        current_rotation = self.current_pose[:3, :3]

        quaternion = Rotation.from_matrix(current_rotation).as_quat() #x,y,z,w

        # Create and populate the Pose message
        pose = Pose()

        # Assign position from NumPy array
        pose.position.x = self.current_pose[1][3]
        pose.position.y = self.current_pose[2][3]
        pose.position.z = self.current_pose[3][3]

        # Assign orientation from NumPy array
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose)
    
        

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
