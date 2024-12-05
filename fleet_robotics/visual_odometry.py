import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from cv_bridge import CvBridge
from collections import deque

from geometry_msgs.msg import Transform, Pose
from sensor_msgs.msg import Image, CompressedImage

from typing import List

class VisualOdometryNode(Node):
    """ 
    The neato will take an image at each set timestep. With the images, we will perform visual
    odometry with the main step: feature extration, feature matching, and estimate motion. (estimate trajectory?)

    Each image is appended to a deque and processed in the img_callback function. 
    
    Output:
    - rotation, translation vector
        - These vectors associated with each index of the taken images will be appended ot the translation and rotation list.
    - transformation matrix (from previous pose) 
    - published pose

    Resources
    - https://github.com/alishobeiri/Monocular-Video-Odometery
    """

    def __init__(self):
        """ """
        super().__init__("visual_odom")

        # attributes
        self.update_rate = 0.01  # sec

        # Neato Camera Calibration Matrix
        self.k = np.array([[500.68763, 0.0, 378.6717,],
              [0.0, 501.1204 , 207.83452,],
              [0.0, 0.0, 1.0]], dtype=np.float32)
        self.focal = (self.k[0][0] + self.k[1][1])/2
        self.pp = (self.k[0][2], self.k[1][2])

        # Pose
        self.current_pose = [np.eye(4)]
        self.latest_pose = None
        # Pose as a 4x4 matrix - extract pose attributes
        self.pose_history: List[Pose] = [np.eye(4)] # need to set inital transform to inital pose

        # MVO attrivutes
        self.kp_list = [] #deque(maxlen=30)
        self.des_list = [] #deque(maxlen=30)

        # Transform
        self.pose_transform_history = [np.eye(4)] # set inital transform to none
        self.current_transform = None
        self.combined_transform = np.eye(4) # all transforms     

        # Publishers and Subscribers
        self.transform_timer = self.create_timer(
            self.update_rate, self.publish_visual_pose
        )

        self.image_sub = self.create_subscription(Image, "/image_raw", self.img_callback, 10)
        self.image_deque = deque(maxlen=30)

        self.pose_pub = self.create_publisher(Pose, "visual_pose", 10)

    def update_pose(self):
        """
        Get current pose by applying the transformation matrix to the latest pose
        """   
        self.latest_pose = self.current_pose
        self.current_pose = self.pose_transform_history[-1] @ self.latest_pose
        self.pose_history.append(self.current_pose)        

    def img_callback(self, img_msg: Image):
        """
        When a new image is received, add it to the image deque and process it
        """
        # Append image to deque
        self.image_deque.append(img_msg)

        # Convert the ROS image message to an OpenCV image
        cv_image = CvBridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        # Extract Features
        kp, des = self.extract_features(cv_image)
        self.kp_list.append(kp)
        self.des_list.append(des)

        # Feature Matching
        if self.image_deque is None or len(self.image_deque) == 1: # do nothing for first image
            pass
        else:
            filtered_matches = self.match_features(self.des_list[-2], self.des_list[-1], dist_threshold=0.6)

        # Estimate Motion
        rmat, tvec, _, _ = self.estimate_motion(filtered_matches, self.kp_list[-2], self.kp_list[-1])

        # Create a 4x4 homogeneous transformation matrix
        self.current_transform = np.eye(4)  # Reset with an identity matrix
        self.current_transform[:3, :3] = rmat  # Insert the rotation matrix
        self.current_transform[:3, 3] = tvec.flatten()  # Insert the translation vector
        self.pose_transform_history.append(self.current_transform)

    def update_transform(self):
        """
        Calculates the transformation matrix from initlized point in the robot odom to current pose

        Output:
        - full_transform: a 4x4 transformation matrix from initial pose to current pose
        """
        # prevent live deque updates from interrupting odom calculations
        # snapshot = list(self.image_deque)

        full_transform = np.eye(4)

        for transform in self.pose_transform_history:
            full_transform = np.dot(self.combined_transform, transform)
        
        return full_transform
        
    def publish_visual_pose(self):
        """
        Publish current pose
        """
        self.update_pose()

        # convert roation to quaternion
        current_rotation = self.current_pose[:3, :3]
        quaternion = Rotation.from_matrix(current_rotation).as_quat() 

        # Create and populate the Pose message
        pose = Pose()

        # Assign position from translation matrix
        pose.position.x = self.current_pose[1][3]
        pose.position.y = self.current_pose[2][3]
        pose.position.z = self.current_pose[3][3]

        # Assign orientation from translation matrix
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose)


    # HELPER FUNCTIONS
    def extract_features(self, image):
        """
        Find keypoints and descriptors for each image in the dataset

        Arguments:
        image -- an image

        Returns:
        kp -- a list of keypoints for the image
        des -- a list of descriptors for the image
        """
        # Create SIFT object
        sift = cv2.xfeatures2d.SIFT_create()
        kp, des = sift.detectAndCompute(image, None)
        return kp, des

    def match_features(des1, des2, dist_threshold=0.6):
        """
        Match features for each subsequent image pair in the dataset

        Arguments:
        des_list -- a list of descriptors for each image in the dataset
        dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0) 

        Returns:
        matches_list -- list of matches for each subsequent image pair in the dataset. 
                Each matches[i] is a list of matched features from images i and i + 1
                
        """
        matches = []
        filtered_matches = []

        # Brute Force matching
        bf = cv2.BFMatcher()
        # BFMatcher.knnMatch() to get k best matches. In this example, we will take k=2 so that we can apply ratio test explained by D.Lowe in his paper.
        matches = bf.knnMatch(des1, des2, k=2)

        # Filter matches by distance
        for m,n in matches:
            if m.distance < (dist_threshold * n.distance):
                filtered_matches.append([m])

        return filtered_matches

    def estimate_motion(self, matches, kp1, kp2):
        """
        Estimate camera motion from a pair of subsequent image frames

        Arguments:
        matches -- list of matched features from the pair of images
        kp1 -- list of the keypoints in the first image
        kp2 -- list of the keypoints in the second image
        
        Optional arguments:
        depth1 -- a depth map of the first frame. This argument is not needed if you use Essential Matrix Decomposition

        Returns:
        rmat -- recovered 3x3 rotation numpy matrix
        tvec -- recovered 3x1 translation numpy vector                
        """
        # rmat = np.eye(3)
        # tvec = np.zeros((3, 1))
        # img1_points = []
        # img2_points = []

        # queryIdx: Index of the keypoint in the first image.
        # trainIdx: Index of the keypoint in the second image.
        img1_points = np.array([kp1[m.queryIdx].pt for m in matches])
        img2_points = np.array([kp2[m.trainIdx].pt for m in matches])

        E, _ = cv2.findEssentialMat(img2_points, img1_points, self.focal, self.pp, cv2.RANSAC, 0.999, 1.0, None)
        _, rmat, tvec, _ = cv2.recoverPose(E, img1_points, img2_points, focal=self.focal, pp=self.pp, mask=None)

        return rmat, tvec
    
        

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
