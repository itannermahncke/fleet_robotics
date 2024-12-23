import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from scipy.spatial.transform import Rotation

from cv_bridge import CvBridge
from collections import deque

from geometry_msgs.msg import Transform, Pose, PoseStamped
from sensor_msgs.msg import Image, CompressedImage


from fleet_robotics_msgs.msg import PoseStampedSourced

from typing import List

import requests
import matplotlib.pyplot as plt

# Need to run this
# ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/image_raw -r out:=/camera/image/compressed
## TO DO LIST
# - make a flag for only updating poses when a new image is published
# - turn kp and des list into a deque


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

        # Neato Camera Calibration Matrix
        self.K = np.array(
            [
                [
                    500.68763,
                    0.0,
                    378.6717,
                ],
                [
                    0.0,
                    501.1204,
                    207.83452,
                ],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        self.FOCAL = (self.K[0][0] + self.K[1][1]) / 2
        self.PP = (self.K[0][2], self.K[1][2])

        # Transform to apply
        self.current_transform = np.eye(4)

        # Pose relative to odom 0,0
        self.posemat = np.eye(4) # Pose as a 4x4 matrix

        # MVO attrivutes
        self.kp_deque = deque(maxlen=20)  # deque(maxlen=30)
        self.des_deque = deque(maxlen=20)  # deque(maxlen=30)
        self.image_deque = deque(maxlen=20)

        self.t = False

        # Publishers and Subscribers
        self.UPDATE_RATE = 1  # sec

        self.transform_timer = self.create_timer(
            self.UPDATE_RATE, self.timing
        )

        # self.calibration_timer = self.create_timer(8, self.cam_calibration)

        # self.image_sub = self.create_subscription(
        #     Image, "camera/image_raw", self.testing_camera, 10
        # )
        self.image_sub = self.create_subscription(
            CompressedImage, "camera/image_raw/compressed", self.img_callback,10
        )
        self.pose_pub = self.create_publisher(PoseStampedSourced, "visual_pose", 10)

        # temp variables
        self.latest_pose = np.eye(4)
        self.pose_calibration = [np.eye(4)]

        self.tvec = None

    def cam_calibration(self):
        self.pose_calibration.append(self.posemat)
        print("THE CAMERA CALIBARTION LIST OF POSES is:")
        print(self.pose_calibration)
    

    def timing(self):
        self.t = True

    def img_callback(self, img_msg):
        """
        When a new image is received, add it to the image deque and process it
        """
        if self.t == True:
            print("TAKING AN IMAGE")

            # Turn image message into a CV image to process
            np_arr = np.frombuffer(img_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.image_deque.append(cv_image)
            # print(f"IAMGE DEQUE LENGTH IS {len(self.image_deque)}")

            # Extract Features
            kp, des = self.extract_features(cv_image)
            self.kp_deque.append(kp)
            self.des_deque.append(des)

            # print(f"Number of features detected in frame {len(self.image_deque)} is {len(kp)}")


            # Feature Matching
            if len(self.image_deque) > 1:       # check if there is more than one image
                filtered_matches = self.match_features(
                    np.asarray(self.des_deque[-2], np.float32), 
                    np.asarray(self.des_deque[-1], np.float32),
                )
                print(f"THE NUMBER OF FILTERED MATCHS IS {len(filtered_matches)}")

                # Estimate Motion
                rmat, tvec, _, _ = self.estimate_motion(
                    filtered_matches, 
                    self.kp_deque[-2],
                    self.kp_deque[-1],
                )

                # Create a 4x4 homogeneous transformation matrix
                self.current_transforma = np.eye(4)
                self.current_transform[:3, :3] = rmat  # Insert the rotation matrix
                self.current_transform[:3, 3] = tvec.flatten()  # Insert the translation vector

                # self.latest_pose = self.posemat
                # Update pose with transform
                # self.posemat = np.add(self.current_transform, self.posemat)

                self.posemat = self.current_transform @ self.posemat

                self.publish_visual_pose()
                # self.cam_calibration()

                self.tvec = tvec

                # print(f"TRANSLATION IS {tvec}")
                # print(f"ROTATION IS {rmat}")
                print(f"X IS {self.posemat[0][3]}")
                print("PUBLISHING A VISUAL POSE.....")
                # print(f"NEW POSE FORM ODOM 0,0 IS {self.posemat}")
                print(self.posemat)

            else:
                pass
            self.t = False
        else:
            pass


    def publish_visual_pose(self):
        """
        Publish current pose
        """
        rmat = self.posemat[:3, :3]
        quaternion = Rotation.from_matrix(rmat).as_quat()

        # Create and populate the Pose message
        pose_msg = PoseStampedSourced()

        # Assign position from translation matrix
        pose_msg.pose.position.x = self.posemat[0][3]
        pose_msg.pose.position.y = self.posemat[1][3]
        pose_msg.pose.position.z = self.posemat[2][3]

        # Assign orientation from translation matrix
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # print("PUBLISHING A VISUAL POSE")
        # print(self.posemat)

        self.pose_pub.publish(pose_msg)

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

    def match_features(self, des1, des2):
        """
        Match features for each subsequent image pair in the dataset

        Arguments:
        des_1

        Returns:
        matches_list -- list of matches for each subsequent image pair in the dataset.
                Each matches[i] is a list of matched features from images i and i + 1

        """
        dist_threshold=0.6
        matches = []
        filtered_matches = []

        # # Brute Force matching
        # bf = cv2.BFMatcher()
        # # BFMatcher.knnMatch() to get k best matches. In this example, we will take k=2 so that we can apply ratio test explained by D.Lowe in his paper.
        # matches = bf.knnMatch(des1, des2, k=2)

        # FLANN parameters.
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50) 
    
        # FLANN based matcher with implementation of k nearest neighbour.
        flann = cv2.FlannBasedMatcher(index_params,search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Filter matches by distance
        for m, n in matches:
            if m.distance < (dist_threshold * n.distance):
                filtered_matches.append([m])

        return filtered_matches

    def estimate_motion(self, match, kp1, kp2):
        """
        Estimate camera motion from a pair of subsequent image frames

        Arguments:
        matches -- list of matched features from the pair of images
        kp1 -- list of the keypoints in the first image
        kp2 -- list of the keypoints in the second image

        Returns:
        rmat -- recovered 3x3 rotation numpy matrix
        tvec -- recovered 3x1 translation numpy vector
        image1_points -- a list of selected match coordinates in the first image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
        image2_points -- a list of selected match coordinates in the second image. image1_points[i] = [u, v], where u and v are 
                     coordinates of the i-th match in the image coordinate system
        """
        img1_points = []
        img2_points = []

        img1_points = np.array([kp1[m[0].queryIdx].pt for m in match])
        img2_points = np.array([kp2[m[0].trainIdx].pt for m in match])

        E, _ = cv2.findEssentialMat(
            img2_points, img1_points, self.FOCAL, self.PP, cv2.RANSAC, 0.999, 1.0, None
        )
        _, rmat, tvec, _ = cv2.recoverPose(
            E, img1_points, img2_points, focal=self.FOCAL, pp=self.PP, mask=None
        )

        return rmat,tvec, img1_points, img2_points


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
