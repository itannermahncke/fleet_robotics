"""
Converting my jupyter notebook into a python script.
Contains a class to handle the input of images from the neato:
    - turn images to rgb
    - turn images to grayscale
    - create a depth map for each image         [UNSURE HOW TO DO YET]
    - extract features
    - match features & filter by distance
    - estimate motion                           [NEED DEPTH MAP]
    - STRETCH GOAL: estimate trajecgtory 
"""

import os

import math 
import numpy as np
import cv2

from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D


class NeatoFleetDatatsetHander:
    def __init__(self):
        # Only storing 20 frames at a time
        self.num_frames = 20

        # Set up paths to images
        root_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.image_dir = os.path.join(root_dir_path, 'data/rgb')
        self.depth_dir = os.path.join(root_dir_path, 'data/depth')

        # Set up image lists
        self.images_gray = []
        self.images_rgb = []
        self.depth_maps = []


        # Neato Camera Calibration Matrix
        self.k = np.array([[500.68763, 0.0, 378.6717,],
              [0.0, 501.1204 , 207.83452,],
              [0.0, 0.0, 1.0]], dtype=np.float32)

        # Read first frame
        self.read_frame()
        print("\r" + ' '*20 + "\r", end='')
        
    def read_frame(self):
        """
        Call functions to read the images and depth maps
        """
        self._read_depth()
        self._read_image()
              
    def _read_image(self):
        """
        Reading image
        """
        for i in range(1, self.num_frames + 1):
            zeroes = "0" * (5 - len(str(i)))
            im_name = "{0}/frame_{1}{2}.png".format(self.image_dir, zeroes, str(i))
            self.images.append(cv.imread(im_name, flags=0))
            self.images_rgb.append(cv.imread(im_name)[:, :, ::-1])
            print ("Data loading: {0}%".format(int((i + self.num_frames) / (self.num_frames * 2 - 1) * 100)), end="\r")
            
       
    def _read_depth(self):
        """
        Reading depth map
        """
        for i in range(1, self.num_frames + 1):
            zeroes = "0" * (5 - len(str(i)))
            depth_name = "{0}/frame_{1}{2}.dat".format(self.depth_dir, zeroes, str(i))
            depth = np.loadtxt(
                depth_name,
                delimiter=',',
                dtype=np.float64) * 1000.0
            self.depth_maps.append(depth)
            print ("Data loading: {0}%".format(int(i / (self.num_frames * 2 - 1) * 100)), end="\r")

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

        # Filter through matches
        for m,n in matches:
            if m.distance < (dist_threshold * n.distance):
                filtered_matches.append([m])

        return filtered_matches
    
    def estimate_motion(match, kp1, kp2, k, depth1):
        """
        Estimate camera motion from a pair of subsequent image frames

        Arguments:
        match -- list of matched features from the pair of images
        kp1 -- list of the keypoints in the first image
        kp2 -- list of the keypoints in the second image
        k -- camera calibration matrix 
        
        Optional arguments:
        depth1 -- a depth map of the first frame. This argument is not needed if you use Essential Matrix Decomposition

        Returns:
        rmat -- recovered 3x3 rotation numpy matrix
        tvec -- recovered 3x1 translation numpy vector
        image1_points -- a list of selected match coordinates in the first image. image1_points[i] = [u, v], where u and v are 
                        coordinates of the i-th match in the image coordinate system
        image2_points -- a list of selected match coordinates in the second image. image1_points[i] = [u, v], where u and v are 
                        coordinates of the i-th match in the image coordinate system
                
        """
        rmat = np.eye(3)
        tvec = np.zeros((3, 1))
        image1_points = []
        image2_points = []

        return rmat, tvec, image1_points, image2_points
        pass


    def estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps):
        pass


def visualize_camera_movement(image1, image1_points, image2, image2_points, is_show_img_after_move=False):
    """
    Draw circles around extracted features between two subsequent images.
    Red and green circles indicates previous and current frame, respectively.
    Arrows showing the movement from image1 to image2.

    Args:
        - image1 -- the first image in a matched image pair
        - kp1 -- list of the keypoints in the first image
        - image2 -- the second image in a matched image pair
        - kp2 -- list of the keypoints in the second image
        - match -- list of matched features from the pair of images
        - is_show_img_after_move -- boolean to show features of image2
    Returns:
        - image1 -- the first image with circles drawn around matched features
        - image2 -- the second image with circles drawn around matched features
    """
    image1 = image1.copy()
    image2 = image2.copy()
    
    for i in range(0, len(image1_points)):
        # Coordinates of a point on t frame
        p1 = (int(image1_points[i][0]), int(image1_points[i][1]))
        # Coordinates of the same point on t+1 frame
        p2 = (int(image2_points[i][0]), int(image2_points[i][1]))

        cv2.circle(image1, p1, 5, (0, 255, 0), 1)
        cv2.arrowedLine(image1, p1, p2, (0, 255, 0), 1)
        cv2.circle(image1, p2, 5, (255, 0, 0), 1)

        if is_show_img_after_move:
            cv2.circle(image2, p2, 5, (255, 0, 0), 1)
    
    if is_show_img_after_move: 
        return image2
    else:
        return image1
    







