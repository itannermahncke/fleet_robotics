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


        # Camera Calibration Matrix
        self.k = np.array([[640, 0, 640],
                           [0, 480, 480],
                           [0,   0,   1]], dtype=np.float32)

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
    


def visualize_trajectory(trajectory):
    pass

#####################################################
#####################################################
# EXTRACTING FEATURES

def extract_features_dataset(images):
    """
    Find keypoints and descriptors for each image in the dataset

    Arguments:
    images -- a list of grayscale images

    Returns:
    kp_list -- a list of keypoints for each image in images
    des_list -- a list of descriptors for each image in images
    
    """
    kp_list = []
    des_list = []

    # Create SIFT object
    sift = cv2.xfeatures2d.SIFT_create()
    
    for img in images:
        # Detect keypoints and compute descriptors
        kp, des = sift.detectAndCompute(img, None)
        kp_list.append(kp)
        des_list.append(des)
    
    return kp_list, des_list


def visualize_features(image, kp):
    """
    Visualize extracted features in the image

    Arguments:
    image -- a grayscale image
    kp -- list of the extracted keypoints

    Returns:
    """
    display = cv2.drawKeypoints(image, kp, None, color=(0,255,0), flags=0)
    plt.imshow(display)
    plt.show()



# FEATURE MATCHING -- FILTERED

def match_features_dataset(des_list, dist_threshold=0.6):
    """
    Match features for each subsequent image pair in the dataset

    Arguments:
    des_list -- a list of descriptors for each image in the dataset
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0) 

    Returns:
    matches_list -- list of matches for each subsequent image pair in the dataset. 
               Each matches[i] is a list of matched features from images i and i + 1
               
    """
    matches_list = []

    # Brute Force matching
    bf = cv2.BFMatcher()
    
    # matches features in image i and i+1
    for i in range(len(des_list)-1):
        # Set empty list for each image
        lst = []

        # BFMatcher.knnMatch() to get k best matches. In this example, we will take k=2 so that we can apply ratio test explained by D.Lowe in his paper.
        match = bf.knnMatch(des_list[i], des_list[i+1],k=2)

        # Filter through matches
        for m,n in match:
            if m.distance < (dist_threshold * n.distance):
                lst.append([m])
        matches_list.append(lst)
                       
    return matches_list


def visualize_matches(image1, kp1, image2, kp2, match, outImg):
    """
    Visualize corresponding matches in two images

    Arguments:
    image1 -- the first image in a matched image pair
    kp1 -- list of the keypoints in the first image
    image2 -- the second image in a matched image pair
    kp2 -- list of the keypoints in the second image
    match -- list of matched features from the pair of images

    Returns:
    image_matches -- an image showing the corresponding matches on both image1 and image2 or None if you don't use this function
    """    
    # draw lines to match the features 
    image_matches = cv2.drawMatchesKnn(image1,kp1,image2,kp2, outImg, match,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    return image_matches



def estimate_motion(match, kp1, kp2, k, depth1):
    # Returns roation and translatino matrix
    pass
def estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps):
    pass