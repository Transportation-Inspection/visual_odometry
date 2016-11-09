"""

MIT License (MIT)

Copyright (c) SUMMER 2016, Carnegie Mellon University

Author: Jahdiel Alvarez

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Original Code:
https://github.com/uoip/monoVO-python

"""

import py_MVO_OptFlow as OF
from Common_Modules import *

# CONSTANT VARIABLES
STAGE_FIRST_FRAME = 0  # The three STAGE variables
STAGE_SECOND_FRAME = 1  # define which function will be
STAGE_DEFAULT_FRAME = 2  # used in the update function.
kMinNumFeature = 1000  # Minimum amount of features needed, if less feature detection is used
fMATCHING_DIFF = 1  # Minimum difference in the KLT point correspondence


# Parameters used for cv2.goodFeaturesToTrack (Shi-Tomasi Features)
feature_params = dict(maxCorners=500,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)


class VisualOdometry:
    """
     Visual Odometry Class:

     Creates the trajectory of an image sequence based on Visual Odometry (VO).
     The trajectory is constructed by estimating the Rotation and translation
     vectors from the Essential Matrix.

    """

    def __init__(self, CameraIntrinMat, f_detector, groundTruth):
        """
        Initialize the Visual Odometry Class:

        self.frame_stage     The current stage of the algorithm
        self.new_frame       The current frame
        self.last_frame      The previous frame
        self.skip_frame      Determines if the next frame will be skipped
        self.last_id         The ID of the previous frame
        self.new_roi         The Region of Interest(ROI) of the current frame
        self.last_roi        The ROI of the previous frame
        self.cur_R           The current concatenated Rotation Matrix
        self.cur_t           The current concatenated Translation Vector
        self.px_ref          The previous corresponded feature points
        self.px_cur          The current corresponded feature points
        self.prev_GPS        The GPS coordinate of previous frame
        self.cur_GPS         The GPS coordinate of current frame
        self.K               Camera Intrinsic Matrix
        self.distCoeff       Distortion Coefficients
        self.Scale           Scale, used to scale the translation and rotation matrix
        self.T_vectors       List which contains all the translation vectors
        self.R_matrices      List which contains all the Rotation matrices
        self.new_cloud       3-D point cloud of current frame, i, and previous frame, i-1
        self.last_cloud      3-D point cloud of previous frame, i-1, and the one before that, i-2
        self.F_detectors     Dictionary of Feature Detectors available
        self.detector        The chosen Feature Detector
        self.annotations     Translation vectors of the ground truth data, taken from a text file
        self.OFF_prev        The previous corresponded feature points for Optical Flow Window
        self.OFF_cur         The current corresponded feature points for Optical Flow Window

        """

        self.frame_stage = 0
        self.new_frame = None
        self.last_frame = None
        self.skip_frame = False
        self.last_id = 0
        self.new_roi = None
        self.last_roi = None
        self.cur_R = None
        self.cur_t = None
        self.px_ref = None
        self.px_cur = None
        self.prev_GPS = None
        self.cur_GPS = None
        self.K = CameraIntrinMat
        self.Scale = 0
        self.T_vectors = []
        self.R_matrices = []
        self.new_cloud = None
        self.last_cloud = None
        self.F_detectors = {'FAST': cv2.FastFeatureDetector_create(threshold=25, nonmaxSuppression=True),
                            'SIFT': cv2.xfeatures2d.SIFT_create(),
                            'SURF': cv2.xfeatures2d.SURF_create(),
                            'SHI-TOMASI': 'SHI-TOMASI'}  # All the features detectors available

        self.detector = self.F_detectors[f_detector]
        self.groundTruth = groundTruth
        self.OFF_prev = None
        self.OFF_cur = None
        if self.groundTruth:
            with open(groundTruth) as f:
                self.groundTruth = f.readlines()

    def getAbsoluteScale(self, frame_id):
        """ Obtains the absolute scale utilizing
        the ground truth poses. (KITTI dataset)"""

        T_mat = self.groundTruth[self.last_id].strip().split()  # Uses the R and t from the last frame
        x_prev = float(T_mat[3])
        y_prev = float(T_mat[7])
        z_prev = float(T_mat[11])
        T_mat = self.groundTruth[frame_id].strip().split()
        x = float(T_mat[3])
        y = float(T_mat[7])
        z = float(T_mat[11])

        return np.sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev))


    def triangulatePoints(self, R, t):
        """Triangulates the feature correspondence points with
        the camera intrinsic matrix, rotation matrix, and translation vector.
        It creates projection matrices for the triangulation process."""

        # The canonical matrix (set as the origin)
        P0 = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0]])
        P0 = self.K.dot(P0)
        # Rotated and translated using P0 as the reference point
        P1 = np.hstack((R, t))
        P1 = self.K.dot(P1)
        # Reshaped the point correspondence arrays to cv2.triangulatePoints's format
        point1 = self.px_ref.reshape(2, -1)
        point2 = self.px_cur.reshape(2, -1)

        return cv2.triangulatePoints(P0, P1, point1, point2).reshape(-1, 4)[:, :3]



    def getRelativeScale(self):
        """ Returns the relative scale based on the 3-D point clouds
         produced by the triangulation_3D function. Using a pair of 3-D corresponding points
         the distance between them is calculated. This distance is then divided by the
         corresponding points' distance in another point cloud."""

        min_idx = min([self.new_cloud.shape[0], self.last_cloud.shape[0]])
        ratios = []  # List to obtain all the ratios of the distances
        for i in xrange(min_idx):
            if i > 0:
                Xk = self.new_cloud[i]
                p_Xk = self.new_cloud[i - 1]
                Xk_1 = self.last_cloud[i]
                p_Xk_1 = self.last_cloud[i - 1]

                if np.linalg.norm(p_Xk - Xk) != 0:
                    ratios.append(np.linalg.norm(p_Xk_1 - Xk_1) / np.linalg.norm(p_Xk - Xk))

        d_ratio = np.median(ratios) # Take the median of ratios list as the final ratio
        return d_ratio


    def frame_Skip(self, pixel_diff):
        """Determines if the current frame needs to be skipped.
         A frame is skipped on the basis that the current feature points
         are almost identical to the previous feature points, meaning the image
         was probably taken from the same place and the translation should be zero."""

        # We tried this parameter with 20, 15, 10, 5, 3, 2, 1 and 0
        # for one dataset and found that 3 produces the best results.
        return pixel_diff < 3

    def detectNewFeatures(self, cur_img):
        """Detects new features in the current frame.
        Uses the Feature Detector selected."""
        if self.detector == 'SHI-TOMASI':
            feature_pts = cv2.goodFeaturesToTrack(cur_img, **feature_params)
            feature_pts = np.array([x for x in feature_pts], dtype=np.float32).reshape((-1, 2))
        else:
            feature_pts = self.detector.detect(cur_img, None)
            feature_pts = np.array([x.pt for x in feature_pts], dtype=np.float32)

        return feature_pts

    def processFirstFrame(self):
        """Process the first frame. Detects feature points on the first frame
        in order to provide them to the Kanade-Lucas-Tomasi Tracker"""

        self.px_ref = self.detectNewFeatures(self.new_frame)
        self.T_vectors.append(tuple([[0], [0], [0]]))
        self.R_matrices.append(tuple(np.zeros((3, 3))))
        self.frame_stage = STAGE_SECOND_FRAME

    def processSecondFrame(self):
        """Process the second frame. Detects feature correspondence between the first frame
        and the second frame with the Kanade-Lucas-Tomasi Tracker. Initializes the
        rotation matrix and translation vector. The first point cloud is formulated."""

        # The images or roi used for the VO process (feature detection and tracking)
        prev_img, cur_img = self.last_frame, self.new_frame
        # Obtain feature correspondence points
        self.px_ref, self.px_cur, _diff = OF.KLT_featureTracking(prev_img, cur_img, self.px_ref)
        # Estimate the essential matrix
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        # Estimate Rotation and translation vectors
        _, self.cur_R, self.cur_t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, self.K)
        self.T_vectors.append(tuple(self.cur_R.dot(self.cur_t)))
        self.R_matrices.append(tuple(self.cur_R))
        # Triangulation, returns 3-D point cloud
        self.new_cloud = self.triangulatePoints(self.cur_R, self.cur_t)
        # For Optical Flow Field
        self.OFF_prev, self.OFF_cur = self.px_ref, self.px_cur
        # The new frame becomes the previous frame
        self.frame_stage = STAGE_DEFAULT_FRAME
        self.px_ref = self.px_cur
        self.last_cloud = self.new_cloud

    def processFrame(self, frame_id):
        """Process the ith frame. Detects feature correspondence between the first frame
        and the second frame with the Kanade-Lucas-Tomasi Tracker. Computes the
        rotation matrix and translation vector, with a relative or absolute scale.
        Also, it formulates a 3-D point cloud."""

        # The images or roi used for the VO process (feature detection and tracking)
        prev_img, cur_img = self.last_frame, self.new_frame
        # Obtain feature correspondence points
        self.px_ref, self.px_cur, px_diff = OF.KLT_featureTracking(prev_img, cur_img, self.px_ref)
        # Verify if the current frame is going to be skipped
        self.skip_frame = self.frame_Skip(px_diff)
        if self.skip_frame:
            if self.px_ref.shape[0] < kMinNumFeature:  # Verify if features on last_frame are sparse
                self.px_cur = self.detectNewFeatures(prev_img)
                self.px_ref = self.px_cur
                self.last_cloud = self.new_cloud
            return

        # Estimate the essential matrix
        E, mask = cv2.findEssentialMat(self.px_cur, self.px_ref, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        # Estimate Rotation and translation vectors
        _, R, t, mask = cv2.recoverPose(E, self.px_cur, self.px_ref, self.K)
        # Triangulation, returns 3-D point cloud
        self.new_cloud = self.triangulatePoints(R, t)


        # Scaling the trajectory
        # If ground truth is provided use it for scaling
        if self.groundTruth:
            # Ground Truth Scale
            self.Scale = self.getAbsoluteScale(frame_id)
        else:
            # Relative Scale
            self.Scale = self.getRelativeScale()

        if (t[2] > t[0] and t[2] > t[1]):  # Accepts only dominant forward motion
            self.cur_t = self.cur_t + self.Scale * self.cur_R.dot(t)  # Concatenate the translation vectors
            self.cur_R = R.dot(self.cur_R)  # Concatenate the rotation matrix
            self.T_vectors.append(tuple(self.cur_t))
            self.R_matrices.append(tuple(self.cur_R))

        if self.px_ref.shape[0] < kMinNumFeature:                     # Verify if the amount of feature points
            self.px_cur = self.detectNewFeatures(cur_img)  # is above the kMinNumFeature threshold

        # For Optical Flow Field
        self.OFF_prev, self.OFF_cur = self.px_ref, self.px_cur
        # The new frame becomes the previous frame
        self.px_ref = self.px_cur
        self.last_cloud = self.new_cloud

    def update(self, img, frame_id):
        """ Updates the stage of the algorithm and the images."""

        assert(img.ndim == 2), "Frame: provided image is not grayscale"

        self.new_frame = img  # Input new image
        # The bottom part of the image is used as the ROI
        self.new_roi = img[int(img.shape[0] * 0.40):img.shape[0], 0:img.shape[1]]

        # The image are processed based on the stage of the algorithm
        if self.frame_stage == STAGE_DEFAULT_FRAME:
            self.processFrame(frame_id)
        elif self.frame_stage == STAGE_SECOND_FRAME:
            self.processSecondFrame()
        elif self.frame_stage == STAGE_FIRST_FRAME:
            self.processFirstFrame()

        if self.skip_frame:  # If the current image is skipped the
            return False          # last frame is not updated.

        # The new frame is converted to last frame
        self.last_id = frame_id
        self.last_frame = self.new_frame
        self.last_roi = self.new_roi

        return True
