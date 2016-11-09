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

"""


from Common_Modules import *


# CONSTANTS
fMATCHING_DIFF = 1  # Minimum difference in the KLT point correspondence

lk_params = dict(winSize=(21, 21),  # Parameters used for cv2.calcOpticalFlowPyrLK (KLT tracker)
                 maxLevel=3,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def KLT_featureTracking(image_ref, image_cur, px_ref):
    """Feature tracking using the Kanade-Lucas-Tomasi tracker.
    A backtracking check is done to ensure good features. The backtracking features method
    consist of tracking a set of features, f-1, onto a new frame, which will produce the corresponding features, f-2,
    on the new frame. Once this is done we take the f-2 features, and look for their
    corresponding features, f-1', on the last frame. When we obtain the f-1' features we look for the
    absolute difference between f-1 and f-1', abs(f-1 and f-1'). If the absolute difference is less than a certain
    threshold(in this case 1) then we consider them good features."""

    # Feature Correspondence with Backtracking Check
    kp2, st, err = cv2.calcOpticalFlowPyrLK(image_ref, image_cur, px_ref, None, **lk_params)
    kp1, st, err = cv2.calcOpticalFlowPyrLK(image_cur, image_ref, kp2, None, **lk_params)

    d = abs(px_ref - kp1).reshape(-1, 2).max(-1)  # Verify the absolute difference between feature points
    good = d < fMATCHING_DIFF  # Verify which features produced good results by the difference being less
                               # than the fMATCHING_DIFF threshold.
    # Error Management
    if len(d) == 0:
        print 'Error: No matches where made.'
    elif list(good).count(
            True) <= 5:  # If less than 5 good points, it uses the features obtain without the backtracking check
        print 'Warning: No match was good. Returns the list without good point correspondence.'
        return kp1, kp2

    # Create new lists with the good features
    n_kp1, n_kp2 = [], []
    for i, good_flag in enumerate(good):
        if good_flag:
            n_kp1.append(kp1[i])
            n_kp2.append(kp2[i])

    # Format the features into float32 numpy arrays
    n_kp1, n_kp2 = np.array(n_kp1, dtype=np.float32), np.array(n_kp2, dtype=np.float32)

    # Verify if the point correspondence points are in the same
    # pixel coordinates. If true the car is stopped (theoretically)
    d = abs(n_kp1 - n_kp2).reshape(-1, 2).max(-1)

    # The mean of the differences is used to determine the amount
    # of distance between the pixels
    diff_mean = np.mean(d)

    return n_kp1, n_kp2, diff_mean


def betterMatches(F, points1, points2):
    """ Minimize the geometric error between corresponding image coordinates.
    For more information look into OpenCV's docs for the cv2.correctMatches function."""

    # Reshaping for cv2.correctMatches
    points1 = np.reshape(points1, (1, points1.shape[0], 2))
    points2 = np.reshape(points2, (1, points2.shape[0], 2))

    newPoints1, newPoints2 = cv2.correctMatches(F, points1, points2)

    return newPoints1[0], newPoints2[0]





