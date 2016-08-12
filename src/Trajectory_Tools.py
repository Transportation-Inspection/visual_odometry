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

import glob
import math
import sys
from os import path
from itertools import *


from Common_Modules import *


def images_from_Folder(folder, format):
    """ Gets the images from a folder and stores
        the full image's path in a list. """
    folder = path.normpath(folder)

    if format == 'JPEG' or format == 'JPG' or format == 'jpg':
        format = 'jpg'
    elif format == 'PNG' or format == 'png':
        format = 'png'

    # Load all the JPEGs into a list
    return glob.glob(folder + '/*.'+format)


# Print iterations progress
def printProgress (iteration, total, prefix = '', suffix = '', decimals = 2, barLength = 100):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : number of decimals in percent complete (Int)
        barLength   - Optional  : character length of bar (Int)
    """
    filledLength    = int(round(barLength * iteration / float(total)))
    percents        = round(100.00 * (iteration / float(total)), decimals)
    bar             = '#' * filledLength + '-' * (barLength - filledLength)
    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),
    sys.stdout.flush()
    if iteration == total:
        sys.stdout.write('\n')
        sys.stdout.flush()

def drawFeatureMatches(img, ref_pts, cur_pts, vo_roi):
    """ Shows a window which marks the feature correspondence keypoints. """
    # Create some random colors
    color = np.random.randint(0, 255, (len(ref_pts), 3))
    # Draw the tracks
    for i, (new, old) in enumerate(zip(cur_pts, ref_pts)):
        a, b = new.ravel()
        tup = (a, b)
        cv2.circle(img, tup, 5, color[i].tolist(), -1)

    cv2.imshow('Feature Corresponding Matches', img)
    cv2.waitKey(1)

    return

def drawOpticalFlowField(img, ref_pts, cur_pts):
    """ Shows a window which shows the optical flow of detected features. """

    # Draw the tracks
    for i, (new, old) in enumerate(zip(cur_pts, ref_pts)):

        a,b = new.ravel()
        c,d = old.ravel()
        v1 = tuple((new - old)*2.5 + old)
        d_v = [new-old][0]*0.75
        arrow_color = (28,24,178)
        arrow_t1 = rotateFunct([d_v], 0.5)
        arrow_t2 = rotateFunct([d_v], -0.5)
        tip1 = tuple(np.float32(np.array([c, d]) + arrow_t1)[0])
        tip2 = tuple(np.float32(np.array([c, d]) + arrow_t2)[0])
        cv2.line(img, v1,(c,d), (0,255,0), 2)
        cv2.line(img, (c,d), tip1, arrow_color, 2)
        cv2.line(img, (c,d), tip2, arrow_color, 2)
        cv2.circle(img, v1,1,(0,255,0),-1)

    cv2.imshow('Optical Flow Field', img)
    cv2.waitKey(1)

    return


def RT_trajectory_window(window, x, y, z, img_id):
    """ Real-time trajectory window. Draws the VO trajectory
        while the images are being processed. """
    # Drawing the points and creating the Real-time trajectory window
    draw_x, draw_y = int(x) + 290, int(z) + 90
    cv2.circle(window, (draw_x, draw_y), 1, (img_id * 255 / 4540, 255 - img_id * 255 / 4540, 0), 1)
    cv2.rectangle(window, (10, 20), (600, 60), (0, 0, 0), -1)
    text = "Coordinates: x=%2fm y=%2fm z=%2fm" % (x, y, z)
    cv2.putText(window, text, (20, 40), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, 8)
    cv2.imshow('Real-Time Trajectory', window)

    return window


def rotateFunct(pts_l, angle, degrees=False):
    """ Returns a rotated list(function) by the provided angle."""
    if degrees == True:
        theta = math.radians(angle)
    else:
        theta = angle

    R = np.array([ [math.cos(theta), -math.sin(theta)],
                   [math.sin(theta), math.cos(theta)] ])
    rot_pts = []
    for v in pts_l:
        v = np.array(v).transpose()
        v = R.dot(v)
        v = v.transpose()
        rot_pts.append(v)

    return rot_pts


def VO_plot(T_v):
    """ Plot the VO trajectory"""
    plt.figure(1)
    pyMVO, = plt.plot(*zip(*T_v), marker='o', color='b', label='py-MVO')
    plt.legend(handles=[pyMVO])
    # Set plot parameters and show it
    plt.axis('equal')
    plt.grid()
    plt.show()

    return

def VO_GT_plot(T_v, GT_l):
    """ Plot the VO and Ground Truth trajectories"""
    plt.figure(1)
    pyMVO, = plt.plot(*zip(*T_v), marker='o', color='b', label='py-MVO')
    GT, = plt.plot(*zip(*GT_l), marker='o', color='g', label='Ground Truth')
    plt.legend(handles=[pyMVO, GT])
    # Set plot parameters and show it
    plt.axis('equal')
    plt.grid()
    plt.show()

    return


def GPS_VO_plot(T_v, utm_dict):
    """ Plot the VO and GPS trajectories.
        The GPS trajectory is rotated and translated to the origin
        in order to obtain a visual comparison between both trajectories."""

    # Retrieving the GPS coordinates into a list
    # Shifting the trajectory to the origin
    utm_dx = utm_dict.values()[0][0]
    utm_dy = utm_dict.values()[0][1]
    gps = [(u[0] - utm_dx, u[1] - utm_dy) for u in utm_dict.values()]

    # Scale factor from GPS to VO
    last_gps = gps[len(gps) - 1]
    last_vo = T_v[len(T_v) - 1]

    d_gps = math.sqrt((last_gps[0] ** 2) + (last_gps[1] ** 2))
    d_VO = math.sqrt((last_vo[0] ** 2) + (last_vo[1] ** 2))

    scale = d_gps / d_VO

    print 'The scale factor', scale
    # Apply scale factor to the translation vectors
    T_v = [np.array(t) * scale for t in T_v]

    for i, t in enumerate(T_v):
        magn = 0
        if i != 0:
            magn = np.linalg.norm((t - T_v[i - 1]))

        print i, t, math.sqrt((t[0] ** 2) + (t[1] ** 2)), magn

    # Obtaining the angle between the first points of each list: VO list and GPS list
    rotate_idx = 6
    VO_v = np.array(T_v[rotate_idx])
    GPS_v = np.array(gps[rotate_idx])

    # Distance between points.
    d1 = math.sqrt((VO_v[0] - GPS_v[0]) ** 2 + (VO_v[1] - GPS_v[1]) ** 2)
    # Obtain the angle assuming the two points are vectors
    angle = math.acos((VO_v.dot(GPS_v)) / (np.linalg.norm(VO_v) * np.linalg.norm(GPS_v)))
    # Rotates the GPS point only for verification
    GPS_v = rotateFunct([GPS_v], angle)
    # Distance between points after rotation.
    d2 = math.sqrt((VO_v[0] - GPS_v[0][0]) ** 2 + (VO_v[1] - GPS_v[0][1]) ** 2)
    # Verify if points are closer after rotation if not rotate the other way
    if d2 < d1:
        sign = 1
    else:
        sign = -1

    # Rotating the GPS function so it aligns with the VO function
    gps = rotateFunct(gps, sign * angle)

    # --------------------------------------------------

    # Plotting the VO and GPS trajectories
    plt.figure(1)
    GPS, = plt.plot(*zip(*gps), color='red', marker='o', label='GPS')
    pyMVO, = plt.plot(*zip(*T_v), marker='o', color='b', label='py-MVO')
    plt.legend(handles=[pyMVO, GPS])
    # Set plot parameters and show it
    plt.axis('equal')
    plt.grid()
    plt.show()

    return

def unique_everseen(iterable, key=None):
    "List unique elements, preserving order. Remember all elements ever seen."
    # unique_everseen('AAAABBBCCDAABBB') --> A B C D
    # unique_everseen('ABBCcAD', str.lower) --> A B C D
    seen = set()
    seen_add = seen.add
    if key is None:
        for element in ifilterfalse(seen.__contains__, iterable):
            seen_add(element)
            yield element
    else:
        for element in iterable:
            k = key(element)
            if k not in seen:
                seen_add(k)
                yield element

def GPS_VO_Merge_plot(T_v_dict, utm_dict):
    """ Plot the VO and GPS trajectories.
        The GPS trajectory is rotated and translated to the origin
        in order to obtain a visual comparison between both trajectories."""
    k = T_v_dict.keys() + utm_dict.keys()
    k = [i for i in unique_everseen([i for i in k if k.count(i) > 1])]

    T_v, gps_orig = [], []
    for key in k:
        T_v.append(T_v_dict[key])
        gps_orig.append(utm_dict[key])

    # Retrieving the GPS coordinates into a list
    # Shifting the trajectory to the origin
    utm_dx = gps_orig[0][0]
    utm_dy = gps_orig[0][1]

    gps = [(u[0] - utm_dx, u[1] - utm_dy) for u in gps_orig]

    # Scale factor from GPS to VO
    last_gps = gps[len(gps) - 1]
    last_vo = T_v[len(T_v) - 1]

    d_gps = math.sqrt((last_gps[0] ** 2) + (last_gps[1] ** 2))
    d_VO = math.sqrt((last_vo[0] ** 2) + (last_vo[1] ** 2))

    scale = d_gps / d_VO

    # Apply scale factor to the translation vectors
    T_v = [np.array(t) * scale for t in T_v]

    for i, t in enumerate(T_v):
        magn = 0
        if i != 0:
            magn = np.linalg.norm((t - T_v[i - 1]))

        print i, t, math.sqrt((t[0] ** 2) + (t[1] ** 2)), magn

    # Obtaining the angle between the first points of each list: VO list and GPS list
    rotate_idx = min(len(T_v)-1, len(gps)-1)
    VO_v = np.array(T_v[rotate_idx])
    GPS_v = np.array(gps[rotate_idx])

    # Distance between points.
    d1 = math.sqrt((VO_v[0] - GPS_v[0]) ** 2 + (VO_v[1] - GPS_v[1]) ** 2)
    # Obtain the angle assuming the two points are vectors
    angle = math.acos((VO_v.dot(GPS_v)) / (np.linalg.norm(VO_v) * np.linalg.norm(GPS_v)))
    # Rotates the GPS point only for verification
    VO_v = rotateFunct([VO_v], angle)

    # Distance between points after rotation.
    d2 = math.sqrt((VO_v[0][0] - GPS_v[0]) ** 2 + (VO_v[0][1] - GPS_v[1]) ** 2)
    # Verify if points are closer after rotation if not rotate the other way
    if d2 < d1:
        sign = -1
    else:
        sign = 1

    # Rotating the GPS function so it aligns with the VO function
    T_v = rotateFunct(T_v, sign * angle)

    T_v = [np.array((t[0] + utm_dx, t[1] + utm_dy)) for t in T_v]

    # Dictionary key: image path, value: vo utm coordinates
    VO_dict = OrderedDict()
    for key, value in zip(k, T_v):
        VO_dict[key] = value

    # --------------------------------------------------

    # Plotting the VO and GPS trajectories
    plt.figure(1)
    GPS, = plt.plot(*zip(*gps_orig), color='red', marker='o', label='GPS')
    pyMVO, = plt.plot(*zip(*T_v), marker='o', color='b', label='py-MVO')
    plt.legend(handles=[pyMVO, GPS])
    # Set plot parameters and show it
    plt.axis('equal')
    plt.grid()
    plt.show()


    return VO_dict

