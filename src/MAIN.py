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

import argparse
from os import path
from time import sleep

import CameraParams_Parser as Cam_Parser
import GPS_VO
import Ground_Truth as GT
import Trajectory_Tools as TT
from Common_Modules import *
from py_MVO import VisualOdometry


def run():

    print '-- Press ESC key to end program\n'
    #Parse the Command Line/Terminal
    cmd_parser = argparse.ArgumentParser()
    cmd_parser.add_argument('txt_file', help= 'Text file that contains all the input parameters. Verify the CameraParams file.')
    args = cmd_parser.parse_args()
    try:
        # Create Camera Parameters object
        CP = Cam_Parser.CameraParams(args.txt_file)
    except:
        print "Error: txt_file argument not .txt file or is in the wrong format."
        return
    # Returns the images' directory, images' format, and GPS_FLAG
    folder, img_format, GPS_flag  = CP.folder, CP.format, CP.GPS_FLAG
    images = TT.images_from_Folder(folder, img_format)  # List of all the images' filepaths

    gps_switch = False  # Determines if GPS was recovered
    if GPS_flag == 'GPS_T' or GPS_flag == 'GPS_T_M':  # Verify if flag was raised
        gps_dict = GPS_VO.gps_filename_dict(images)  # Retrieve the GPS info into a dictionary
        utm_dict = GPS_VO.gps_to_utm(gps_dict)       # Keys: image filepath Values: GPS coordinates
        if gps_dict and utm_dict:
            gps_switch = True  # Verify if GPS info was retrieved
        else:
            print "Warning: No GPS data recovered from the images' EXIF file"

    # Returns the camera intrinsic matrix, feature detector, ground truth (if provided), and windowed displays flag
    K, f_detector, GT_poses, window_flag = CP.CamIntrinMat, CP.featureDetector, CP.groundTruth, CP.windowDisplay
    # Initializing the Visual Odometry object
    vo = VisualOdometry(K, f_detector, GT_poses)
    # Square for the real-time trajectory window
    traj = np.zeros((600, 600, 3), dtype=np.uint8)

    # ------------------ Image Sequence Iteration and Processing ---------------------
    # Gives each image an id number based position in images list
    img_id = 0

    images = images[:40]
    # Initial call to print 0% progress bar
    TT.printProgress(img_id, len(images)-1, prefix='Progress:', suffix='Complete', barLength=50)

    for i, img in enumerate(images):  # Iterating through all images

        k = cv2.waitKey(20) & 0xFF
        if k == 27:  # Wait for ESC key to exit
            cv2.destroyAllWindows()
            return

        imgKLT = cv2.imread(img)  # Read the image for real-time trajectory
        img = cv2.imread(img, 0)  # Read the image for Visual Odometry

        if img_id != 0 and gps_switch is True:
            # Retrieve image distance in order to scale translation vectors
            prev_GPS = gps_dict.values()[img_id - 1]
            cur_GPS = gps_dict.values()[img_id]
            distance = GPS_VO.getGPS_distance(prev_GPS, cur_GPS)  # Returns the distance between current and last GPS

        vo.update(img, img_id)  # Updating the vectors in VisualOdometry class

        cur_t = vo.cur_t  # Retrieve the translation vectors

        # ------- Windowed Displays ---------
        if window_flag == 'WINDOW_YES':
            if img_id > 0:  # Set the points for the real-time trajectory window
                x, y, z = cur_t[0], cur_t[1], cur_t[2]
                TT.drawFeatureMatches(imgKLT, vo.px_ref, vo.px_cur, vo.new_roi)  # Draw the features that were matched
            else:
                x, y, z = 0., 0., 0.

            traj = TT.RT_trajectory_window(traj, x, y, z, img_id)  # Draw the trajectory window
        # -------------------------------------
        sleep(0.1) # Sleep for progress bar update
        img_id += 1  # Increasing the image id
        TT.printProgress(i, len(images)-1, prefix='Progress:', suffix='Complete', barLength=50)  # update progress bar

    # --------------------------------------------------------------------------------

    # Write poses to text file in the images sequences directory
    poses_MVO = open(path.normpath(folder+'/py-MVO_Poses.txt'), 'w')
    for t_v in vo.T_vectors:
        poses_MVO.write(' '.join([str(t[0]) for t in t_v]) + '\n')
    poses_MVO.close()  # Close the Poses text file

    # -------- Plotting Trajectories ----------
    if window_flag == 'WINDOW_YES' or window_flag == 'WINDOW_T':
        # Retrieving the translation vectors from the
        # translation vector list which is an attribute of the VO class
        T_v = [(t[0][0], t[2][0]) for t in vo.T_vectors]

        if GT_poses:  # Ground Truth Data is used in case GPS and GT are available
            # Ground Truth poses in list
            GT_poses = GT.ground_truth(GT_poses)
            # Plot VO and ground truth trajectories
            TT.VO_GT_plot(T_v, GT_poses)

        elif gps_switch:  # Plotting the VO and GPS trajectories
            if GPS_flag == 'GPS_T':
                TT.GPS_VO_plot(T_v, utm_dict)
            elif GPS_flag == 'GPS_T_M':
                # Do merged trajectory
                Merged = 'Not Yet'
        else:
            TT.VO_plot(T_v)
    # -------------------------------------------

    return

#################################################################################

if __name__ == '__main__':

    run()

