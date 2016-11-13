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
from utm import to_latlon

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

    CP = Cam_Parser.CameraParams(args.txt_file)

    # Returns the images' directory, images' format, list of images and GPS_FLAG
    folder, img_format, images, GPS_flag = CP.folder, CP.format, CP.images, CP.GPS_FLAG

    gps_switch = False  # Determines if GPS was recovered
    if GPS_flag == 'GPS_T' or GPS_flag == 'GPS_T_M':  # Verify if flag was raised
        gps_dict = GPS_VO.gps_filename_dict(images)  # Retrieve the GPS info into a dictionary
        utm_dict = GPS_VO.gps_to_utm(gps_dict)       # Keys: image filepath Values: GPS coordinates
        if gps_dict and utm_dict:
            gps_switch = True  # Verify if GPS info was retrieved
            # Write GPS to text file in the images sequences directory
            GPS_utm_coord = open(path.normpath(folder + '/raw_GPS.txt'), 'w')
            for key, value in utm_dict.items():
                value = to_latlon(value[0], value[1], 17, 'U') # Specific to Pittsburgh
                GPS_utm_coord.write(key + ' ' + str(value[0]) + ' ' + str(value[1]) + '\n')
            GPS_utm_coord.close()  # Close the Poses text file
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
    T_v_dict = OrderedDict()  # dictionary with image and translation vector as value
    # Initial call to print 0% progress bar
    TT.printProgress(img_id, len(images)-1, prefix='Progress:', suffix='Complete', barLength=50)

    for i, img_path in enumerate(images):  # Iterating through all images

        k = cv2.waitKey(10) & 0xFF
        if k == 27:  # Wait for ESC key to exit
            cv2.destroyAllWindows()
            return

        imgKLT = cv2.imread(img_path)  # Read the image for real-time trajectory
        img = cv2.imread(img_path, 0)  # Read the image for Visual Odometry

        # Create a CLAHE object (contrast limiting adaptive histogram equalization)
        clahe = cv2.createCLAHE(clipLimit=5.0)
        img = clahe.apply(img)

        if img_id != 0 and gps_switch is True:
            # Retrieve image distance in order to scale translation vectors
            prev_GPS = gps_dict.values()[img_id - 1]
            cur_GPS = gps_dict.values()[img_id]
            distance = GPS_VO.getGPS_distance(prev_GPS, cur_GPS)  # Returns the distance between current and last GPS

        if vo.update(img, img_id):  # Updating the vectors in VisualOdometry class
            if img_id == 0:
                T_v_dict[img_path] = ([[0], [0], [0]])
            else:
                T_v_dict[img_path] = vo.cur_t   # Retrieve the translation vectors for dictionary
            cur_t = vo.cur_t  # Retrieve the translation vectors

            # ------- Windowed Displays ---------
            if window_flag == 'WINDOW_YES':
                if img_id > 0:  # Set the points for the real-time trajectory window
                    x, y, z = cur_t[0], cur_t[1], cur_t[2]
                    TT.drawOpticalFlowField(imgKLT, vo.OFF_prev, vo.OFF_cur)  # Draw the features that were matched
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
    for t_v, R_m in zip(vo.T_vectors, vo.R_matrices):
        T = np.hstack((R_m, t_v)).flatten()
        poses_MVO.write(' '.join([str(t) for t in T]) + '\n')
    poses_MVO.close()  # Close the Poses text file

    # Write the images path and translation vector to text file in the images sequences directory
    VO_t = open(path.normpath(folder + '/py-MVO_TV.txt'), 'w')
    # Retrieving the translation vectors from the
    # translation vector dictionary and write it in a txt file
    T_v = []
    for key, value in T_v_dict.items():
        T_v_dict[key] = np.array((value[0][0], value[2][0]))
        T_v.append((value[0][0], value[2][0]))
        VO_t.write(key + ' ' + str(value[0][0]) + ' ' + str(value[2][0]) + '\n')

    VO_t.close()  # Close the Poses text file

    # -------- Plotting Trajectories ----------
    if window_flag == 'WINDOW_YES' or window_flag == 'WINDOW_T':

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
                VO_dict = TT.GPS_VO_Merge_plot(T_v_dict, utm_dict)

                # Write GPS to text file in the images sequences directory
                VO_utm_coord = open(path.normpath(folder + '/py-MVO_GPS.txt'), 'w')
                for key, value in VO_dict.items():
                    value = to_latlon(value[0], value[1], 17, 'U')
                    VO_utm_coord.write(key+' '+str(value[0])+' '+str(value[1])+'\n')
                VO_utm_coord.close()  # Close the Poses text file

        else:
            TT.VO_plot(T_v)
    # -------------------------------------------

    return

#################################################################################

if __name__ == '__main__':

    run()

