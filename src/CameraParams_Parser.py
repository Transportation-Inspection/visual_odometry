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
from os import path
import glob

class CameraParams:
    """ A class for the parsing of the Camera Parameter's text file.
    The parameters obtained are used for the py-MVO algorithm."""
    def __init__(self, txt_file):
        """

        Attributes:
        """
        self.folder = None                 # Image Sequence's Directory
        self.format = None                 # Images' File Format (e.g. PNG, JPG)
        self.isProjMat = None              # Boolean: is True if the Projection Matrix is used
        self.ProjMat = None                # Camera Projection Matrix
        self.CamIntrinMat = None           # Camera Intrinsic Matrix
        self.featureDetector = None        # Feature Detector: SIFT, FAST, SURF, SHI-TOMASI
        self.GPS_FLAG = None               # GPS Flag: Look at the CameraParams.txt file to see the GPS_FLAGS
        self.groundTruth = None            # Ground Truth Poses: a text file with the transformation matrices as 1-D arrays(KITTI dataset format)
        self.images = None                 # List with all the images.

        """See the CameraParams text file to see an in depth explanation of each attribute."""


        # Read the Camera Params file
        try:

            with open(txt_file) as f:
                self.txt = f.readlines()
            idx = 59  # The index where the attributes begin

        except IOError:
            raise IOError('No such file or directory - ' + txt_file)
        except:
            raise IOError('Error encountered opening the text file.')

        try:
            self.folder = path.normpath(self.txt[idx].strip())
            if not path.isdir(self.txt[idx].strip()):
                raise IOError

            self.format = self.txt[idx+1].strip()
            self.images = glob.glob(self.folder + '/*.' + self.format)
            if len(self.images) == 0:
                raise TypeError

        except IOError:
            raise IOError('No such file or directory found - ' + self.txt[idx].strip())
        except TypeError:
            raise TypeError('No images of the specified format were found. Verify format in the CameraParams.txt')
        except:
            raise ValueError('Error encountered with Image Folder or Image Format')

        try:
            self.isProjMat = self.txt[idx+2].strip()
            if self.isProjMat == 'True' or self.isProjMat == 'TRUE':
                # Convert to a 1-D float list
                self.ProjMat = [float(i.strip()) for i in self.txt[idx+3].split()]
                # Convert to a 3x4 numpy array
                self.ProjMat = np.array([[self.ProjMat[0], self.ProjMat[1], self.ProjMat[2], self.ProjMat[3]],
                                         [self.ProjMat[4], self.ProjMat[5], self.ProjMat[6],self.ProjMat[7]],
                                         [self.ProjMat[8], self.ProjMat[9], self.ProjMat[10], self.ProjMat[11]]])
            else:
                # Convert to a 1-D float list
                self.CamIntrinMat = [float(i.strip()) for i in self.txt[idx+4].split()]
                # Convert to a 3x3 numpy array
                self.CamIntrinMat = np.array([[self.CamIntrinMat[0], self.CamIntrinMat[1], self.CamIntrinMat[2]],
                                              [self.CamIntrinMat[3], self.CamIntrinMat[4], self.CamIntrinMat[5]],
                                              [self.CamIntrinMat[6], self.CamIntrinMat[7], self.CamIntrinMat[8]]])

            # If Projection Matrix used, decompose to obtain the Camera Intrinsic Matrix
            if isinstance(self.CamIntrinMat, np.ndarray) is False:
                self.decomposedP = np.array(cv2.decomposeProjectionMatrix(self.ProjMat))
                self.CamIntrinMat = self.decomposedP[0].reshape((3, 3))
        except:
            raise ValueError('Projection or Intrinsic Matrix not valid.')

        self.featureDetector = self.txt[idx+5].strip()
        self.GPS_FLAG = self.txt[idx+6].strip()
        self.groundTruth = self.txt[idx+7].strip()
        # If groundTruth is not provided, it is assigned as False
        if self.groundTruth == '' or self.groundTruth == 'None':
            self.groundTruth = False
        self.windowDisplay = self.txt[idx+8].strip()



