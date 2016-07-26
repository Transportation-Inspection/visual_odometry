# Installation

## Python 2.7

At https://www.python.org/downloads/ you can find the Python 2.7 version the project requires.
In the Python official documentation and in the internet there is extensive information on the proper installation
of Python on multiple platforms(Windows/Linux).

## OpenCV 3.1.0-dev with opencv_contrib modues

This is the installation proccesses for OpenCV 3 with the extra modules
which provide essential tools such as SIFT and SURF feature detectors
for OpenCV 3. 


https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_setup/py_setup_in_windows/py_setup_in_windows.html#building-opencv-from-source

Follow the steps provided by this link in the "Building OpenCV from source" section:

- Do steps 1-4
- Download and extract the [opencv_contrib](https://github.com/Itseez/opencv_contrib) source.
- Do steps 4-12
- Browse the parameters and look for the form called OPENCV_EXTRA_MODULES_PATH (use the search form to focus rapidly on it)
- Complete this OPENCV_EXTRA_MODULES_PATH by the proper pathname to the <opencv_contrib>/modules value using its browse button.
- Do steps 12-18

Note: It is possible that when using 'import cv2' on the Python shell it might not work.
      If this occurs do the following

- Copy <opencv_build_folder>\lib\Release\cv2.pyd to C:\Python2.7\Lib\site-packages (assuming Python 2.7 is installed to the default location).
- Edit the system's Path variable and append ; <opencv_build_folder>/bin/Release 
- Reboot your system.

*This should resolve the problems. Type 'import cv2' into the Python2.7 shell and verify that it runs.

**The process was tried on Windows 10 using CMAKE
## Numpy, Matplotlib, Exifread, Haversine, UTM

This modules can be simply installed with Python's package manager, PIP.
```
pip install numpy

pip install matplotlib

pip install exifread

pip install haversine

pip install utm
```














