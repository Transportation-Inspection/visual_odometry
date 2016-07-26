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

def ground_truth(folder):
    """ Obtain the Ground Truth poses """
    with open(folder) as f:
        txt_file = f.readlines()

    l_poses = []
    for num, i in enumerate(txt_file):
        i = i.strip().split()

        x = float(i[3])
        y = float(i[7])
        z = float(i[11])

        l_poses.append([x, z])

    return l_poses


def plot_ground_truth(l_poses):
    """ Plots the ground truth data"""
    plt.figure(1)
    plt.plot(*zip(*l_poses), color='red', marker='o')

    # Set plot parameters and show it
    plt.axis('equal')
    plt.grid()
    plt.show()

def run():
    """ Place run() function in main on this file in
    order to plot the Ground Truth data only. """
    folder = raw_input('Enter the filepath for the ground truth poses:')
    l_poses = ground_truth(folder)
    plot_ground_truth(l_poses)

