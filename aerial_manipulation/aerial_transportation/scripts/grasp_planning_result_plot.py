#!/usr/bin/env python
import rospy
import sys
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
import numpy as np
#from scipy import genfromtxt

#option1: http://www.turbare.net/transl/scipy-lecture-notes/intro/matplotlib/matplotlib.html
#option2: http://d.hatena.ne.jp/white_wheels/20100327/p3

if __name__=="__main__":
    rospy.init_node('grasp_planning_plot')

    file_name = rospy.get_param("~file_name")
    d_list = []
    phy_list = []
    tau_list = []

    fig = pyplot.figure()
    ax = Axes3D(fig)

    ax.set_xlabel("d [m]")
    ax.set_ylabel("phi [rad]")
    ax.set_zlabel("norm of torque")

    ax.set_xlim(0, 0.2)
    ax.set_ylim(-1.58, 1.58)
    ax.set_zlim(0, 5.0)

    for line in open(file_name, 'r'):
        itemList = line.split('\t')
        side = int(itemList[0])
        d = float(itemList[1])
        phi = float(itemList[2])
        tau_norm = float(itemList[3])
        force_norm = float(itemList[4])
        """
        if tau_norm == -1 and force_norm == -1:
            # kinemitcs and statics are both invalid
            d_list.append(d)
            phy_list.append(phi)
            tau_list.append(0.0)
        elif tau_norm == -1 and force_norm == 0:
            # kinemitcs is valid, but statics is invalid
            d_list.append(d)
            phy_list.append(phi)
            tau_list.append(0.0)
        else:
            d_list.append(d)
            phy_list.append(phi)
            tau_list.append(tau_norm)
        """

        if tau_norm != -1
            d_list.append(d)
            phy_list.append(phi)
            tau_list.append(tau_norm)


    ax.scatter(d_list, phy_list, tau_list, s= 10, c='violet')

    pyplot.show()
