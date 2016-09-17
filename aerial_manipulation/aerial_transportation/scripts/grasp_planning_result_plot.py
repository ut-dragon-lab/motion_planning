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
    object = rospy.get_param("~object")

    fig = pyplot.figure()
    d_list = []
    phy_list = []
    tau_list = []

    if object == "convex" :
        ax = Axes3D(fig)
        ax.set_xlabel("d [m]")
        ax.set_ylabel("phi [rad]")
        ax.set_zlabel("norm of torque")
        ax.set_xlim(0.1, 0.2)
        ax.set_ylim(0.0, 0.5)
        ax.set_zlim(1.0, 2.0)

    #if object == "cylinder" :

    for line in open(file_name, 'r'):

        itemList = line.split('\t')

        if object == "convex" :
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

            if tau_norm != -1 and tau_norm < 2.0:
                d_list.append(d)
                phy_list.append(phi)
                tau_list.append(tau_norm)

        if object == "cylinder" :
            phi = float(itemList[0])
            tau_norm = float(itemList[1])
            force_norm = float(itemList[2])

            if tau_norm != -1:
                phy_list.append(phi)
                tau_list.append(tau_norm)


    cm = pyplot.get_cmap("winter")
    col = [cm(float(i)/(len(tau_list))) for i in xrange(0, len(tau_list))]

    if object == "convex" :
        ax.scatter(d_list, phy_list, tau_list, s= 2, c=col)
        #ax.scatter(d_list, phy_list, tau_list, s= 1, c='blue')
        #ax.plot_wireframe(d_list, phy_list, tau_list)
        #ax.plot_surface(d_list, phy_list, tau_list) #bad
        start1 = []
        start2 = []
        start3 = []
        start1.append(0.1635)
        start2.append(0.212642)
        start3.append(1.4097)

        ax.scatter(start1, start2, start3, s= 100, c='violet')
        pyplot.show()

    if object == "cylinder" :
        pyplot.plot(phy_list, tau_list)
        pyplot.show()

