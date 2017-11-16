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
    delta_list = []
    state_list = []
    max_tau_list = []
    max_thrust_list = []
    min_thrust_list = []

    if object == "convex" :
        ax = Axes3D(fig)
        ax.set_xlabel("d [m]")
        ax.set_ylabel("delta [rad]")
        ax.set_zlabel("norm of torque")
        ax.set_xlim(0.1, 0.3)
        ax.set_ylim(0.0, 0.6)
        ax.set_zlim(2.0, 4.0)

    for line in open(file_name, 'r'):

        itemList = line.split('\t')

        if object == "convex" :
            side = int(itemList[0])
            d = float(itemList[1])
            delta = float(itemList[2])
            state = float(itemList[3])
            max_tau = float(itemList[4])
            max_thrust = float(itemList[5])
            min_thrust = float(itemList[6])

            if max_tau != -1 :
                d_list.append(d)
                delta_list.append(delta)
                state_list.append(state)
                max_tau_list.append(max_tau)
                max_thrust_list.append(max_thrust)
                min_thrust_list.append(min_thrust)

        if object == "cylinder" :
            delta = float(itemList[0])
            state = float(itemList[1])
            max_tau = float(itemList[2])
            max_thrust = float(itemList[3])
            min_thrust = float(itemList[4])

            if max_tau != -1:
                delta_list.append(delta)
                state_list.append(state)
                max_tau_list.append(max_tau)
                max_thrust_list.append(max_thrust)
                min_thrust_list.append(min_thrust)

    cm = pyplot.get_cmap("winter")
    col = [cm(float(i)/(len(max_tau_list))) for i in xrange(0, len(max_tau_list))]

    if object == "convex" :
        ax.scatter(d_list, delta_list, max_tau_list, s= 2, c=col)
        start1 = []
        start2 = []
        start3 = []
        start1.append(0.225)
        start2.append(0.159704)
        start3.append(3.09678)

        ax.scatter(start1, start2, start3, s= 100, c='violet')
        pyplot.show()

    if object == "cylinder" :
        pyplot.plot(delta_list, max_tau_list)
        pyplot.show()

