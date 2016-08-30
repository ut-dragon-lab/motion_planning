#!/usr/bin/env python
import rospy
import sys
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
import numpy as np
#from scipy import genfromtxt


if __name__=="__main__":
    rospy.init_node('state_validation_plot')

    file_name = rospy.get_param("~file_name")
    joint1_singular = []
    joint2_singular = []
    joint3_singular = []

    joint1_dist = []
    joint2_dist = []
    joint3_dist = []

    joint1_range = []
    joint2_range = []
    joint3_range = []


    joint1_invalid = []
    joint2_invalid = []
    joint3_invalid = []


    fig = pyplot.figure()
    ax = Axes3D(fig)
    #ax = fig.gca(projection='3d')


    ax.set_xlabel("joint1 angle[rad]")
    ax.set_ylabel("joint2 angle[rad]")
    ax.set_zlabel("joint3 angle[rad]")

    ax.set_xlim(-1.58, 1.58)
    ax.set_ylim(-1.58, 1.58)
    ax.set_zlim(-1.58, 1.58)

    for line in open(file_name, 'r'):
        itemList = line.split('\t')
        state = int(itemList[3])

        if  state == 1:
            joint1_singular.append( float(itemList[0]))
            joint2_singular.append( float(itemList[1]))
            joint3_singular.append( float(itemList[2]))
        
        if state == 2 or state == 6:
            joint1_range.append( float(itemList[0]))
            joint2_range.append( float(itemList[1]))
            joint3_range.append( float(itemList[2]))
        
        if state == 4 or state == 6:
            joint1_dist.append( float(itemList[0]))
            joint2_dist.append( float(itemList[1]))
            joint3_dist.append( float(itemList[2]))

        if state > 0:
            joint1_invalid.append( float(itemList[0]))
            joint2_invalid.append( float(itemList[1]))
            joint3_invalid.append( float(itemList[2]))

    start1 = []
    start2 = []
    start3 = []
    goal1 = []
    goal2 = []
    goal3 = []
    start1.append(1.57)
    start2.append(1.57)
    start3.append(1.57)
    goal1.append(-1.57)
    goal2.append(-1.57)
    goal3.append(-1.57)


    #ax.plot(joint1_singular, joint2_singular, joint3_singular, "o", color="#ff0000", ms=4, mew=0.01)
    #ax.plot(joint1_range, joint2_range, joint3_range, "o", color="#00ff00", ms=4, mew=0.01)
    #ax.plot(joint1_range, joint2_range, joint3_range, "o", color=col, ms=4, mew=0.01)
    #ax.plot(joint1_dist, joint2_dist, joint3_dist, "o", color="#0000ff", ms=4, mew=0.01)
    cm = pyplot.get_cmap("winter")
    col = [cm(float(i)/(len(joint1_singular))) for i in xrange(0, len(joint1_singular))]
    #ax.scatter(joint1_singular, joint2_singular, joint3_singular, c=col)
    cm = pyplot.get_cmap("autumn")
    col = [cm(float(i)/(len(joint1_range))) for i in xrange(0, len(joint1_range))]
    #ax.scatter(joint1_range, joint2_range, joint3_range, c=col)
    cm = pyplot.get_cmap("summer")
    col = [cm(float(i)/(len(joint1_dist))) for i in xrange(0, len(joint1_dist))]
    #ax.scatter(joint1_dist, joint2_dist, joint3_dist, c=col)

    #ax.plot(start1, start2, start3, "o", color="#ff00ff", ms=10, mew=0.05)
    #ax.plot(goal1, goal2, goal3, "o", color="#ff00ff", ms=10, mew=0.01)

    cm = pyplot.get_cmap("RdYlGn")
    col = [cm(float(i)/(len(joint1_invalid))) for i in xrange(0, len(joint1_invalid))]
    ax.scatter(joint1_invalid, joint2_invalid, joint3_invalid, c=col)


    ax.scatter(start1, start2, start3, s= 100, c='violet')
    ax.scatter(goal1, goal2, goal3, s= 100, c='violet')

    pyplot.show()

