#!/usr/bin/env python
import rospy
import sys
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
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


    fig = pyplot.figure()
    ax = Axes3D(fig)

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
        
            #if( float(itemList[0]) >  0.2 and float(itemList[0]) <  0.4 and float(itemList[1]) >  0.1 and float(itemList[1]) <  0.3):
            #    rospy.loginfo("%f, %f, %f", float(itemList[0]), float(itemList[1]), float(itemList[2]))
        if state == 2 or state == 6:
            joint1_range.append( float(itemList[0]))
            joint2_range.append( float(itemList[1]))
            joint3_range.append( float(itemList[2]))
        
        if state == 4 or state == 6:
            joint1_dist.append( float(itemList[0]))
            joint2_dist.append( float(itemList[1]))
            joint3_dist.append( float(itemList[2]))


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

    ax.plot(joint1_singular, joint2_singular, joint3_singular, "o", color="#ff0000", ms=4, mew=0.01)
    ax.plot(joint1_range, joint2_range, joint3_range, "o", color="#00ff00", ms=4, mew=0.01)
    ax.plot(joint1_dist, joint2_dist, joint3_dist, "o", color="#0000ff", ms=4, mew=0.01)

    ax.plot(start1, start2, start3, "o", color="#ff00ff", ms=10, mew=0.05)
    ax.plot(goal1, goal2, goal3, "o", color="#ff00ff", ms=10, mew=0.01)
    pyplot.show()

