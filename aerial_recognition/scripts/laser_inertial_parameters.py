#!/usr/bin/env python

import sys
import time
import rospy
import math
from hydrus_transform_control.srv  import AddExtraModule

if __name__ == "__main__":

    rospy.init_node("laser_inertial_parameters")

    laser1_parent_link = rospy.get_param("~laser1_parent_link", 0)
    laser1_mass = rospy.get_param("~laser1_mass", 0)
    laser1_offset = rospy.get_param("~laser1_offset", 0)

    laser2_parent_link = rospy.get_param("~laser2_parent_link", 0)
    laser2_mass = rospy.get_param("~laser2_mass", 0)
    laser2_offset = rospy.get_param("~laser2_offset", 0)
    time.sleep(1)

    rospy.wait_for_service("/add_extra_module")

    try:
        set_laser_module = rospy.ServiceProxy("/add_extra_module", AddExtraModule)
        set_laser_module(laser1_parent_link, laser1_mass, laser1_offset)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    time.sleep(0.5)

    try:
        set_laser_module = rospy.ServiceProxy("/add_extra_module", AddExtraModule)
        set_laser_module(laser2_parent_link, laser2_mass, laser2_offset)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
