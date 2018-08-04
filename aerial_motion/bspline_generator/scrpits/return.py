#!/usr/bin/env python

import sys
import time
import rospy

import math
from sensor_msgs.msg import JointState
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

if __name__=="__main__":
    rospy.init_node("return")

    return_x = rospy.get_param("~return_x", 0.3)
    return_y = rospy.get_param("~return_y", 0.9)
    return_yaw = rospy.get_param("~return_yaw", -1.57)

    return_delay = rospy.get_param("~return_delay", 15.0)

    final_roll = rospy.get_param("~final_roll", 0.747568487412)
    final_pitch = rospy.get_param("~final_pitch", -0.100243185628)

    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "/dragon/joints_ctrl")
    joint_control_pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=10)
    uav_nav_topic_name = rospy.get_param("~uav_nav_topic_name", "/uav/nav")
    uav_nav_pub = rospy.Publisher(uav_nav_topic_name, FlightNav, queue_size=10)

    desire_coord_topic_name = rospy.get_param("~desire_coord_topic_name", "/desire_coordinate")
    desire_coord_pub = rospy.Publisher(desire_coord_topic_name, DesireCoord, queue_size=10)

    time.sleep(0.5)

    ctrl_joint = JointState()
    ctrl_joint.position = []

    ctrl_joint.position.append(0)
    ctrl_joint.position.append(1.57)
    ctrl_joint.position.append(0)
    ctrl_joint.position.append(1.57)
    ctrl_joint.position.append(0)
    ctrl_joint.position.append(1.57)
    joint_control_pub.publish(ctrl_joint)

    start_time = rospy.get_rostime()
    desire_coord = DesireCoord()

    rospy.loginfo("send the level command");
    
    while rospy.get_rostime().to_sec() - start_time.to_sec() < return_delay :

        if rospy.get_rostime().to_sec() - start_time.to_sec()  < return_delay * 2 / 3:
            rate = 1 - (rospy.get_rostime().to_sec() - start_time.to_sec()) / (return_delay * 2 / 3)
            desire_coord.roll = rate * final_roll
            desire_coord.pitch = rate * final_pitch
            desire_coord_pub.publish(desire_coord)

        time.sleep(0.1)

    rospy.loginfo("send the return point command");

    uav_nav = FlightNav()
    uav_nav.header.frame_id = "/world"
    uav_nav.header.stamp = rospy.get_rostime()

    uav_nav.control_frame = uav_nav.WORLD_FRAME;
    uav_nav.target = uav_nav.COG;
    uav_nav.pos_xy_nav_mode = uav_nav.POS_MODE;
    uav_nav.target_pos_x = return_x;
    uav_nav.target_pos_y = return_y;
    uav_nav.psi_nav_mode = uav_nav.POS_MODE;
    uav_nav.target_psi = return_yaw;
    uav_nav_pub.publish(uav_nav);

