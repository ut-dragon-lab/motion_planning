#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from aerial_robot_msgs.msg import FlightNav

pos_err_pub = rospy.Publisher('pos_err', Vector3Stamped, queue_size=10)
uav_odom = Odometry()

def odom_callback(msg):
    global uav_odom
    uav_odom = msg

def nav_callback(msg):
    pos_err = Vector3Stamped()
    pos_err.header = msg.header
    pos_err.vector.x = msg.target_pos_x - uav_odom.pose.pose.position.x
    pos_err.vector.y = msg.target_pos_y - uav_odom.pose.pose.position.y
    pos_err.vector.z = msg.target_pos_z - uav_odom.pose.pose.position.z

    global pos_err_pub
    pos_err_pub.publish(pos_err)

if __name__ == '__main__':
    rospy.init_node('pos_err_path', anonymous=True)

    rospy.Subscriber("/uav/nav", FlightNav, nav_callback)
    rospy.Subscriber("/uav/cog/odom", Odometry, odom_callback)

    rospy.spin()
