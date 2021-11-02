#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class JoyControl(object):
    def __init__(self):

        self.start_pub = rospy.Publisher('/motion_start', Empty, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCb)

    def joyCb(self, msg):
        if len(msg.buttons) != 14:
            rospy.logerr("wrong joystick, please use PS4 joy")
            self.joy_sub.unregister()

        if msg.buttons[4] == 1: # L1
            rospy.loginfo("receive trigger from ps4 joystick, send start flag")
            self.start_pub.publish(Empty())
            self.joy_sub.unregister()


if __name__=="__main__":

    rospy.init_node('joystick_operation',log_level=rospy.INFO)
    joy_control = JoyControl()

    rospy.spin()
