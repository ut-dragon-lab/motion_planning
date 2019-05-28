#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class JoyControl(object):
    def __init__(self):

        self.start_topic = rospy.get_param('~start_topic','/motion_start')
        self.start_pub = rospy.Publisher(self.start_topic, Empty, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joyCb)
        self.PS3_BUTTONS = 17
        self.PS4_BUTTONS = 14
        self.PS3_TRIGGER_BUTTON = 4; # PS3_BUTTON_CROSS_UP
        self.PS4_AXIS_BUTTON_CROSS_UP_DOWN = 10 # up = +1, down= -1

        self.trigger_once = True

    def joyCb(self, msg):

        if not self.trigger_once:
            return

        if len(msg.buttons) == self.PS3_BUTTONS:
            if msg.buttons[self.PS3_TRIGGER_BUTTON] == 1:
                rospy.loginfo("receive trigger from ps3 joystick, send start flag")
                self.trigger_once = False
                self.start_pub(Empty())

        if len(msg.buttons) == self.PS4_BUTTONS:
            if msg.axes[self.PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1: # use axes
                rospy.loginfo("receive trigger from ps4 joystick, send start flag")
                self.trigger_once = False
                self.start_pub.publish(Empty())

if __name__=="__main__":

    rospy.init_node('joystick_operation',log_level=rospy.INFO)
    joy_control = JoyControl()

    rospy.spin()
