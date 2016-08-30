#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker

if __name__ == '__main__':
    rospy.init_node('object_display', anonymous=True)

    ## publish
    pub = rospy.Publisher('cylinder_config', Marker, queue_size=10)

    ## param
    object_cylinder = Marker()
    object_cylinder.header.frame_id = "object"
    object_cylinder.type = 3
    object_cylinder.action = 0
    object_cylinder.color.a = 1.0
    object_cylinder.color.r = 1.0
    object_cylinder.color.g = 0.0
    object_cylinder.color.b = 0.0

    if not rospy.has_param('~radius'):
        rospy.logfatal("no config about radius value")
    else:
        object_cylinder.scale.x = rospy.get_param('~radius')
        object_cylinder.scale.y = rospy.get_param('~radius')

    object_cylinder.scale.z = 0.4

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        object_cylinder.header.stamp = rospy.get_rostime()
        pub.publish(object_cylinder)
        r.sleep()
