#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, Point32

if __name__ == '__main__':
    rospy.init_node('object_display', anonymous=True)

    ## publish
    pub = rospy.Publisher('convex_polygonal_column_config', PolygonStamped, queue_size=10)

    ## param
    vertex_num = 0
    if not rospy.has_param('~vertex_num'):
        rospy.logfatal("no config about number of vetex")
    else:
        vertex_num = rospy.get_param('~vertex_num')

    object_polygon = PolygonStamped()

    for i in range(vertex_num):
        vertex_point = Point32()
        if not rospy.has_param('~x_' + str(i + 1)):
            rospy.logfatal("no config about x value in " + str(i + 1))
        else:
            vertex_point.x = rospy.get_param('~x_' + str(i + 1))

        if not rospy.has_param('~y_' + str(i + 1)):
            rospy.logfatal("no config about y value in " + str(i + 1))
        else:
            vertex_point.y = rospy.get_param('~y_' + str(i + 1))

        object_polygon.polygon.points.append(vertex_point)

    object_polygon.header.frame_id = "object"
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        object_polygon.header.stamp = rospy.get_rostime()
        pub.publish(object_polygon)
        r.sleep()
