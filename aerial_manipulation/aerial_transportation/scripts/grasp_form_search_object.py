#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, Point32, Inertia, Vector3
from visualization_msgs.msg import Marker
from aerial_transportation.srv import *

if __name__ == '__main__':
    rospy.init_node('object_display', anonymous=True)

    object_type = -1
    inertia = Inertia()
    vertices = []
    if not rospy.has_param('~object_type'):
        rospy.logfatal("no config about object_type")
        sys.exit(1)
    else:
        object_type = rospy.get_param('~object_type')

    if not rospy.has_param('~object_mass'):
        rospy.logfatal("no config about object_mass")
        sys.exit(1)
    else:
        inertia.m = rospy.get_param('~object_mass')
        ## TODO: inertia tensor

    if object_type == ObjectConfigureRequest.CONVEX_POLYGONAL_COLUMN:
        ## param: convex_polygonal_column
        vertex_num = 0
        if not rospy.has_param('~vertex_num'):
            rospy.logfatal("no config about number of vetex")
            sys.exit(1)
        else:
            vertex_num = rospy.get_param('~vertex_num')

            for i in range(vertex_num):
                vertex = Vector3()
                if not rospy.has_param('~x_' + str(i + 1)):
                    rospy.logfatal("no config about x value in " + str(i + 1))
                    sys.exit(1)
                else:
                    vertex.x = rospy.get_param('~x_' + str(i + 1))

                if not rospy.has_param('~y_' + str(i + 1)):
                    rospy.logfatal("no config about y value in " + str(i + 1))
                    sys.exit(1)
                else:
                    vertex.y = rospy.get_param('~y_' + str(i + 1))
                vertices.append(vertex)

    if object_type == ObjectConfigureRequest.CYLINDER:
        if not rospy.has_param('~radius'):
            rospy.logfatal("no config about radius value")
            sys.exit(1)
        else:
            object_radius = Vector3()
            object_radius.x = rospy.get_param('~radius')
            vertices.append(object_radius)

    # client
    only_pub = rospy.get_param('~only_pub')
    if not only_pub:
        rospy.wait_for_service('/target_object_configuration')
        try:
            object_configuration = rospy.ServiceProxy('/target_object_configuration', ObjectConfigure)
            resp = object_configuration(object_type, inertia, vertices)
            print resp.message
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    # visualize
    r = rospy.Rate(10) # 10hz
    if object_type == ObjectConfigureRequest.CONVEX_POLYGONAL_COLUMN:
        object_polygon = PolygonStamped()
        object_polygon.header.frame_id = "object"
        for i in range(len(vertices)):
            vertex = Point32()
            vertex.x = vertices[i].x
            vertex.y = vertices[i].y
            vertex.z = vertices[i].z
            object_polygon.polygon.points.append(vertex)

        ## publish
        pub = rospy.Publisher('grasp_form_search/convex_polygonal_column_object', PolygonStamped, queue_size=10)
        while not rospy.is_shutdown():
            object_polygon.header.stamp = rospy.get_rostime()
            pub.publish(object_polygon)
            r.sleep()

    if object_type == ObjectConfigureRequest.CYLINDER:
        object_cylinder = Marker()
        object_cylinder.scale.x = vertices[0].x
        object_cylinder.scale.y = vertices[0].x
        object_cylinder.scale.z = 0.4
        object_cylinder.header.frame_id = "object"
        object_cylinder.type = 3
        object_cylinder.action = 0
        object_cylinder.color.a = 1.0
        object_cylinder.color.r = 1.0
        object_cylinder.color.g = 0.0
        object_cylinder.color.b = 0.0

        pub = rospy.Publisher('grasp_form_search/cylinder_object', Marker, queue_size=10)
        while not rospy.is_shutdown():
            object_cylinder.header.stamp = rospy.get_rostime()
            pub.publish(object_cylinder)
            r.sleep()

