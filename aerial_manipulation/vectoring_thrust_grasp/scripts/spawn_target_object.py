#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion

class SpawnObject(object):
    def __init__(self):

        self.initial_pose = Pose()
        self.initial_pose.position.x = rospy.get_param("~x", 0.0);
        self.initial_pose.position.y = rospy.get_param("~y", 0.0);
        self.initial_pose.position.z = rospy.get_param("~z", 0.5);
        q = quaternion_from_euler(0, 0, rospy.get_param("~yaw", 0.0))
        self.initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        sdf_file = rospy.get_param('~sdf_file_path')
        f = open(sdf_file,'r')
        self.sdff = f.read()

        if rospy.get_param("~spawn_from_trigger", 0.0):
            self.trigger_sub = rospy.Subscriber("/trigger", Empty, self.triggerCb)
        else:
            self.spawnModel()

        rospy.on_shutdown(self.deleteModel)

    def triggerCb(self, data):
        self.spawnModel()

    def spawnModel(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox("target_object", self.sdff, "", self.initial_pose, "")


    def deleteModel(self):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        delete_model_prox("target_object")


if __name__=="__main__":

    rospy.init_node('insert_object',log_level=rospy.INFO)
    model_spawn = SpawnObject()

    rospy.spin()
