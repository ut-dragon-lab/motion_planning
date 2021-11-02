#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import ForceList
from gazebo_msgs.srv import SpawnModel, DeleteModel
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
import numpy as np
import tf2_ros
import ros_numpy as ros_np
from tf.transformations import *

class SpawnObject(object):
    def __init__(self):

        self.initial_pose = Pose()

        # spacial process for quad-type
        self.joint1_goal_angle = rospy.get_param("~joint1_init_angle");
        self.joint3_goal_angle = rospy.get_param("~joint3_init_angle");
        self.joint1_curr_angle = 0
        self.joint3_curr_angle = 0
        self.object_pos = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.joint_state_sub_ = rospy.Subscriber('joint_states', JointState, self.jointStateCallback)
        self.extra_vectoring_force_sub_ = rospy.Subscriber('extra_vectoring_force', ForceList, self.ExtraVectoringFoceCallback)

        self.deleteModel('target_object')
        self.deleteModel('stand')

    def ExtraVectoringFoceCallback(self, msg):
        if len(msg.forces) == 4:
            rospy.loginfo("remove stand")
            self.deleteModel('stand')
            self.extra_vectoring_force_sub_.unregister()

    def jointStateCallback(self, msg):
        self.joint1_curr_angle = msg.position[msg.name.index('joint1_yaw')]
        self.joint3_curr_angle = msg.position[msg.name.index('joint3_yaw')]

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, rospy.Time.now(), rospy.Duration(wait))
        return trans


    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():

            if np.abs(self.joint1_curr_angle - self.joint1_goal_angle) < 0.05 and \
               np.abs(self.joint3_curr_angle - self.joint3_goal_angle) < 0.05 and \
               self.object_pos is None:

                end1_tf = ros_np.numpify(self.getTF('dragon/tail_ball').transform)
                end2_tf = ros_np.numpify(self.getTF('dragon/head_ball').transform)
                end1_pos = translation_from_matrix(end1_tf)
                end2_pos = translation_from_matrix(end2_tf)
                self.object_pos = (end1_pos + end2_pos) / 2
                yaw = np.arctan2((end2_pos - end1_pos)[1], (end2_pos - end1_pos)[0])
                rot = quaternion_from_euler(0, 0, yaw)

                rospy.loginfo("Spawn object becuase the robot joints are openned. position: {}, {}, {}, yaw: {}".format(self.object_pos, end1_pos, end2_pos, yaw))
                self.spawnModel(self.object_pos, rot)

            r.sleep()

    def spawnModel(self, pos, rot):

        obj_width = rospy.get_param('~object_width')
        obj_mass = rospy.get_param('~object_mass')
        static_flag = rospy.get_param('~static_object')

        sdf_file = rospy.get_param('~sdf_file_path')
        f = open(sdf_file,'r')
        object_sdff = f.read()
        object_sdff = object_sdff.replace('obj_width', str(obj_width)) # object width
        object_sdff = object_sdff.replace('obj_mass', str(obj_mass)) # object mass
        object_sdff = object_sdff.replace('obj_ixx', str(obj_mass * obj_width**2 / 6)) # object mass
        object_sdff = object_sdff.replace('static_flag', 'true' if static_flag  else 'false')
        #print("modified sdf content: {}".format(object_sdff))

        sdf_file = rospy.get_param('~sdf_file_path')
        f = open(sdf_file,'r')
        stand_sdff = f.read()
        stand_height = pos[2] - obj_width / 2
        stand_sdff = stand_sdff.replace('obj_width obj_width obj_width', str(obj_width * 1.2) + str(" ") + str(obj_width * 1.2) + str(" ")  + str(stand_height))
        stand_sdff = stand_sdff.replace('obj_mass', str(5.0))
        stand_sdff = stand_sdff.replace('obj_ixx', str(1))
        stand_sdff = stand_sdff.replace('static_flag', 'true')
        #print("modified stand content: {}".format(stand_sdff))

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        pose = Pose()
        pose.position = ros_np.msgify(Point, pos)
        pose.position.z = stand_height / 2
        pose.orientation =  ros_np.msgify(Quaternion, rot)
        spawn_model_prox("stand", stand_sdff, "", pose, "")

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        pose.position =  ros_np.msgify(Point, pos)
        pose.orientation =  ros_np.msgify(Quaternion, rot)
        spawn_model_prox("target_object", object_sdff, "", pose, "")


    def deleteModel(self, name):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(name)


if __name__=="__main__":

    rospy.init_node('insert_object',log_level=rospy.INFO)
    model_spawn = SpawnObject()

    model_spawn.main()

    rospy.spin()
