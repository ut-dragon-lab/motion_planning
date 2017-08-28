#!/usr/bin/env python

import rospy
import sys
import time
import math
import tf
import numpy as np

from std_msgs.msg import Empty, String, Bool
from nav_msgs.msg import Odometry
from aerial_robot_base.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, JointState
from hydrus.srv import AddExtraModule

class HydrusObjectTransportation:
    def init(self):
        rospy.init_node("hydrus_object_transportation", anonymous=True)

        self.GLOBAL_FRAME_ = 0 #FlightNav.GLOBAL_FRAME
        self.LOCAL_FRAME_ = 1 #FlightNav.LOCAL_FRAME

        self.uav_global_xy_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_roll_ = 0.0
        self.uav_pitch_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_linear_vel_ = np.zeros(3)
        self.uav_q_ = np.zeros(4)

        self.target_frame_ = self.GLOBAL_FRAME_
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_yaw_ = 0.0
        self.target_z_pos_internal_ = 0.0

        self.initial_uav_global_xy_pos_ = np.zeros(2)
        self.initial_uav_yaw_ = 0.0

        self.searching_point_ = np.array([[-15,  -15], [-7.5,  -15], [0,  -15], [ 7.5,  -15], [ 15,  -15],
                                          [ 15, -7.5], [ 7.5, -7.5], [0, -7.5], [-7.5, -7.5], [-15, -7.5],
                                          [-15,    0], [-7.5,    0], [0,    0], [ 7.5,    0], [ 15,    0],
                                          [ 15,  7.5], [ 7.5,  7.5], [0,  7.5], [-7.5,  7.5], [-15,  7.5],
                                          [-15,   15], [-7.5,   15], [0,   15], [ 7.5,   15], [ 15,   15]])

        self.target_joint_ = [[0.0113, -0.668,   1.57,   1.57,  0.706],
                              [  1.41,  -1.34,   1.16, 0.0756,  0.846],
                              [   1.1,   1.07,   1.07,   1.16,  0.736]]
        self.target_tf_ = ['/link3', '/link5', '/link1']
        self.extra_module_link_ = [3, 5, 1]
        self.extra_module_mass_ = [0.5, 0.5, 0.5]
        self.extra_module_offset_ = [0.0, 0.0, 0.0]

        self.joint_name_ = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self.transform_count_ = 0
        
        self.searching_point_index_ = 0

        self.detect_object_ = False
        self.target_object_pos_ = np.zeros(3)
        self.target_object_global_xy_pos_ = np.zeros(2)

        self.link4_to_cam_pos_ = np.array([0.3, 0.0, -0.2])
        self.link4_to_cam_rot_ = tf.transformations.quaternion_matrix(np.array([0.707, -0.707, 0.000, -0.000]))[0:3, 0:3]
        
        #state machine
        self.INITIAL_STATE_ = 0
        self.TAKEOFF_STATE_ = 1
        self.OBJECT_SEARCHING_STATE_ = 2
        self.OBJECT_PICKING_STATE_ = 3
        self.OBJECT_PLACING_STATE_ = 4
        self.state_machine_ = self.INITIAL_STATE_
        self.state_name_ = ["initial", "takeoff", "object_searching", "object_picking", "object_placing"]
        
        self.task_start_ = False
        
        #ros param
        self.control_rate_ = rospy.get_param("~control_rate", 20.0);
        self.nav_pos_convergence_thresh_ = rospy.get_param("~nav_pos_convergence_thresh", 0.2)
        self.nav_yaw_convergence_thresh_ = rospy.get_param("~nav_yaw_convergence_thresh", 0.05)
        self.task_start_sub_topic_name_ = rospy.get_param("~task_start_sub_topic_name", "task_start")
        self.uav_start_pub_topic_name_ = rospy.get_param("~uav_start_pub_topic_name_", "/teleop_command/start")
        self.uav_takeoff_pub_topic_name_ = rospy.get_param("~uav_takeoff_pub_topic_name", "/teleop_command/takeoff")
        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")
        #self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/hydrusx/ground_truth")
        self.uav_nav_pub_topic_name_ = rospy.get_param("~uav_nav_pub_topic_name", "/uav/nav")
        self.takeoff_height_ = rospy.get_param("~takeoff_height", 3.5)
        self.state_machine_pub_topic_name_ = rospy.get_param("~state_machine_pub_topic_name", "/state_machine")
        self.nav_xy_vel_thresh_ = rospy.get_param("~nav_xy_vel_thresh", 1.0)
        self.nav_xy_pos_pgain_ = rospy.get_param("~nav_xy_pos_pgain", 0.5)
        self.object_pos_sub_topic_name_ = rospy.get_param("~object_pos_sub_topic_name", "/object_pos")
        self.joint_state_pub_topic_name_ = rospy.get_param("~joint_state_pub_topic_name", "/hydrusx/joints_ctrl")
        self.nav_vel_convergence_thresh_ = rospy.get_param("nav_vel_convergence_thresh_", 0.05)
        self.picking_height_ = rospy.get_param("~picking_height", 0.5)
        self.dropping_box_xy_pos_ = np.zeros(2)
        self.dropping_box_xy_pos_[0] = rospy.get_param("~dropping_box_x", -30)
        self.dropping_box_xy_pos_[1] = rospy.get_param("~dropping_box_y", 20)
        self.target_object_num_ = rospy.get_param("~target_object_num", 3)
        self.mag_control_pub_topic_name_ = rospy.get_param("~mag_control_pub_topic_name_", "/mag_on")
        self.add_extra_module_srv_name_ = rospy.get_param("~add_extra_module_srv_name", "/add_extra_module")
        
        #subscriber
        self.task_start_sub_ = rospy.Subscriber(self.task_start_sub_topic_name_, Empty, self.taskStartCallback)
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        self.object_pos_sub_topic_name_ = rospy.Subscriber(self.object_pos_sub_topic_name_, PoseStamped, self.objectPosCallback)
        
        #publisher
        self.uav_start_pub_ = rospy.Publisher(self.uav_start_pub_topic_name_, Empty, queue_size = 10)
        self.uav_takeoff_pub_ = rospy.Publisher(self.uav_takeoff_pub_topic_name_, Empty, queue_size = 10)
        self.uav_nav_pub_ = rospy.Publisher(self.uav_nav_pub_topic_name_, FlightNav, queue_size = 10)
        self.state_machine_pub_ = rospy.Publisher(self.state_machine_pub_topic_name_, String, queue_size = 10)
        self.joint_state_pub_ = rospy.Publisher(self.joint_state_pub_topic_name_, JointState, queue_size = 10)
        self.mag_control_pub_ = rospy.Publisher(self.mag_control_pub_topic_name_, Bool, queue_size = 10)
        
        #timer
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)

        #tf listener
        self.listener = tf.TransformListener()

        #service client
        self.add_extra_module_srv_client_ = rospy.ServiceProxy(self.add_extra_module_srv_name_, AddExtraModule)
        
    def taskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        self.task_start_ = True
        self.task_start_sub_.unregister()

    def uavOdomCallback(self, msg):
        self.uav_global_xy_pos_ = np.array([msg.pose.pose.position.x,
                                            msg.pose.pose.position.y])
        self.uav_z_pos_ = msg.pose.pose.position.z
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_roll_ = rpy[0]
        self.uav_pitch_ = rpy[1]
        self.uav_yaw_ = rpy[2]
        self.uav_linear_vel_ = np.array([msg.twist.twist.linear.x,
                                         msg.twist.twist.linear.y,
                                         msg.twist.twist.linear.z])
        self.uav_q_ = q
        
    def objectPosCallback(self, msg):
        if msg.pose.orientation.w != 0.0:
            self.detect_object_ = True
            self.target_object_pos_[0] = msg.pose.position.x
            self.target_object_pos_[1] = msg.pose.position.y
            self.target_object_pos_[2] = msg.pose.position.z
        else:
            self.detect_object_ = False
            
    def isConvergent(self, frame, target_xy_pos, target_z_pos, target_yaw = 0):
        if frame == self.GLOBAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0] - self.uav_global_xy_pos_[0], target_xy_pos[1] - self.uav_global_xy_pos_[1], target_z_pos - self.uav_z_pos_])
        elif frame == self.LOCAL_FRAME_:
            delta_pos = np.array([target_xy_pos[0], target_xy_pos[1], target_z_pos - self.uav_z_pos_])
        else:
            return

        delta_yaw = target_yaw - self.uav_yaw_
        delta_yaw = 0.0 #yaw control is no need
        if delta_yaw > math.pi:
            delta_yaw -= math.pi * 2
        elif delta_yaw < -math.pi:
            delta_yaw += math.pi * 2

        if np.linalg.norm(delta_pos) < self.nav_pos_convergence_thresh_ and abs(delta_yaw) < self.nav_yaw_convergence_thresh_ and np.linalg.norm(self.uav_linear_vel_) < self.nav_vel_convergence_thresh_ and abs(self.uav_roll_) < 0.001 and abs(self.uav_pitch_) < 0.001:
            return True
        else:
            return False

    def saturateVelocity(self, xy_vel):
        ret = xy_vel
        if np.linalg.norm(xy_vel) > self.nav_xy_vel_thresh_:
            ret = xy_vel * (self.nav_xy_vel_thresh_ / np.linalg.norm(xy_vel))
        return ret
        
    def goPos2(self, frame, target_xy_pos, target_z_pos, target_yaw = 0):
        delta_xy = target_xy_pos - self.uav_global_xy_pos_
        nav_xy_vel = self.saturateVelocity(delta_xy * self.nav_xy_pos_pgain_)
        
        msg = FlightNav()
        msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.POS_MODE
        
        msg.target_vel_x = nav_xy_vel[0]
        msg.target_vel_y = nav_xy_vel[1]

        if (abs(target_z_pos - self.uav_z_pos_)) < 0.3:
            msg.target_pos_z = target_z_pos
        else:
            if target_z_pos > self.uav_z_pos_:
                msg.target_pos_z = self.uav_z_pos_ + 0.1
            else:
                msg.target_pos_z = self.uav_z_pos_ - 0.02

        msg.header.stamp = rospy.Time.now()
        msg.control_frame = frame

        self.target_frame_ = frame
        self.target_xy_pos_ = target_xy_pos
        self.target_z_pos_ = target_z_pos
        self.target_yaw_ = target_yaw

        self.uav_nav_pub_.publish(msg)

    def goPos(self, frame, target_xy_pos, target_z_pos, target_yaw = 0):
        msg = FlightNav()
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.POS_MODE

        msg.header.stamp = rospy.Time.now()
        msg.control_frame = frame
        msg.target = FlightNav.BASELINK

        msg.target_pos_x = target_xy_pos[0]
        msg.target_pos_y = target_xy_pos[1]
        msg.target_pos_z = target_z_pos

        self.target_frame_ = frame
        self.target_xy_pos_ = target_xy_pos
        self.target_z_pos_ = target_z_pos
        self.target_yaw_ = target_yaw

        self.uav_nav_pub_.publish(msg)

    def setJoint(self, joint_index):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_name_
        joint_state.position = self.target_joint_[joint_index]
        self.joint_state_pub_.publish(joint_state)

    def addExtraModule(self, extra_module_link, extra_module_mass, extra_module_offset):
        rospy.wait_for_service(self.add_extra_module_srv_name_)
        self.add_extra_module_srv_client_(extra_module_link, extra_module_mass, extra_module_offset)

    def controlCallback(self, event):
        if not self.task_start_:
            return

        #navigation
        if self.state_machine_ == self.TAKEOFF_STATE_:
            self.goPos(self.GLOBAL_FRAME_, self.initial_uav_global_xy_pos_, self.takeoff_height_)
        if self.state_machine_ == self.OBJECT_SEARCHING_STATE_:
            self.goPos(self.GLOBAL_FRAME_, self.searching_point_[self.searching_point_index_], self.takeoff_height_)
        if self.state_machine_ == self.OBJECT_PICKING_STATE_:
            [trans, rot] = self.listener.lookupTransform('/link4', self.target_tf_[self.transform_count_], rospy.Time(0))
            xy_offset_vec = np.dot(tf.transformations.quaternion_matrix(self.uav_q_)[0:3, 0:3], trans)[0:2]

            self.goPos(self.GLOBAL_FRAME_, self.target_object_global_xy_pos_ - xy_offset_vec, self.picking_height_)
        if self.state_machine_ == self.OBJECT_PLACING_STATE_:
            self.goPos(self.GLOBAL_FRAME_, self.dropping_box_xy_pos_, self.takeoff_height_)
        #end navigation

        #state machine
        if self.state_machine_ == self.INITIAL_STATE_:
            self.initial_uav_global_xy_pos_ = self.uav_global_xy_pos_
            self.initial_uav_yaw_ = self.uav_yaw_
            self.uav_start_pub_.publish()
            time.sleep(0.5)
            self.uav_takeoff_pub_.publish()
            rospy.loginfo("take off")
            time.sleep(20)
            self.state_machine_ = self.TAKEOFF_STATE_

        elif self.state_machine_ == self.TAKEOFF_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_):
                self.state_machine_ = self.OBJECT_SEARCHING_STATE_

        elif self.state_machine_ == self.OBJECT_SEARCHING_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_):
                if self.detect_object_:
                    rospy.loginfo("object detect")
                    object_in_link4 = self.link4_to_cam_pos_ + np.dot(self.link4_to_cam_rot_, self.target_object_pos_)
                    self.target_object_global_xy_pos_ = np.dot(tf.transformations.quaternion_matrix(self.uav_q_)[0:3, 0:3], object_in_link4)[0:2] + self.uav_global_xy_pos_
                    rospy.logwarn("object:%f %f", self.target_object_global_xy_pos_[0], self.target_object_global_xy_pos_[1])
                    self.setJoint(self.transform_count_)
                    time.sleep(10)
                    self.addExtraModule(self.extra_module_link_[self.transform_count_], self.extra_module_mass_[self.transform_count_], self.extra_module_offset_[self.transform_count_])
                    self.state_machine_ = self.OBJECT_PICKING_STATE_
                else:
                    self.searching_point_index_ += 1
                    if self.searching_point_index_ >= len(self.searching_point_):
                        self.searching_point_index_ = 0
                    self.state_machine_ = self.OBJECT_SEARCHING_STATE_

        elif self.state_machine_ == self.OBJECT_PICKING_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_):
                time.sleep(3)
                self.transform_count_ += 1
                if self.transform_count_ != self.target_object_num_:
                    self.state_machine_ = self.OBJECT_SEARCHING_STATE_
                else:
                    self.state_machine_ = self.OBJECT_PLACING_STATE_

        elif self.state_machine_ == self.OBJECT_PLACING_STATE_:
            if self.isConvergent(self.target_frame_, self.target_xy_pos_, self.target_z_pos_):
                self.mag_control_pub_.publish(False)
                self.state_machine_ = self.OBJECT_SEARCHING_STATE_
                self.transform_count_ = 0
                time.sleep(0.5)
                self.mag_control_pub_.publish(True)
        #end state machine

        self.state_machine_pub_.publish(self.state_name_[self.state_machine_])

if __name__ == '__main__':
    try:
        hydrus_object_transportation = HydrusObjectTransportation()
        hydrus_object_transportation.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
