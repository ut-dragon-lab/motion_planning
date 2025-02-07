// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <differential_kinematics/constraint/collision_avoidance.h>

namespace differential_kinematics
{
  namespace constraint
  {
    CollisionAvoidance::CollisionAvoidance():
      collision_manager_(new fcl::DynamicAABBTreeCollisionManager<double>()),
      min_dist_(1e6)
    {
    }

    void CollisionAvoidance::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<differential_kinematics::Planner> planner,
                                        std::string constraint_name,
                                        bool orientation, bool full_body)
    {
      Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

      /* collision model conversion */
      for(auto link: planner->getRobotModelPtr()->getUrdfModel().links_)
        {
          if(link.second->collision)
            {
              robot_collision_model_.push_back(new fcl::CollisionObject<double>(createGeometryObject(link.second)));
              /* add additional data */
              robot_collision_model_.back()->setUserData(link.second.get());
            }
        }

      nc_ = robot_collision_model_.size();

      getParam<double>("collision_damping_gain", collision_damping_gain_, 0.01);
      getParam<double>("collision_distance_constraint_range", collision_distance_constraint_range_, 0.03);
      getParam<double>("collision_distance_forbidden_range", collision_distance_forbidden_range_, 0.01);    }

    std::shared_ptr<fcl::CollisionGeometry<double>> CollisionAvoidance::createGeometryObject(urdf::LinkSharedPtr link)
    {
      urdf::GeometrySharedPtr geom = link->collision->geometry;

      if(geom->type == urdf::Geometry::BOX)
        {
          urdf::Vector3 dim = urdf::dynamic_pointer_cast<urdf::Box>(geom)->dim;
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Box<double>(dim.x, dim.y, dim.z));
        }
      else if(geom->type == urdf::Geometry::SPHERE)
        {
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Sphere<double>(urdf::dynamic_pointer_cast<urdf::Sphere>(geom)->radius));
        }
      else if(geom->type == urdf::Geometry::CYLINDER)
        {
          return std::shared_ptr<fcl::CollisionGeometry<double> >(new fcl::Cylinder<double>(urdf::dynamic_pointer_cast<urdf::Cylinder>(geom)->radius, urdf::dynamic_pointer_cast<urdf::Cylinder>(geom)->length));
        }
      else if(geom->type == urdf::Geometry::MESH)
        {
          ROS_WARN_STREAM("MESH --- currently not supported");
          /* TODO implement the mesh type */
        }
      else
        {
          ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
        }

      return nullptr;
    }

    bool CollisionAvoidance::getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug)
    {   
       std::cout<<"col"<<std::endl;     
      const auto robot_model = planner_->getRobotModelPtr();
      const auto joint_positions =  planner_->getTargetJointVector<KDL::JntArray>();
      const auto& seg_frames = robot_model->getSegmentsTf();

      A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
      lb = Eigen::VectorXd::Constant(nc_, -1e6);
      ub = Eigen::VectorXd::Constant(nc_, 1e6);

      /* get root tf in world frame */
      KDL::Frame f_root = planner_->getTargetRootPose<KDL::Frame>();

      /* check the distance */
      for(auto it = robot_collision_model_.begin(); it != robot_collision_model_.end(); it++)
        {
          /* get collision-included link tf in root frame */
          urdf::Link link_info = *(reinterpret_cast<urdf::Link *>((*it)->getUserData()));
          KDL::Frame f_link = seg_frames.at(link_info.name);

          /* get collision model offset tf in collision-included link frame */
          KDL::Frame f_collision_obj_offset(KDL::Rotation::Quaternion(link_info.collision->origin.rotation.x,
                                                                      link_info.collision->origin.rotation.y,
                                                                      link_info.collision->origin.rotation.z,
                                                                      link_info.collision->origin.rotation.w),
                                            KDL::Vector(link_info.collision->origin.position.x,
                                                        link_info.collision->origin.position.y,
                                                        link_info.collision->origin.position.z));

          /* get collision model offset tf in world frame */
          KDL::Frame f_collision_obj = f_root * f_link * f_collision_obj_offset;

          double qx, qy, qz, qw;
          f_collision_obj.M.GetQuaternion(qx, qy, qz, qw);
          if(debug) ROS_INFO("link `%s` collision model origin w.r.t in world frame : [%f, %f, %f], orientation: [%f, %f, %f, %f]", link_info.name.c_str(), f_collision_obj.p.x(), f_collision_obj.p.y(), f_collision_obj.p.z(), qx, qy, qz, qw);

          /* convert from KDL to FCL and update collision object */
          Eigen::Vector3d pos;
          tf::vectorKDLToEigen(f_collision_obj.p, pos);
          Eigen::Quaterniond rot;
          tf::quaternionKDLToEigen(f_collision_obj.M, rot);

          (*it)->setTransform(rot, pos);
          (*it)->computeAABB();

          /* start nearest points calculation */
          fcl::DistanceResult<double> result;
          result.clear();
          fcl::DistanceRequest<double> request(true);
          DistanceData distance_data;
          distance_data.request = request;
          collision_manager_->distance((*it), &distance_data, CollisionAvoidance::defaultDistanceFunction);

          if(boost::math::isnan(distance_data.result.nearest_points[0](0) + distance_data.result.nearest_points[0](1) + distance_data.result.nearest_points[0](2)))
            {
              ROS_WARN("FCL: broadphase distance process causes nan");

              /* special process */
              std::vector< fcl::CollisionObject<double> * > env_objs;
              collision_manager_->getObjects (env_objs);

              fcl::DistanceResult<double> result;
              fcl::DistanceRequest<double> request(true);
              double distance = 1e6;
              for(auto env_obj: env_objs)
                {
                  result.clear();
                  double distance_temp = fcl::distance(env_obj, (*it), request, result);
                  ROS_INFO_STREAM("distance is "<< distance_temp << "point of env in world frame: " << result.nearest_points[0].transpose() << ", point on robot in world framed: " << result.nearest_points[1]);
                  if(boost::math::isnan(result.nearest_points[0](0)))
                    {
                      ROS_INFO_STREAM("object type1: " << env_obj->getNodeType() <<  "; object type2: " << (*it)->getNodeType());

                      if(env_obj->getNodeType()  == fcl::GEOM_BOX && (*it)->getNodeType()  == fcl::GEOM_BOX)
                        {
                          auto obj1_ptr = reinterpret_cast<const fcl::Box<double>*>((env_obj->collisionGeometry()).get());
                          auto obj2_ptr = reinterpret_cast<const fcl::Box<double>*>((*it)->collisionGeometry().get());

                          ROS_INFO("obj1: size: [%f, %f, %f], pos: [%f, %f, %f], rot: [%f, %f, %f, %f]",
                                   obj1_ptr->side[0], obj1_ptr->side[1], obj1_ptr->side[2],
                                   (*it)->getTranslation()[0], (*it)->getTranslation()[1], (*it)->getTranslation()[2],
                                   (*it)->getQuatRotation().x(), (*it)->getQuatRotation().y(), (*it)->getQuatRotation().z(), (*it)->getQuatRotation().w());
                          ROS_INFO("obj2: size: [%f, %f, %f], pos: [%f, %f, %f], rot: [%f, %f, %f, %f]",
                                   obj2_ptr->side[0], obj2_ptr->side[1], obj2_ptr->side[2],
                                   (*it)->getTranslation()[0], (*it)->getTranslation()[1], (*it)->getTranslation()[2],
                                   (*it)->getQuatRotation().x(), (*it)->getQuatRotation().y(), (*it)->getQuatRotation().z(), (*it)->getQuatRotation().w());
                        }
                    }

                  if(!boost::math::isnan(result.nearest_points[0](0) + result.nearest_points[0](1) + result.nearest_points[0](2) + result.nearest_points[1](0) + result.nearest_points[1](1) + result.nearest_points[1](2)))
                    {
                      if(distance_temp < distance && distance_temp > 0)
                        {
                          distance_data.result = result;
                          distance_data.result.min_distance = distance_temp;
                          distance = distance_temp;
                        }
                    }
                }

              if(distance == 1e6)
                {
                  ROS_WARN("all narrowphase distance process cause nan");
                  return false;
                }

            }

          /* closest points in local object frame */
          double min_dist = distance_data.result.min_distance;
          if(debug) ROS_WARN("distance = %f", min_dist);

          if(min_dist <= 0)
            {
              ROS_ERROR("distance: %f is negative", min_dist);
              return false;
            }

          /* update the min distance */
          if(min_dist < min_dist_)
            {
              min_dist_ = min_dist;
              min_dist_link_ = link_info.name;
              ROS_DEBUG_STREAM("update min dist, " << min_dist_link_ << ": " << min_dist_);
            }

          // fcl::DistanceDta::nearest_points is described in the world coordinates
          // https://github.com/flexible-collision-library/fcl/commit/91d9d4d5735e44f91ba9df013c9fcd16a22938e4#diff-a6cbd7f133bb7a9426e02c3070d71dd3
          KDL::Vector p_in_env;
          std::vector< fcl::CollisionObject<double> * > env_objs;
          collision_manager_->getObjects(env_objs);
          for(auto env_obj : env_objs)
            {
              /* TODO:this seaching algorithm is not good! */
              if(env_obj->collisionGeometry().get() == distance_data.result.o1)
                {
                  if(debug)
                    ROS_INFO_STREAM("the collision object origin world position is "
                                    <<  env_obj->getTranslation().transpose()
                                    << ", the nearest point in collision object w.r.t world frame is"
                                    << distance_data.result.nearest_points[0].transpose());
                  tf::vectorEigenToKDL(distance_data.result.nearest_points[0], p_in_env);
                }
            }

          if(debug) ROS_INFO_STREAM("the nearest point in robot w.r.t world frame is" << distance_data.result.nearest_points[1].transpose());
          KDL::Vector p_in_robot;
          tf::vectorEigenToKDL(distance_data.result.nearest_points[1], p_in_robot);
          KDL::Vector p_in_robot_local_frame = f_collision_obj.Inverse() * p_in_robot;
          if(debug) ROS_INFO("the nearest point in robot w.r.t link collision frame is %f, %f, %f", p_in_robot_local_frame.x(), p_in_robot_local_frame.y(), p_in_robot_local_frame.z());


          /* conver normal to root link: from env to robot */
          Eigen::Vector3d distance_normal = aerial_robot_model::kdlToEigen(p_in_robot - p_in_env).normalized();

          /* update jacobian and update constraint */
          int index = std::distance(robot_collision_model_.begin(), it);
          Eigen::MatrixXd jacobian = robot_model->getJacobian(joint_positions, link_info.name.c_str(), f_collision_obj_offset * p_in_robot_local_frame);
          A.row(index) = distance_normal.transpose() * jacobian.topRows(3);

          if(min_dist < collision_distance_constraint_range_)
            lb(index) = damplingBound(min_dist, - collision_damping_gain_, collision_distance_constraint_range_, collision_distance_forbidden_range_);
          else
            lb(index) = -1e6;
        }
      if(!full_body_) A.leftCols(6).setZero(); // good

      if(debug)
        {
          std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
          std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb << std::endl;
          std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
        }

      return true;
    }

    bool CollisionAvoidance::defaultDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double& dist)
    {
      auto* cdata = static_cast<DistanceData*>(cdata_);
      const fcl::DistanceRequest<double>& request = cdata->request;
      fcl::DistanceResult<double>& result = cdata->result;

      if(cdata->done) { dist = result.min_distance; return true; }

      fcl::distance(o1, o2, request, result);

      dist = result.min_distance;

      if(dist <= 0) return true; // in collision or in touch

      return cdata->done;
    }

    void CollisionAvoidance::result()
    {
      std::cout << constraint_name_ << "min distance: "
                << min_dist_ << ", at " << min_dist_link_ << std::endl;
    }
      };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::CollisionAvoidance, differential_kinematics::constraint::Base);
