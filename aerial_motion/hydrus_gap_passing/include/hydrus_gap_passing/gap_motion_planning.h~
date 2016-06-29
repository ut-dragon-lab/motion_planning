#include <hydra_gap_passing/motion_planning.h>


bool gap_motion_planning(tf::Vector3 left_bound, tf::Vector3 right_bound,  TransformController* transform_controller, std::vector<double> start_state, std::vector<double> goal_state, std::vector<conf_values>& original_path, planning_scene::PlanningScene* planning_scene,   moveit_msgs::CollisionObject collision_object)
  {
    original_path.resize(0);
    real_robot_path_.resize(0);

    tf::Vector3 left_corner = world2GapCoord(left_bound);
    tf::Vector3 right_corner = world2GapCoord(right_bound);

    //float l = transform_controller->getLinkLength() - 0.025; //temporary
    //float d = transform_controller->getPropellerDiameter() + 0.1;
    //temporary
    float l = 0.55; //link1
    float d = 0.3; //

    //overlap type 
    int overlap_type = NO_OVERLAP;
    if(left_corner.getX() >= right_corner.getX())
      {
        overlap_type = X_AXIS_OVERLAP;
        ROS_INFO("x axis overlap");
      }
    else if((left_corner.getY() + left_corner.getZ() < right_corner.getY() )||
            right_corner.getY() + right_corner.getZ() < left_corner.getY())
      {
        ROS_INFO("no overlap");
      }
    else
      {
        overlap_type = Y_AXIS_OVERLAP;
        ROS_INFO("y axis overlap");
      }

    //length of gap
    float length_of_gap = 0;
    float gap_inclination = 0;
    if(overlap_type == NO_OVERLAP)
      {
        if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
          {
            length_of_gap = distance(left_corner.getX(), right_corner.getX(),
                                     left_corner.getY() + left_corner.getZ(), 
                                     right_corner.getY());
            gap_inclination = atan2(right_corner.getY() - (left_corner.getY() + left_corner.getZ()), right_corner.getX() - left_corner.getX());
            ROS_INFO("left half lower, length:%f, inclination:%f",length_of_gap, gap_inclination);
          }
        else
          {
            length_of_gap = distance(left_corner.getX(), right_corner.getX(),
                                   right_corner.getY() + right_corner.getZ(), 
                                   left_corner.getY());
            gap_inclination = atan2((right_corner.getY() + right_corner.getZ()) -left_corner.getY() , right_corner.getX() - left_corner.getX());
            ROS_INFO("right half lower, length:%f, inclination:%f",length_of_gap, gap_inclination);
          }
      }
    else if(overlap_type == X_AXIS_OVERLAP)
      {
        if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
          {
            length_of_gap = right_corner.getY() - left_corner.getY() - left_corner.getZ();
            ROS_INFO("left half lower, length:%f",length_of_gap);
          }
        else
          { 
            length_of_gap =  left_corner.getY() - right_corner.getZ() - right_corner.getY();
            ROS_INFO("right half lower, length:%f",length_of_gap);
          }
      }
    else if(overlap_type == Y_AXIS_OVERLAP)
      {
        length_of_gap = right_corner.getX() - left_corner.getX();
        ROS_INFO("length:%f",length_of_gap);
      }

    if(length_of_gap > l + d)
      {
        ROS_WARN("traverse without transform");
        //TODO: ompl temporary
        planning_mode_ = ONLY_BASE_MODE + ORIGINAL_MODE;

        ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
        ompl::base::RealVectorBounds motion_bounds(2);
        motion_bounds.low[0] = start_state[0] - 0.5; //margin
        motion_bounds.low[1] = (start_state[1] < goal_state[1])?(start_state[1] - 0.5):(goal_state[1] - 0.5);
        motion_bounds.high[0] = goal_state[0] + 0.5;;
        motion_bounds.high[1] = (start_state[1] < goal_state[1])?(goal_state[1] + 0.5):(start_state[1] + 0.5);
        se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);

        hydra_space_ = se2;
        hydra_space_->as<ompl::base::SE2StateSpace>()->setValidSegmentCountFactor(200);
        space_information_  = boost::shared_ptr<ompl::base::SpaceInformation> (new ompl::base::SpaceInformation(hydra_space_));
        space_information_->setup();
        space_information_->setStateValidityChecker(boost::bind(&PlanningScene::isStateValid, this, _1));
        space_information_->setStateValidityCheckingResolution(0.03); // 3%
        space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
        space_information_->setup();

        ompl::base::ScopedState<> start(hydra_space_);
        ompl::base::ScopedState<> goal(hydra_space_);

        start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state[0], start_state[1]);
        start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_state[2]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state[0], goal_state[1]);
        goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_state[2]);



        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(space_information_));
        pdef->setStartAndGoalStates(start, goal);

        ompl::base::PlannerPtr planner;
        if(ompl_mode_ == RRT_START_MODE)
          {
            pdef->setOptimizationObjective(getPathLengthObjective(space_information_));
            planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
          }
        else if(ompl_mode_ == LBKPIECE1_MODE)
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::LBKPIECE1(space_information_));
          }
        else
          {
            planner = ompl::base::PlannerPtr(new ompl::geometric::SBL(space_information_));
          }
        planner->setProblemDefinition(pdef);
        planner->setup();
        space_information_->printSettings(std::cout);


        solved_ = false;
        ompl::base::PlannerStatus solved = planner->solve(3600.0);
        if (solved)
          {
            solved_ = true;
            path_ = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            path_->print(std::cout);

            //visualztion
            plan_states_ = new ompl::base::StateStorage(hydra_space_);
            int index = (int)(boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getStateCount());
            for(int i = 0; i < index; i++)
              {
                ompl::base::State *state1;
                ompl::base::State *state2;
            
                if(i == 0)
                  state1 = start.get();
                else 
                  state1 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i - 1);
        
                state2 = boost::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i);

                plan_states_->addState(state1);
                int nd = hydra_space_->validSegmentCount(state1, state2);
                if (nd > 1)
                  {
                    ompl::base::State *interpolated_state = space_information_->allocState();
                    for (int j = 1 ; j < nd ; ++j)
                      {
                        hydra_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                        plan_states_->addState(interpolated_state);
                      }
                  }
                plan_states_->addState(state2);
              }
          }
        else
          std::cout << "No solution found" << std::endl;

        return true;
      }
    else if(length_of_gap < d)
      {
        ROS_WARN("can not traverse");
        return false;
      }

    //centroid
    int l_r_flag = 0;
    tf::Vector3 shorter_corner;
    float test_corner_x;
    float test_corner_y_down;
    float test_corner_y_top;
    if(left_corner.getZ() <= right_corner.getZ()) 
      {
        shorter_corner = left_corner;
        test_corner_x = right_corner.getX() - left_corner.getX();
        test_corner_y_down = right_corner.getY() - (left_corner.getY() + left_corner.getZ()/2);
        test_corner_y_top = right_corner.getY() - (left_corner.getY() + left_corner.getZ()/2) + right_corner.getZ();

        l_r_flag = 0; //left
        ROS_INFO("left shorter, test_corner_x:%f, test_corner_y_down:%f, test_corner_y_top:%f",test_corner_x, test_corner_y_down, test_corner_y_top);
      }
    else 
      {
        shorter_corner = right_corner;
        test_corner_x = left_corner.getX() - right_corner.getX();
        test_corner_y_down = left_corner.getY() - (right_corner.getY() + right_corner.getZ()/2);
        test_corner_y_top = left_corner.getY() - (right_corner.getY() + right_corner.getZ()/2) + left_corner.getZ();
        l_r_flag = 1; //right
        ROS_INFO("right shorter, test_corner_x:%f, test_corner_y_down:%f, test_corner_y_top:%f",test_corner_x, test_corner_y_down, test_corner_y_top);

      }

    float h = shorter_corner.getZ();
    bool collision_flag = false;
    int centroid_type = 0; //0: ellipse, -1: under_parabola, 1: top_parabola

    float a = h / 2 + (sqrt(2)  -1 ) * d;
    float b = (h + 2 * d) / 2;
    

    if(l_r_flag == 0)//left
      {
        //*** visualizatio of the centroid
        //ellipse
        for(float theta = -M_PI/2; theta < M_PI/2; theta += M_PI /100)
          {
            float ellipse_x = cos(theta) * a + d + left_corner.getX();
            float ellipse_y = sin(theta) * b + left_corner.getY() + left_corner.getZ()/2;
            gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getY();
            geometry_msgs::Pose pose;
            pose.position.x = gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getX();
            pose.position.y = gap2WorldCoord(tf::Vector3(ellipse_x,ellipse_y,0)).getY();
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose);
          }
        //parabola
        for(float theta =  0; theta < M_PI/4; theta += M_PI /100)
          {
            float parabola_y_top = (2*l - d/2) * theta + d *( theta + 1) + h / 2;
            float parabola_x =  - (2*l - d/2) * theta * theta + d * (1 -theta) + left_corner.getX();
            float parabola_y_down = -parabola_y_top + left_corner.getY() + left_corner.getZ()/2;
            parabola_y_top = parabola_y_top + left_corner.getY() + left_corner.getZ()/2;

            geometry_msgs::Pose pose1, pose2;
            pose1.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getX();
            pose1.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getY();
            pose1.position.z = 0.0;
            pose1.orientation.w = 1.0;
            pose2.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getX();
            pose2.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getY();
            pose2.position.z = 0.0;
            pose2.orientation.w = 1.0;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose1);
            collision_object.primitive_poses.push_back(pose2);

            geometry_msgs::Pose pose3, pose4;
            parabola_y_top = (2*l - d/2) * sin(theta) + sqrt(2) * d * sin(M_PI/4 + theta) + h / 2;
            parabola_x =  - (2*l - d/2) * sin(theta) * tan(theta) + sqrt(2) * d * cos(M_PI/4 + theta) + left_corner.getX();
            parabola_y_down = -parabola_y_top + left_corner.getY() + left_corner.getZ()/2;
            parabola_y_top = parabola_y_top + left_corner.getY() + left_corner.getZ()/2;

            pose3.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getX();
            pose3.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_top,0)).getY();
            pose3.position.z = 0.0;
            pose3.orientation.w = 1.0;
            pose4.position.x = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getX();
            pose4.position.y = gap2WorldCoord(tf::Vector3(parabola_x,parabola_y_down,0)).getY();
            pose4.position.z = 0.0;
            pose4.orientation.w = 1.0;

            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.01;
            primitive.dimensions[1] = 0.01;
            primitive.dimensions[2] = 1;
            collision_object.primitives.push_back(primitive);
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose3);
            collision_object.primitive_poses.push_back(pose4);

          }


#if 0
        collision_object.operation = collision_object.ADD;
        planning_scene->processCollisionObjectMsg(collision_object);
        moveit_msgs::PlanningScene planning_scene_msg;
        planning_scene->getPlanningSceneMsg(planning_scene_msg);
        planning_scene_msg.is_diff = true;
        planning_scene_diff_pub_.publish(planning_scene_msg);
#endif

        if(test_corner_x < h/2 + sqrt(2) * d)
          {//else is ellipse
            ROS_INFO("left fit some centroid curve");
            if(test_corner_x >= d)
              {//ellipse
                ROS_INFO("left fit ellipse centroid curve");

                // y^2 = b^2 - b^2 / a^2 * (x -d )^2

                float ellipse_y1 = sqrt(b*b*(1- (test_corner_x - d) * (test_corner_x - d) / (a * a)));
                float ellipse_y2 = -ellipse_y1;

                if(! ((test_corner_y_top > ellipse_y1 && test_corner_y_down > ellipse_y1) || (test_corner_y_top < ellipse_y2 && test_corner_y_down < ellipse_y2)))
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
                ROS_INFO("ellipse path, ellipse_y1:%f, a:%f, b:%f",ellipse_y1, a, b);
                centroid_type = 0; //ellipse
              }
            else
              { // parabola
                float parabola_y1 = d + (2*l - d/2 + d) * (sqrt(d*d/(4*(2*l - d/2)*(2*l - d/2)) + (d-test_corner_x)/(2*l - d/2)) - d/2/(2*l - d/2)) + h/2;
                float parabola_y2 = -parabola_y1;

                ROS_INFO("parabola path, parabola_y1:%f",parabola_y1);
                if(test_corner_y_top > parabola_y1 && test_corner_y_down > parabola_y1)
                  {//top parabola
                    centroid_type = 1; //top parabola
                    ROS_INFO("top parabola");
                  }
                else if(test_corner_y_top < parabola_y2 && test_corner_y_down < parabola_y2)
                  {
                    //under parabola
                    ROS_INFO("under parabola");
                    centroid_type = -1; //under parabola
                  }
                else
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
              }
          }
        else
          {
            ROS_INFO("left,x out of all centroid, no collision, fit ellipse path");
          }
      }
    else//right
      {
        if(test_corner_x  > -(h/2 + sqrt(2) * d))
          {//else is ellipse
            ROS_INFO("right fit some centroid curve");
            if(test_corner_x <= -d)
              {//ellipse
                ROS_INFO("right fit ellipse centroid curve");
                //float a = h / 2 + (sqrt(2)  -1 ) * d;
                //float b = (h + 2 * d) / 2;
                // y^2 = b^2 - b^2 / a^2 * (x -d )^2
                float ellipse_y1 = sqrt(b*b*(1- (test_corner_x + d) * (test_corner_x + d) / (a * a)));
                float ellipse_y2 = -ellipse_y1;

                if(! ((test_corner_y_top > ellipse_y1 && test_corner_y_down > ellipse_y1) && (test_corner_y_top < ellipse_y2 && test_corner_y_down < ellipse_y2)))
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
                ROS_INFO("ellipse path, ellipse_y1:%f, a:%f, b:%f",ellipse_y1, a, b);
                centroid_type = 0; //ellipse
              }
            else
              { // parabola
                float parabola_y1 = d + (2*l - d/2 + d) * (sqrt(d*d/(4*(2*l - d/2)*(2*l - d/2)) + (d+test_corner_x)/(2*l - d/2)) - d/2/(2*l - d/2)) + h/2;
                float parabola_y2 = -parabola_y1;
                ROS_INFO("parabola path, parabola_y1:%f",parabola_y1);
                if(test_corner_y_top > parabola_y1 && test_corner_y_down > parabola_y1)
                  {//top parabola
                    centroid_type = 1; //top parabola
                    ROS_INFO("top parabola");
                  }
                else if(test_corner_y_top < parabola_y2 && test_corner_y_down < parabola_y2)
                  {
                    //under parabola
                    ROS_INFO("under parabola");
                    centroid_type = -1; //under parabola
                  }
                else
                  {
                    ROS_WARN("traverse with two step of transform");
                    collision_flag = true;
                  }
              }
          }
        else
          {
            ROS_INFO("right,x out of all centroid, no collision, fit ellipse path");
          }
      }

    if(collision_flag)
      {//two step of transform
        float middle_point_x, middle_point_y;
        float d_up, d_down;
        float x1,x2,x3,x4;
        float y1,y2,y3,y4;
        int transform_type = 0; 
        float direction = 0; //top shape(sgn), down shape(odd/even)

        ROS_INFO("two step traverse");

        if(overlap_type == NO_OVERLAP)
          {//NO Overlap

            middle_point_x = (left_corner.getX() + right_corner.getX())/2;
             middle_point_y = (gap_inclination > 0)?(left_corner.getY() + right_corner.getY() + left_corner.getZ())/2: (left_corner.getY() + right_corner.getY() + right_corner.getZ())/2;

             d_up = (gap_inclination > 0)?right_corner.getZ() * cos(gap_inclination):left_corner.getZ() * cos(gap_inclination);

             d_down = (gap_inclination > 0)?left_corner.getZ() * cos(gap_inclination):right_corner.getZ() * cos(gap_inclination);

            if((gap_inclination > 0 && gap_inclination <= M_PI/4)||
               (gap_inclination > -M_PI && gap_inclination <= -M_PI/4))
              transform_type = -2; //"2" model
            else
              transform_type = 1; // "s" model


             x1 = middle_point_x + d_down * sin(gap_inclination);
             y1 = middle_point_y - d_down * cos(gap_inclination);
             x2 = middle_point_x - 2 * l * sin(gap_inclination);
             y2 = middle_point_y + 2 * l * cos(gap_inclination);
             x3 = middle_point_x - l * sin(gap_inclination) + l * abs(transform_type)/transform_type * cos(gap_inclination);
             y3 = middle_point_y + l * cos(gap_inclination) + l * abs(transform_type)/transform_type * sin(gap_inclination) ;
             x4 = middle_point_x - (3*l + d_up) * sin(gap_inclination) + l * abs(transform_type)/transform_type * cos(gap_inclination);
             y4 = middle_point_y + (3*l + d_up) * cos(gap_inclination) + l * abs(transform_type)/transform_type * sin(gap_inclination) ;

            direction = gap_inclination;
          }
        else if (overlap_type == X_AXIS_OVERLAP)
          {
            if(left_corner.getX() - right_corner.getX() >= (2*l-d))
              {
                ROS_WARN("can not traverse");
                return false;
              }
            else
              {
                if(left_corner.getY() + left_corner.getZ() < right_corner.getY())
                  {//left down
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + right_corner.getY())/2;
                    transform_type = 1; // "s" model
                    x1 = left_corner.getX() + 0.1; //+ 0.1mm margin
                    y1 = middle_point_y;
                    x2 = middle_point_x - 2*l;
                    y2 = middle_point_y;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = right_corner.getX() - 3*l;
                    y4 = middle_point_y + l;
                    direction = M_PI/2;
                  }
                else
                  {//right down
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + right_corner.getZ() + right_corner.getY())/2;
                    transform_type = -2; // "2" model
                    x1 = right_corner.getX() - 0.1; //+ 0.1mm margin
                    y1 = middle_point_y;
                    x2 = middle_point_x + 2*l;
                    y2 = middle_point_y;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = left_corner.getX() + 3*l;
                    y4 = middle_point_y + l;
                    direction = -M_PI/2;
                  }
              }
          }
        else if (overlap_type == Y_AXIS_OVERLAP)
          {
            direction = 0;
            if(left_corner.getY() + left_corner.getZ() < right_corner.getY() + right_corner.getZ())
              {
                if(left_corner.getY() < right_corner.getY())
                  {//l_up + r_down
                    ROS_INFO("l up and r down");
                    if(left_corner.getY() + left_corner.getZ() - right_corner.getY() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + right_corner.getY())/2;
                    transform_type = -2; // "2" model
                    x1 = middle_point_x;
                    y1 = left_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x - l;
                    y4 = right_corner.getY() + right_corner.getZ() + 3*l + 0.1; //0.2m padding

                  }
                else
                  {//l_up + l_down
                    ROS_INFO("l up and l down");
                    if(left_corner.getZ() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + left_corner.getZ() + left_corner.getY())/2;
                    transform_type = -1; // "inv c" model
                    x1 = middle_point_x;
                    y1 = left_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x - l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x - l;
                    y4 = right_corner.getY() + right_corner.getZ() + 3*l + 0.1; //0.1m padding
                  }
              }
            else
              {
                if(left_corner.getY() > right_corner.getY())
                  {//l_down + r_up
                    ROS_INFO("l down and r up");
                    if(right_corner.getY() + right_corner.getZ() - left_corner.getY() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (left_corner.getY() + right_corner.getZ() + right_corner.getY())/2;

                    transform_type = 1; // "s" model
                    x1 = middle_point_x;
                    y1 = right_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x + l;
                    y4 = left_corner.getY() + left_corner.getZ() + 3*l + 0.1; //0.2m padding
                  }
                else
                  {//r_up + r_down
                    ROS_INFO("r down and r up");
                    if(right_corner.getZ() >= (2*l-d))
                      {
                        ROS_WARN("can not traverse");
                        return false;
                      }
                    middle_point_x = (left_corner.getX() + right_corner.getX())/2;
                    middle_point_y = (right_corner.getY() + right_corner.getZ() + right_corner.getY())/2;

                    transform_type = 2; // "c" model
                    x1 = middle_point_x;
                    y1 = right_corner.getY();
                    x2 = middle_point_x;
                    y2 = middle_point_y + 2*l;
                    x3 = middle_point_x + l;
                    y3 = middle_point_y + l;
                    x4 = middle_point_x + l;
                    y4 = left_corner.getY() + left_corner.getZ() + 3*l + 0.1; //0.2m padding
                  }
              }
          }

        float delta_x, delta_y;
        float joint1_delta, joint2_delta, joint3_delta, theta_delta;

        tf::Vector3 xy1_world_coord = gap2WorldCoord(tf::Vector3(x1, y1, 0));
        tf::Vector3 xy2_world_coord = gap2WorldCoord(tf::Vector3(x2, y2, 0));
        tf::Vector3 xy3_world_coord = gap2WorldCoord(tf::Vector3(x3, y3, 0));
        tf::Vector3 xy4_world_coord = gap2WorldCoord(tf::Vector3(x4, y4, 0));
        tf::Vector3 middle_point_world_coord = gap2WorldCoord(tf::Vector3(middle_point_x, middle_point_y, 0));


        //*** step 1 transform
        theta_delta  = direction - start_state[2];
        if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
        else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
        theta_delta /= 100;

        joint1_delta = (0 - start_state[3]) /100;
        joint2_delta = (0 - start_state[4]) /100;
        joint3_delta = (M_PI /2 * ((transform_type)%2?-1:1) - start_state[5]) /100;



        for(int i = 0; i < 100; i++)
          {//  bad trnasform TODO
            conf_values new_state;
            new_state.x = start_state[0];
            new_state.y = start_state[1];
            new_state.theta =  start_state[2] + i * theta_delta;
            new_state.joint1 = start_state[3] + i * joint1_delta;
            new_state.joint2 = start_state[4] + i * joint2_delta;
            new_state.joint3 = start_state[5] + i * joint3_delta;
            original_path.push_back(new_state);
          }


        //*** step2: access to first pint
        delta_x = (xy1_world_coord.getX() - start_state[0])/100;
        delta_y = (xy1_world_coord.getY() - start_state[1])/100;

        for(int i = 0; i < 100; i++)
          {// bad access TODO
            conf_values new_state;
            new_state.x = start_state[0] + delta_x * i;
            new_state.y = start_state[1] + delta_y * i;
            new_state.theta =  direction;
            new_state.joint1 = 0;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }


        //step 3: first insert
        delta_x = (xy2_world_coord.getX() - xy1_world_coord.getX())/100;
        delta_y = (xy2_world_coord.getY() - xy1_world_coord.getY())/100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy1_world_coord.getX() + delta_x * i;
            new_state.y = xy1_world_coord.getY() + delta_y * i;
            new_state.theta =  direction;
            new_state.joint1 = 0;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }



        //*** step4: transform in the gap
        joint1_delta = M_PI/2 * abs(transform_type)/transform_type/100;
        joint3_delta = (0 - M_PI /2 * ((transform_type)%2?-1:1))/100;

        float x_pivot = (xy2_world_coord.getX() + middle_point_world_coord.getX())/2;
        float y_pivot = (xy2_world_coord.getY() + middle_point_world_coord.getY())/2;
        float changed_direction = direction - fabs(joint1_delta)/joint1_delta * M_PI/2;
        float changed_joint1 = fabs(joint1_delta)/joint1_delta * M_PI / 2;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = x_pivot + l * cos(direction - joint1_delta * i);
            new_state.y = y_pivot + l * sin(direction - joint1_delta * i);
            new_state.theta =  direction - joint1_delta * i;
            new_state.joint1 = joint1_delta * i;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1);
            original_path.push_back(new_state);
          }

        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy3_world_coord.getX();
            new_state.y = xy3_world_coord.getY();
            new_state.theta =  changed_direction;
            new_state.joint1 = changed_joint1;
            new_state.joint2 = 0;
            new_state.joint3 = M_PI /2 * ((transform_type)%2?-1:1) + joint3_delta *i;
            original_path.push_back(new_state);
          }


        //*** step6: escape
        delta_x = (xy4_world_coord.getX() - xy3_world_coord.getX())/100;
        delta_y = (xy4_world_coord.getY() - xy3_world_coord.getY())/100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy3_world_coord.getX() + delta_x * i;
            new_state.y = xy3_world_coord.getY() + delta_y * i;
            new_state.theta =  changed_direction;;
            new_state.joint1 = changed_joint1;
            new_state.joint2 = 0;
            new_state.joint3 = 0;
            original_path.push_back(new_state);
          }


        //*** step 7: transform
        joint1_delta = (goal_state[3] - fabs(joint1_delta)/joint1_delta * M_PI/2) /100;
        joint2_delta = goal_state[4] /100;
        joint3_delta = goal_state[5] /100;
        for(int i = 0; i < 100; i++)
          {
            conf_values new_state;
            new_state.x = xy4_world_coord.getX();
            new_state.y = xy4_world_coord.getY();
            new_state.theta =  changed_direction;
            new_state.joint1 = changed_joint1 + joint1_delta * i;
            new_state.joint2 = joint2_delta * i;
            new_state.joint3 = joint3_delta * i;
            original_path.push_back(new_state);
          }

        delta_x = (goal_state[0] - xy4_world_coord.getX())/100;
        delta_y = (goal_state[1] - xy4_world_coord.getY())/100;
        theta_delta = goal_state[2] - changed_direction;
        if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
        else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
        theta_delta /= 100;


        //*** step7: to goal 
        for(int i = 0; i < 100; i++)
          {//bad TODO
            conf_values new_state;
            new_state.x = xy4_world_coord.getX() + delta_x * i;
            new_state.y = xy4_world_coord.getY() + delta_y * i;
            new_state.theta =  changed_direction + theta_delta * i;
            new_state.joint1 = goal_state[3];
            new_state.joint2 = goal_state[4];
            new_state.joint3 = goal_state[5];
            original_path.push_back(new_state);
          }

        return true;
      }
    else
      {//ku model traverse
        float delta_x, delta_y;
        float joint1_delta, joint2_delta, joint3_delta, theta_delta;

        if(centroid_type == 0)
          {//ellipse path
            if(l_r_flag == 0)
              {//left

                //*** step1: transform
                theta_delta  = 0 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;
                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //*** step2: access1
                tf::Vector3 access_point_world_coord 
                  = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2, left_corner.getY() ,0));
                delta_x = (access_point_world_coord.getX() - start_state[0]) /100;
                delta_y = (access_point_world_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; i++)
                  {//TODO: bad aces => best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i* delta_x + start_state[0];
                    new_state.y = i* delta_y + start_state[1];
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                delta_y = (2*l - d/2) / 100;
                for(int i = 0; i < 100; i++)
                  {
                    tf::Vector3 moving_world_coord 
                      = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2, left_corner.getY() + delta_y * i ,0));
                    conf_values new_state;
                    new_state.x = moving_world_coord.getX();
                    new_state.y = moving_world_coord.getY();
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //***step 3: traverse
                float x_ellipse, y_ellipse;
                for(float theta = M_PI; theta >=0; theta -= M_PI/100)
                  {
                    float x_theta = h/2 * sin(theta) + sqrt(2) *d * sin(M_PI/4 + theta/2);
                    float y_theta = h/2 * cos(theta) + sqrt(2) *d * cos(M_PI/4 + theta/2);
                    x_ellipse = x_theta - (2*l+d/2) * sin(M_PI/2 - theta/2) - d/2*cos(M_PI/2 - theta/2) + left_corner.getX();
                    y_ellipse = y_theta + (2*l+d/2) * cos(M_PI/2 - theta/2) - d/2*sin(M_PI/2 - theta/2) + (left_corner.getY() + left_corner.getZ()/2);

                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_ellipse,y_ellipse,0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_ellipse,y_ellipse,0)).getY();
                    new_state.theta = M_PI/2 - theta/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                ROS_INFO("x:%f,y:%f", x_ellipse, y_ellipse);
                //*** step4: escape
                float x_escape = x_ellipse;
                float y_escape = left_corner.getY() + left_corner.getZ() + 2*l + 0.1; //0.1m margin
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float y_tmp = y_ellipse;
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm 
                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getY();
                    new_state.theta =  M_PI /2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-M_PI/2)) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - M_PI/2;
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  M_PI/2 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
            else
              {//right  ** mocap(link39 base planning!! **
                /*
                  start_state[0]: mocap_x
                  start_state[1]: mocap_y
                  start_state[2]: mocap_psi
                  start_state[3]: joint1
                  start_state[4]: joint2
                  start_state[5]: joint2
 
                  mocap center to link center, according to link coord 
                  (mocap_center_to_link_center_x_, mocap_center_to_link_center_y_)
                */
                conf_values new_state_link1, new_state_link3;

                //*** step1: transform
                theta_delta  = M_PI/2 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;
                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    //for link1 visualization
                    new_state_link1.x = start_state[0] + (mocap_center_to_link_center_x_ + l/2) * cos(new_state_link1.theta) - mocap_center_to_link_center_y_ * sin(new_state_link1.theta) + l * cos(new_state_link1.theta - new_state_link1.joint2) + l * cos(new_state_link1.theta - new_state_link1.joint1 - new_state_link1.joint2);
                    new_state_link1.y = start_state[1] + (mocap_center_to_link_center_x_ + l/2) * sin(new_state_link1.theta) + mocap_center_to_link_center_y_ * cos(new_state_link1.theta) + l * sin(new_state_link1.theta - new_state_link1.joint2) + l * sin(new_state_link1.theta - new_state_link1.joint1 - new_state_link1.joint2);
                    new_state_link1.theta =  start_state[2] + i * (theta_delta) - new_state_link1.joint1 - new_state_link1.joint2;
                    new_state_link1.joint1 = start_state[3] + i * joint1_delta;
                    new_state_link1.joint2 = start_state[4] + i * joint2_delta;
                    new_state_link1.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state_link1);
                    //for link which contains mocap , that is, the link3 in this system
                    new_state_link3.x = start_state[0] + mocap_center_to_link_center_x_ * cos(start_state[2]) - mocap_center_to_link_center_y_ * sin(start_state[2]);
                    new_state_link3.y = start_state[1] + mocap_center_to_link_center_x_ * sin(start_state[2]) + mocap_center_to_link_center_y_ * cos(start_state[2]);
                    new_state_link3.theta =  start_state[2] + i * theta_delta;
                    new_state_link3.joint1 = start_state[3] + i * joint1_delta;
                    new_state_link3.joint2 = start_state[4] + i * joint2_delta;
                    new_state_link3.joint3 = start_state[5] + i * joint3_delta;
                    real_robot_path_.push_back(new_state_link3);
                  }
                //**** for real robot move
                //TODO: trasnformation();


                //*** step2: access1
                float init_link1_x = new_state_link1.x;
                float init_link1_y = new_state_link1.y;
                float init_link3_x = new_state_link3.x;
                float init_link3_y = new_state_link3.y;
                tf::Vector3 access_point_world_coord 
                  = gap2WorldCoord(tf::Vector3(right_corner.getX() - d/2, right_corner.getY() ,0));
                delta_x = (access_point_world_coord.getX() - init_link1_x) /100;
                delta_y = (access_point_world_coord.getY() - init_link1_y) /100;
                for(int i = 0; i < 100; i++)
                  {
                    //for link1 visualization
                    new_state_link1.x = i* delta_x + init_link1_x;
                    new_state_link1.y = i* delta_y + init_link1_y;
                    new_state_link1.theta =  0;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = i* delta_x + init_link3_x;
                    new_state_link3.y = i* delta_y + init_link3_y;
                    new_state_link3.theta =  M_PI/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }

                init_link1_x = new_state_link1.x;
                init_link1_y = new_state_link1.y;
                init_link3_x = new_state_link3.x;
                init_link3_y = new_state_link3.y;
                access_point_world_coord = gap2WorldCoord(tf::Vector3(right_corner.getX() - d/2, right_corner.getY() + 2*l -d/2, 0));
                delta_x = (access_point_world_coord.getX() - init_link1_x) /100;
                delta_y = (access_point_world_coord.getY() - init_link1_y) /100;
                for(int i = 0; i < 100; i++)
                  {
                    //for link1 visualization
                    new_state_link1.x = i* delta_x + init_link1_x;
                    new_state_link1.y = i* delta_y + init_link1_y;
                    new_state_link1.theta =  0;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = i* delta_x + init_link3_x;
                    new_state_link3.y = i* delta_y + init_link3_y;
                    new_state_link3.theta =  M_PI/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }


                //***step 3: traverse
                float x_link1, y_link1, x_link3, y_link3;
                for(float theta = M_PI; theta >=0; theta -= M_PI/100)
                  {
                    float x_theta = -h/2 * sin(theta);
                    float y_theta = h/2 * cos(theta);
                    //for link1 visualization
                    x_link1 = x_theta +  (2*l-d/2) * cos(theta/2) - d/2*sin(theta/2) + right_corner.getX();
                    y_link1 = y_theta + (2*l-d/2) * sin(theta/2) + d/2*cos(theta/2) + right_corner.getY() + right_corner.getZ()/2;

                    new_state_link1.x = gap2WorldCoord(tf::Vector3(x_link1,y_link1,0)).getX();
                    new_state_link1.y = gap2WorldCoord(tf::Vector3(x_link1,y_link1,0)).getY();
                    new_state_link1.theta = -M_PI/2 + theta/2;
                    new_state_link1.joint1 = 0;
                    new_state_link1.joint2 = M_PI/2;
                    new_state_link1.joint3 = 0;
                    original_path.push_back(new_state_link1);
                    //for link3 visualization
                    new_state_link3.x = new_state_link1.x + fdfdsf(bakui, mocap link)
                    new_state_link3.y = 
                    new_state_link3.theta = theta/2;
                    new_state_link3.joint1 = 0;
                    new_state_link3.joint2 = M_PI/2;
                    new_state_link3.joint3 = 0;
                    real_robot_path_.push_back(new_state_link3);
                  }

                //*** step4: escape
                float x_escape = x_ellipse;
                float y_escape = right_corner.getY() + right_corner.getZ() + 2*l + 0.1; //0.1m margin
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float y_tmp = y_ellipse;
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm 
                    conf_values new_state;
                    new_state.x = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getX();
                    new_state.y = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0)).getY();
                    new_state.theta =  -M_PI /2;
                    new_state.joint1 = 0;
                    new_state.joint2 = M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - M_PI/2) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  -M_PI/2;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - ( - M_PI/2);
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  -M_PI/2 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
          }
        else if(centroid_type == -1)
          {//under parabola
            if(l_r_flag == 0)
              {//left
                //*** step1: transform
                theta_delta  = -M_PI/4 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;

                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //step 2: access
                tf::Vector3 access_coord = gap2WorldCoord(tf::Vector3(left_corner.getX() + d/2*sqrt(2)/2, left_corner.getY() - d/2*sqrt(2)/2,0));
                float delta_x = (access_coord.getX() - start_state[0]) /100;
                float delta_y = (access_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; i++)
                  {//TODO: best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i*delta_x + start_state[0];
                    new_state.y = i*delta_y + start_state[1];
                    new_state.theta =  -M_PI/4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //step 3: traverse
                float x_parabola, y_parabola;
                for(float theta = M_PI/4; theta >=0; theta -=M_PI/400)
                  {
                    float y_theta = (2*l-d/2) * sin(theta) + sqrt(2)*d* sin(M_PI/4 + theta) + h/2 ;
                    float x_theta = -(2*l-d/2) * sin(theta) * tan(theta) + sqrt(2)* d * cos(M_PI/4 + theta);
                    x_parabola = x_theta + (2*l+d/2) * sin(theta) - d/2*cos(theta) + left_corner.getX();
                    y_parabola = -y_theta + (2*l+d/2) * cos(theta) + d/2*sin(theta) + (left_corner.getY() + left_corner.getZ()/2);

                    tf::Vector3 parabola_coord = gap2WorldCoord(tf::Vector3(x_parabola, y_parabola, 0));
                    conf_values new_state;
                    new_state.x = parabola_coord.getX();
                    new_state.y = parabola_coord.getY();
                    new_state.theta = -theta;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //step 4: escape
                float x_escape = left_corner.getX() + 2 * l + 0.1; //0.1m margin
                float y_escape = y_parabola;
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                float x_tmp = x_parabola;
                while(1)
                  {
                    if(x_tmp >= x_escape) break;
                    x_tmp += 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_tmp, y_escape, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  0;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-M_PI/2)) /100;
                joint3_delta = goal_state[5] /100;
                
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  0;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }
              }
            else
              {//right 
                //TODO

              }
          }
        else if(centroid_type == 1)
          {//top parabola
            if(l_r_flag == 0)
              {//left
                //*** step1: transform
                theta_delta  = M_PI/2 - start_state[2];
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;

                joint1_delta = (0 - start_state[3]) /100;
                joint2_delta = (-M_PI/2 - start_state[4]) /100;
                joint3_delta = (0 - start_state[5]) /100;

                for(int i = 0; i <= 100; i++)
                  {//TODO: bad transform
                    conf_values new_state;
                    new_state.x = start_state[0];
                    new_state.y = start_state[1];
                    new_state.theta =  start_state[2] + i * theta_delta;
                    new_state.joint1 = start_state[3] + i * joint1_delta;
                    new_state.joint2 = start_state[4] + i * joint2_delta;
                    new_state.joint3 = start_state[5] + i * joint3_delta;
                    original_path.push_back(new_state);
                  }

                //step 2: access
                tf::Vector3 access_point_world_coord = gap2WorldCoord(tf::Vector3(left_corner.getX(), left_corner.getY() + left_corner.getZ()+ d/2,0));
                delta_x = (access_point_world_coord.getX() - start_state[0]) /100;
                delta_y = (access_point_world_coord.getY() - start_state[1]) /100;
                for(int i = 0; i < 100; ++i)
                  {//TODO: best path planning(inclination)
                    conf_values new_state;
                    new_state.x = i*delta_x + start_state[0];
                    new_state.y = i*delta_y + start_state[1];
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                delta_x = -(2*l -d) / 100;
                for(int i = 0; i < 100; ++i)
                  {//access2
                    tf::Vector3 moving_world_coord 
                      = gap2WorldCoord(tf::Vector3(left_corner.getX() + delta_x * i, left_corner.getY() + left_corner.getZ()+ d/2,0));

                    conf_values new_state;
                    new_state.x = moving_world_coord.getX();
                    new_state.y = moving_world_coord.getY();
                    new_state.theta =  M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                //*** step3: traverse
                float x_parabola, y_parabola;
                for(float theta = 0; theta <=M_PI/4; theta +=M_PI/400)
                  {

                    float y_theta = (2*l-d/2) * sin(theta) + sqrt(2)*d* sin(M_PI/4 + theta) + h/2;
                    float x_theta = -(2*l-d/2) * sin(theta) * tan(theta) + sqrt(2)* d * cos(M_PI/4 + theta);
                    x_parabola = x_theta - (2*l+d/2) * cos(theta) + d/2*sin(theta) + left_corner.getX();
                    y_parabola = y_theta - (2*l+d/2) * sin(theta) - d/2*cos(theta) + (left_corner.getY() + left_corner.getZ()/2);

                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_parabola, y_parabola, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta = theta + M_PI/2;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }

                //*** step4: escape
                float x_escape = right_corner.getX() - 2 *sqrt(2)* l - 0.1; //0.1m margin
                float y_escape = y_parabola;
                float x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                float y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();

                float x_tmp = x_parabola;
                float y_tmp = y_parabola;
                while(1)
                  {
                    if(x_tmp <= x_escape) break;
                    x_tmp -= 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_tmp, y_escape, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }
                y_escape = y_parabola + 0.6; //0.6m
                x_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getX();
                y_escape_world = gap2WorldCoord(tf::Vector3(x_escape, y_escape, 0)).getY();
                while(1)
                  {
                    if(y_tmp >= y_escape) break;
                    y_tmp += 0.05;// 0.05 cm
                    tf::Vector3 world_coord = gap2WorldCoord(tf::Vector3(x_escape, y_tmp, 0));
                    conf_values new_state;
                    new_state.x = world_coord.getX();
                    new_state.y = world_coord.getY();
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = 0;
                    new_state.joint2 = -M_PI/2;
                    new_state.joint3 = 0;
                    original_path.push_back(new_state);
                  }


                //*** step5: transform
                joint1_delta = goal_state[3] /100;
                joint2_delta = (goal_state[4] - (-3* M_PI /4)) /100;
                joint3_delta = goal_state[5] /100;
                for(int i = 0; i < 100; ++i)
                  {
                    conf_values new_state;
                    new_state.x = x_escape_world;
                    new_state.y = y_escape_world;
                    new_state.theta =  3* M_PI /4;
                    new_state.joint1 = joint1_delta * i;
                    new_state.joint2 = -M_PI/2 + joint2_delta * i;
                    new_state.joint3 = joint3_delta * i;
                    original_path.push_back(new_state);
                  }

                //*** step6: to goal
                delta_x = (goal_state[0] - x_escape_world)/100;
                delta_y = (goal_state[1] - y_escape_world)/100;
                theta_delta = goal_state[2] - 3* M_PI /4;
                if(theta_delta > M_PI)  theta_delta = -2 * M_PI + theta_delta; 
                else if(theta_delta < -M_PI)  theta_delta = 2 * M_PI + theta_delta;
                theta_delta /= 100;
                for(int i = 0; i < 100; ++i)
                  {//bad TODO
                    conf_values new_state;
                    new_state.x = x_escape_world + delta_x * i;
                    new_state.y = y_escape_world + delta_y * i;
                    new_state.theta =  3* M_PI /4 + theta_delta * i;
                    new_state.joint1 = goal_state[3];
                    new_state.joint2 = goal_state[4];
                    new_state.joint3 = goal_state[5];
                    original_path.push_back(new_state);
                  }

                //TODO: to the goal!!
              }
            else
              {//right 
                //TODO
              }
          }
          return true;
      }
    ROS_ERROR("bad dectection");
    return false;
  }
