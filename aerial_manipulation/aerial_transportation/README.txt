grasp planning:
1. object config publishment
roslaunch aerial_transportation object.launch (object:=cylinder)
2. grasp planning
roslaunch aerial_transportation grasp_planning.launch
(please check the grasp_planning.yaml, file record option)
3. robot model
roslaunch aerial_robot_model display_xacro_without_joint_state_publisher.launch
4. rviz 
rosrun rviz rviz


grasp control(test):
1. object config publishment
roslaunch aerial_transportation object.launch (object:=cylinder)
2. grasp control(test)
roslaunch aerial_transportation grasp_control.launch (object:=cylinder)
(please check the grasp_control.yaml, "play_file_flag" and "control_test_flag" should be true)
3. robot model
roslaunch aerial_robot_model display_xacro_without_joint_state_publisher.launch
4. rviz 
rosrun rviz rviz


