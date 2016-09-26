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

grasp demonstration:
1. hydrus aerial_robot bringup launch
roslaunch hydrus_transform_control hydrus3.launch
2. hydrus grasp planning&control launch
box: roslaunch aerial_transportation aerial_transportation.launch plugin_name:=hydrus debug:=true
cylinder: roslaunch aer^Cl_transportation aerial_transportation.launch plugin_name:=hydrus debug:=true object:=cylinder
a. local pc: mocap for hydrus and object
roslaunch aerial_transportation mocap_assistant.launch 
P.S. please swith the mocap project in your windows PC according to the object(box or cylinder)
b. local pc: 
roslaunch aerial_robot_base joy_stick.launch

